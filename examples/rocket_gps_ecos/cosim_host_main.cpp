// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file cosim_host_main.cpp
/// @brief Ecos co-simulation host coupling Aetherion's TwoStageRocket.fmu to
/// Hemerion's GPS, IMU and BMP390 hardware-simulator FMUs.
///
/// Topology (see examples/rocket_gps_ecos/README.md):
///
///   TwoStageRocket.fmu ──(FMI variables, Ecos connections)──> hemerion_gps_fmu.fmu
///        truth: lat/lon/alt, NED velocity        │              noise + dynamics envelope
///               body rates, forces, mass         │              + UBX-NAV-PVT
///                                                │                    │ UDP 127.0.0.1:5762
///                                                ├──────────> hemerion_imu_fmu.fmu
///                                                │              noise + raw-count frames
///                                                │                    │ SPI over shared memory
///                                                └──────────> hemerion_bmp390_fmu.fmu
///                                                               ISA + inverse compensation
///                                                                     │ I2C over shared memory
///                                                                     v
///                                                          gps_flight_computer
///                                                    (GpsDriver/UbxParser + ImuSpiDriver/
///                                                     ImuPacketParser/convert_raw_to_si +
///                                                     Bmp390Driver/Bmp390Compensator,
///                                                     the same stacks that run on the
///                                                     STM32H743)
///
/// The rocket reports geodetic latitude/longitude in degrees and NED velocity
/// in m/s, which is what the GPS FMU takes, so every truth->receiver
/// connection is 1:1. (Before 0.13.0, Aetherion published those two ports as
/// `out.lat_rad`/`out.lon_rad` in radians and this host converted them with
/// Ecos connection modifiers; the example now requires >= 0.13.0 and binds
/// the degree-valued ports directly -- see CMakeLists.txt.)
///
/// The IMU FMU takes true body-frame specific force, which the rocket
/// publishes as `out.specificForce_*` -- (F_aero + F_thrust)/m at the CG,
/// gravitation excluded -- so that connects 1:1 as well, as do body rates
/// p/q/r. (Before 0.14.0 the plant exposed the ingredients but not the sum,
/// and this host divided them itself after every step; doing so meant
/// asserting host-side that thrust acts along body +X, which is the plant's
/// business, not the bench's.)
///
/// None of the sensor FMUs has FMI outputs -- their effect is the sensor bus
/// each part really uses: a UDP stand-in for the GPS receiver's UART, a
/// shared-memory SPI bus (sim/spi_shm) the IMU answers as a peripheral, and a
/// shared-memory I2C bus (sim/i2c_shm) the BMP390 answers as a register-
/// accurate part. All three are consumed by gps_flight_computer. The BMP390
/// takes exactly one FMI input -- truth altitude -- and no rate parameter: it
/// converts at whatever ODR the flight computer programs into its registers,
/// which is the point of simulating the part instead of the datasheet.
///
/// The GPS FMU's receiver dynamics envelope is configured here rather than
/// left at its defaults, because it is the single most visible thing about
/// this scenario: a launch vehicle configured as airborne <4 g loses its fix
/// through boost, and the COCOM export limits keep it dark above 18 km and
/// 515 m/s. See --dyn-model / --no-cocom.
///
/// Rocket truth is logged through Ecos' csv_writer so the flight computer's
/// decoded fixes and IMU samples can be compared against it
/// (plot_results.py).

#include "ecos/algorithm/fixed_step_algorithm.hpp"
#include "ecos/logger/logger.hpp"
#include "ecos/structure/simulation_structure.hpp"

// Deliberately one of Ecos' *internal* headers (its src/ tree, put on this
// target's include path by CMakeLists.txt). Preflighting FMU extraction is only
// worth anything if it runs the same code Ecos will -- reimplementing the
// choice of archiver here would drift, and drift in a diagnostic is worse than
// no diagnostic. The Ecos version is pinned, so if upstream ever moves this
// header the build fails loudly rather than the check silently rotting.
#include "util/unzipper.hpp"

#include "Hemerion/mag/world_magnetic_model.h"

#include "environment.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

using hemerion::examples::GpsErrorModel;
using hemerion::examples::kCorrelatedReceiver;
using hemerion::examples::kWhiteReceiver;

using hemerion::sensors::mag::to_body;
using hemerion::sensors::mag::WmmError;
using hemerion::sensors::mag::WmmField;
using hemerion::sensors::mag::WmmFieldBody;
using hemerion::sensors::mag::WorldMagneticModel;

// Compile-time defaults injected by CMakeLists.txt; both can be overridden on
// the command line, so an empty default (FMU not found at configure time) is
// not fatal until the run actually needs it.
#ifndef HEMERION_ROCKET_FMU_PATH
#define HEMERION_ROCKET_FMU_PATH ""
#endif
#ifndef HEMERION_GPS_FMU_PATH
#define HEMERION_GPS_FMU_PATH ""
#endif
#ifndef HEMERION_IMU_FMU_PATH
#define HEMERION_IMU_FMU_PATH ""
#endif
#ifndef HEMERION_BMP390_FMU_PATH
#define HEMERION_BMP390_FMU_PATH ""
#endif
#ifndef HEMERION_MMC5983MA_FMU_PATH
#define HEMERION_MMC5983MA_FMU_PATH ""
#endif

/// @brief A distinct, reproducible seed per sensor from one run seed.
///
/// Seeding every part with the same number would hand the barometer and the
/// radar altimeter the same first draw -- a correlation between independent
/// parts that no real hardware has, and a coincidence a figure should never
/// rest on. A cheap integer hash (splitmix64's finaliser) decorrelates them
/// while keeping the whole run a function of the one number the user typed.
///
/// A run seed of 0 means "draw a fresh instrument", and stays 0 for every
/// sensor: the FMUs read 0 as "use std::random_device", which is what an
/// unseeded run has always done.
[[nodiscard]] inline int derived_seed(int run_seed, int sensor_index)
{
  if (run_seed == 0)
  {
    return 0;
  }
  std::uint64_t x =
      static_cast<std::uint64_t>(run_seed) + 0x9E3779B97F4A7C15ULL * static_cast<std::uint64_t>(sensor_index + 1);
  x ^= x >> 30U;
  x *= 0xBF58476D1CE4E5B9ULL;
  x ^= x >> 27U;
  x *= 0x94D049BB133111EBULL;
  x ^= x >> 31U;
  // Keep it positive and clear of 0, which the FMUs read as "unseeded".
  return static_cast<int>((x % 2147483646ULL) + 1ULL);
}

struct Options
{
  std::filesystem::path rocket_fmu = HEMERION_ROCKET_FMU_PATH;
  std::filesystem::path gps_fmu = HEMERION_GPS_FMU_PATH;
  std::filesystem::path imu_fmu = HEMERION_IMU_FMU_PATH;
  std::filesystem::path baro_fmu = HEMERION_BMP390_FMU_PATH;
  std::filesystem::path mag_fmu = HEMERION_MMC5983MA_FMU_PATH;
  std::filesystem::path csv_path = "results/rocket_truth.csv";
  // NASA TM-2015-218675 Scenario 17 runs to 200 s: stage 1 burns out at 37.4 s, the vehicle coasts, stage 2
  // ignites at 131.8 s and burns out at 193 s, and the check-case data ends 7 s later with the vehicle still
  // climbing (it is a rocket *to orbit*, so there is no apogee inside the window). Those are the numbers the
  // reference trajectory in Aetherion's data/Atmos_17_TwoStageRocketToOrbit/ is tabulated against.
  double stop_s = 200.0;
  // The FMU defaults this to 0 = "ignite stage 2 the moment stage 1 separates", which is *not* the scenario:
  // it burns the second stage 94 s early and puts the vehicle 400 km high at t = 200 s instead of 234 km.
  // Aetherion's own reference run derives the same figure as endTime - 7 s post-burn coast - 61.2 s burn.
  double stg2_ignition_s = 131.8;
  // Scenario 17's launch site is the equator on the prime meridian, pointing due east. Overridable, but the
  // reference trajectory is only reproducible from here.
  double lat0_deg = 0.0;
  double lon0_deg = 0.0;
  double alt0_m = 0.0;
  double step_s = 0.1;           // communication step == GPS output period (10 Hz)
  double imu_rate_hz = 100.0;    // IMU output data rate; the IMU FMU emits step * rate frames per step
  double realtime_factor = 0.0;  // 0 = run as fast as possible
  // Receiver dynamics envelope. The platform model is off by default and
  // COCOM is on, which isolates the mechanism that is not a configuration
  // choice: COCOM is export control, present in any receiver you can buy,
  // whereas dynModel is a register a firmware engineer writes.
  //
  // Leaving the platform model in place hides that. Even its least
  // restrictive airborne setting (code 8, <4 g) trips on the *second*
  // navigation epoch of this flight -- thrust alone is ~54 m/s^2, so
  // coordinate acceleration off the pad is ~4.6 g -- and the receiver then
  // reports one usable fix in 2001 epochs, with COCOM never getting to be the
  // reason for anything. With it disabled the export limits alone give 312
  // fixes and a clean cut-off at 31.2 s, which is the behaviour worth
  // studying. Pass --dyn-model 8 to put the platform envelope back.
  int dynamic_platform = -1;
  bool cocom_limits = true;
  double reacquisition_time_s = 2.0;
  // GPS error model: the FMU's default white-only receiver, or the realistic
  // time-correlated one (doc/sensor_models.rst). Off by default so the
  // receiver -- and every figure drawn from it -- stays the one this example
  // was verified with until the change is asked for.
  bool gps_correlated_errors = false;
  // 0 = every sensor draws a fresh instrument, as an unseeded run always has.
  //
  // Any other value fixes every sensor's error stream: the sample a part produces
  // at a given simulation time is then identical run to run (measured: 790 of 790
  // shared IMU timestamps, byte for byte). It does *not* make the two-process run
  // reproducible -- the flight computer polls the I2C parts and drains the IMU
  // FIFO on its own wall clock, so which samples it catches still varies, and the
  // logs differ in their first timestamp and their length. Statistics reproduce;
  // a diff of the sensor logs does not.
  // Each part's own oscillator. 0,0 is the single perfect clock the FMUs shared
  // before this existed; see --sensor-clock.
  double sensor_clock_skew_ppm = 0.0;
  double sensor_clock_jitter_s = 0.0;
  // Date the World Magnetic Model is evaluated at. The epoch by default: its
  // coefficients are exact there and a fixed date keeps a run reproducible.
  double mag_date_year = hemerion::sensors::mag::WorldMagneticModel::kValidFromYear;
  int seed = 0;
  // Overrides the derived GPS seed, because this page's GPS figures were
  // published against particular values of it.
  int gps_seed = 0;  // 0 = nondeterministic; any other value reproduces the receiver's errors
};

void print_usage()
{
  std::cout << "usage: rocket_gps_cosim [--rocket <TwoStageRocket.fmu>] [--gps <hemerion_gps_fmu.fmu>]\n"
               "                        [--imu <hemerion_imu_fmu.fmu>] [--baro <hemerion_bmp390_fmu.fmu>]\n"
               "                        [--mag <hemerion_mmc5983ma_fmu.fmu>]\n"
               "                        [--imu-rate <hz>] [--dyn-model <code>] [--no-cocom] [--reacq <s>]\n"
               "                        [--gps-errors white|correlated] [--gps-seed <n>]\n"
               "                        [--stg2-ignition <s>] [--lat0 <deg>] [--lon0 <deg>] [--alt0 <m>]\n"
               "                        [--stop <s>] [--step <s>] [--csv <file>] [--rtf <x>]\n"
               "\n"
               "  --rocket    path to Aetherion's TwoStageRocket.fmu (default: configure-time location)\n"
               "  --gps       path to the packaged hemerion_gps_fmu.fmu (default: build-tree artifact)\n"
               "  --imu       path to the packaged hemerion_imu_fmu.fmu (default: build-tree artifact)\n"
               "  --baro      path to the packaged hemerion_bmp390_fmu.fmu (default: build-tree artifact;\n"
               "              no rate option -- the part converts at the ODR the flight computer programs)\n"
               "  --mag       path to the packaged hemerion_mmc5983ma_fmu.fmu (default: build-tree artifact;\n"
               "              no rate option either -- the part measures at the rate the flight computer\n"
               "              programs into Internal control 2, and on command during its bring-up)\n"
               "  --imu-rate  IMU output data rate [Hz] (default 100)\n"
               "  --dyn-model u-blox dynModel code for the receiver's platform model (default -1 = no platform\n"
               "              envelope, leaving COCOM as the only limit; 8 = airborne <4 g, whose acceleration\n"
               "              limit trips on the second epoch of this flight and masks everything else)\n"
               "  --no-cocom  clear the COCOM export cut-off, as on an export-licensed receiver (default: in force,\n"
               "              so navigation output stops above 18000 m AND 515 m/s)\n"
               "  --reacq     re-acquisition hold-off after any limit trips [s] (default 2)\n"
               "  --gps-errors  white (default: the FMU's white-only receiver) or correlated: time-correlated\n"
               "              Gauss-Markov position/velocity errors and an optimistic reported accuracy\n"
               "  --seed      seed every sensor's error model: each part then produces the same\n"
               "              sample at a given simulation time run to run. The flight computer's\n"
               "              own polling still varies, so the logs are not diff-identical.\n"
               "              (default 0 = each part is a fresh draw)\n"
               "  --gps-seed  override just the receiver's seed (default: derived from --seed)\n"
               "  --mag-date  date for the World Magnetic Model [decimal year] (default 2025.0, the\n"
               "              model epoch, where its coefficients are exact and a run stays\n"
               "              reproducible). Must lie in [2025.0, 2030.0]: the model is predictive\n"
               "              and is refused outside its window rather than extrapolated\n"
               "  --sensor-clock  <skew_ppm>,<jitter_s>: give each sensor its own oscillator (default 0,0,\n"
               "              one perfect clock shared by every part). skew is a rate error drawn once\n"
               "              per part and accumulates; jitter is per-sample and does not. Each part\n"
               "              draws its own from --seed, so one magnitude gives several different\n"
               "              clocks -- which is the point\n"
               "  --stg2-ignition  absolute time stage 2 lights [s] (default 131.8 = NASA Scenario 17; the\n"
               "              FMU's own default of 0 means 'immediately after staging', which is a different\n"
               "              flight profile and does not reproduce the reference trajectory)\n"
               "  --lat0 --lon0 --alt0  launch site (default 0/0/0, Scenario 17's equatorial pad)\n"
               "  --stop      simulation stop time [s] (default 200 = the reference trajectory's window)\n"
               "  --step      communication step size [s]; also the GPS fix period (default 0.1)\n"
               "  --csv       rocket truth output CSV (default results/rocket_truth.csv)\n"
               "  --rtf       real-time factor pacing, e.g. 1 = wall-clock speed; 0 = unpaced (default)\n";
}

// The options that take a value, paired with what to do with it. A table rather than a chain
// of `arg == "--x" && (value = next())`: an assignment inside a condition reads as a
// comparison, and repeating the form once per option made adding one a copy-paste exercise.
// Captureless lambdas convert to the plain function pointer, so the table is constexpr.
struct ValueOption
{
  std::string_view name;
  void (*apply)(Options&, const char*);
};

constexpr std::array<ValueOption, 21> kValueOptions = { {
    { "--rocket", [](Options& o, const char* v) { o.rocket_fmu = v; } },
    { "--gps", [](Options& o, const char* v) { o.gps_fmu = v; } },
    { "--imu", [](Options& o, const char* v) { o.imu_fmu = v; } },
    { "--baro", [](Options& o, const char* v) { o.baro_fmu = v; } },
    { "--mag", [](Options& o, const char* v) { o.mag_fmu = v; } },
    { "--imu-rate", [](Options& o, const char* v) { o.imu_rate_hz = std::stod(v); } },
    { "--dyn-model", [](Options& o, const char* v) { o.dynamic_platform = std::stoi(v); } },
    { "--reacq", [](Options& o, const char* v) { o.reacquisition_time_s = std::stod(v); } },
    { "--gps-errors",
      [](Options& o, const char* v) {
        const std::string model(v);
        if (model != "white" && model != "correlated")
        {
          throw std::invalid_argument("--gps-errors takes 'white' or 'correlated', not '" + model + "'");
        }
        o.gps_correlated_errors = (model == "correlated");
      } },
    { "--seed", [](Options& o, const char* v) { o.seed = std::stoi(v); } },
    { "--mag-date",
      [](Options& o, const char* v) {
        o.mag_date_year = std::stod(v);
        // Rejected here rather than 200 steps in: the model refuses dates
        // outside its window, so a bad one is a user error with a message.
        if (!WorldMagneticModel::covers(o.mag_date_year))
        {
          throw std::invalid_argument("--mag-date must lie in the field model\'s validity "
                                      "window [2025.0, 2030.0]; it is predictive, not extrapolated");
        }
      } },
    { "--sensor-clock",
      [](Options& o, const char* v) {
        std::array<double, 2> clock{};
        hemerion::examples::parse_csv_doubles(v, clock, "--sensor-clock", "skew_ppm,jitter_s, e.g. 50,200e-6");
        o.sensor_clock_skew_ppm = clock[0];
        o.sensor_clock_jitter_s = clock[1];
      } },
    { "--gps-seed", [](Options& o, const char* v) { o.gps_seed = std::stoi(v); } },
    { "--stg2-ignition", [](Options& o, const char* v) { o.stg2_ignition_s = std::stod(v); } },
    { "--lat0", [](Options& o, const char* v) { o.lat0_deg = std::stod(v); } },
    { "--lon0", [](Options& o, const char* v) { o.lon0_deg = std::stod(v); } },
    { "--alt0", [](Options& o, const char* v) { o.alt0_m = std::stod(v); } },
    { "--stop", [](Options& o, const char* v) { o.stop_s = std::stod(v); } },
    { "--step", [](Options& o, const char* v) { o.step_s = std::stod(v); } },
    { "--csv", [](Options& o, const char* v) { o.csv_path = v; } },
    { "--rtf", [](Options& o, const char* v) { o.realtime_factor = std::stod(v); } },
} };

bool parse_args(int argc, char** argv, Options& options)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      print_usage();
      return false;
    }
    // The one option that takes no value, so it cannot live in the table above.
    if (arg == "--no-cocom")
    {
      options.cocom_limits = false;
      continue;
    }

    const auto option = std::ranges::find(kValueOptions, arg, &ValueOption::name);
    // Same two rejections the else-if chain made, and the same message: an option nobody
    // knows, or a known one standing at the end of argv with no value behind it.
    if (option == kValueOptions.end() || i + 1 >= argc)
    {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      print_usage();
      return false;
    }
    // A value the option cannot take -- a word where a number was expected,
    // a model name that is not one -- is a usage error, not a crash: the
    // table's lambdas throw (std::stoi/stod on their own, --gps-errors
    // deliberately), and an exception escaping here would terminate the
    // process with no message at all.
    try
    {
      option->apply(options, argv[++i]);
    }
    catch (const std::exception& ex)
    {
      std::cerr << "bad value for " << arg << ": " << ex.what() << "\n";
      print_usage();
      return false;
    }
  }
  return true;
}

/// @brief Which external archiver Ecos will spawn to unpack an .fmu.
///
/// Mirrors ecos::detail::make_args(): on Windows it uses `unzip` when SHELL
/// names a bash and `tar` otherwise; elsewhere it is always `unzip`. Only used
/// to word the diagnostic below -- the check itself calls Ecos' own code.
[[nodiscard]] const char* ecos_archiver()
{
#if defined(_WIN32)
  // std::getenv is flagged deprecated by the Windows SDK headers (in favour of _dupenv_s) purely as an MSVC CRT
  // "insecure function" nag, not a real portability issue -- std::getenv is the standard, portable way to do this.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  const char* shell = std::getenv("SHELL");
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
  return (shell != nullptr && std::string(shell).find("bash") != std::string::npos) ? "unzip" : "tar";
#else
  return "unzip";
#endif
}

/// @brief Proves an .fmu can actually be unpacked before the run starts.
///
/// Ecos does not link a zip library: it shells out to `tar` or `unzip` and
/// reports a failure as "Failed to unzip ... to tempdir", which says nothing
/// about *why*. The two ways it goes wrong on a developer machine are both
/// environmental and both invisible from that message:
///
/// * Git for Windows puts a GNU `tar` on PATH ahead of the `tar.exe` shipped in
///   System32 (bsdtar). GNU tar reads a leading `C:` as an rsh-style *host*, so
///   every absolute Windows path fails with "Cannot connect to C: resolve
///   failed".
/// * Ecos switches to `unzip` when SHELL names a bash -- but Git for Windows
///   does not ship `unzip`, so running from Git Bash picks a program that is
///   not there.
///
/// Rather than infer either from PATH, this extracts a real archive with Ecos'
/// own unzip() into a temporary directory and reports what to do if that fails.
///
/// @param archive An .fmu that is known to exist.
/// @return True if extraction succeeded.
[[nodiscard]] bool preflight_fmu_extraction(const std::filesystem::path& archive)
{
  std::error_code ec;
  const std::filesystem::path probe_dir =
      std::filesystem::temp_directory_path(ec) /
      ("hemerion_unzip_probe_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  if (ec || !std::filesystem::create_directories(probe_dir, ec) || ec)
  {
    // Not being able to make a temp directory is a different problem, and Ecos
    // is about to hit it too -- let it report that one itself.
    return true;
  }

  const bool extracted = ecos::unzip(archive, probe_dir);
  std::filesystem::remove_all(probe_dir, ec);
  if (extracted)
  {
    return true;
  }

  const std::string archiver = ecos_archiver();
  std::cerr << "error: cannot unpack " << archive.string() << "\n"
            << "       Ecos unpacks every .fmu by spawning '" << archiver
            << "'; that call failed before the simulation started.\n";
  if (archiver == "tar")
  {
    std::cerr << "       Most likely a GNU tar (Git for Windows' usr\\bin) is ahead of Windows' own\n"
                 "       tar on PATH. GNU tar reads the leading 'C:' of an absolute path as a remote\n"
                 "       host -- 'Cannot connect to C: resolve failed'. Put C:\\Windows\\System32\n"
                 "       first on PATH so the bundled bsdtar wins:\n"
                 "           set PATH=C:\\Windows\\System32;%PATH%\n";
  }
  else
  {
    std::cerr << "       Ecos picks 'unzip' over 'tar' whenever SHELL names a bash, and Git for\n"
                 "       Windows does not ship an unzip. Either install one, or clear SHELL for\n"
                 "       this process so Ecos falls back to tar.\n";
  }
  return false;
}

/// @brief Records the run's configuration next to its truth log.
///
/// A figure that does not say which configuration produced it will eventually
/// be read as if it came from the other one -- and with this example the two
/// differ on whether the GPS reports anything at all, so the misreading is
/// guaranteed rather than merely possible. plot_results.py picks this file up
/// and stamps the receiver settings onto every figure it draws, so a stray PNG
/// still carries its own provenance.
///
/// Written as `<truth log stem>.config` beside the CSV, in trivial key=value
/// lines: it is read by one script and by humans, and nothing here justifies a
/// parser.
void write_run_config(const std::filesystem::path& csv_path, const Options& options)
{
  std::filesystem::path config_path = csv_path;
  config_path.replace_extension(".config");
  std::ofstream out(config_path);
  if (!out)
  {
    // Losing the provenance stamp must not lose the run.
    std::cerr << "warning: cannot write " << config_path.string() << "; figures will be unlabelled\n";
    return;
  }
  // The field the run is driven with, recorded so the analysis reads what the
  // model produced instead of carrying a second copy of it. A figure that turns
  // a magnetic heading into a true one needs this declination; computing it
  // twice, in two languages, is how the two quietly stop agreeing.
  double field_declination_deg = 0.0;
  double field_inclination_deg = 0.0;
  double field_intensity_ut = 0.0;
  WmmField initial_field;
  if (WorldMagneticModel::field_ned(
          options.lat0_deg, options.lon0_deg, options.alt0_m, options.mag_date_year, initial_field) == WmmError::kNone)
  {
    field_declination_deg = initial_field.declination_deg();
    field_inclination_deg = initial_field.inclination_deg();
    field_intensity_ut = initial_field.intensity_nt() * 1e-3;
  }
  // Written in full so a figure can be checked against the model that made it.
  const GpsErrorModel& receiver = options.gps_correlated_errors ? kCorrelatedReceiver : kWhiteReceiver;
  out << "dynamic_platform=" << options.dynamic_platform << "\n"
      << "cocom_limits_enabled=" << (options.cocom_limits ? 1 : 0) << "\n"
      << "reacquisition_time_s=" << options.reacquisition_time_s << "\n"
      << "gps_error_model=" << receiver.name << "\n"
      << "seed=" << options.seed << "\n"
      << "mag_date_year=" << options.mag_date_year << "\n"
      << "mag_declination_deg=" << field_declination_deg << "\n"
      << "mag_inclination_deg=" << field_inclination_deg << "\n"
      << "mag_intensity_ut=" << field_intensity_ut << "\n"
      << "sensor_clock_skew_ppm=" << options.sensor_clock_skew_ppm << "\n"
      << "sensor_clock_jitter_s=" << options.sensor_clock_jitter_s << "\n"
      << "gps_seed=" << options.gps_seed << "\n"
      << "gps_horizontal_pos_noise_m=" << receiver.horizontal_pos_noise_m << "\n"
      << "gps_vertical_pos_noise_m=" << receiver.vertical_pos_noise_m << "\n"
      << "gps_speed_noise_mps=" << receiver.speed_noise_mps << "\n"
      << "gps_course_noise_deg=" << receiver.course_noise_deg << "\n"
      << "gps_horizontal_pos_correlated_m=" << receiver.horizontal_pos_correlated_m << "\n"
      << "gps_vertical_pos_correlated_m=" << receiver.vertical_pos_correlated_m << "\n"
      << "gps_position_correlation_time_s=" << receiver.position_correlation_time_s << "\n"
      << "gps_speed_correlated_mps=" << receiver.speed_correlated_mps << "\n"
      << "gps_course_correlated_deg=" << receiver.course_correlated_deg << "\n"
      << "gps_velocity_correlation_time_s=" << receiver.velocity_correlation_time_s << "\n"
      << "gps_accuracy_scale=" << receiver.accuracy_scale << "\n"
      << "stg2_ignition_s=" << options.stg2_ignition_s << "\n"
      << "lat0_deg=" << options.lat0_deg << "\n"
      << "lon0_deg=" << options.lon0_deg << "\n"
      << "alt0_m=" << options.alt0_m << "\n"
      << "step_s=" << options.step_s << "\n"
      << "stop_s=" << options.stop_s << "\n"
      << "imu_rate_hz=" << options.imu_rate_hz << "\n"
      << "realtime_factor=" << options.realtime_factor << "\n";
}

/// @brief Truth log, written by the host rather than by Ecos' ``csv_writer``.
///
/// ``csv_writer`` formats every real with a default-configured ostringstream:
/// six decimal places. For metres that is sub-micron and perfectly fine, but
/// geodetic position is an angle, and six decimals of an angle is a length on
/// the ground -- which one depends on the unit. In **radians** the sixth
/// decimal was 6.4 m, and it swamped the 1.5 m per axis GpsNoiseModel
/// injects: the decoded-fix error figure was a picture of the log's own
/// rounding, sitting at 2.8 m RMS, exactly
/// sqrt(1.5^2 + (1.5^2 + (6.37/sqrt(12))^2)).
///
/// In **degrees** (Aetherion >= 0.13.0) the same sixth decimal is 0.11 m, so
/// the quantisation is 3.2 cm RMS and inflates the horizontal error figure by
/// 0.02%. This class is therefore no longer load-bearing for those numbers,
/// and is kept because the reason for it never rested on the size of the
/// error: the plant's state should reach the file intact, and a reference a
/// measurement is judged against should not carry avoidable noise of its
/// own.
///
/// The host already reads these properties every step for the specific-force
/// computation, so writing the row itself costs nothing and puts the format
/// under our control. The header and separator deliberately match
/// ``csv_writer``'s so plot_results.py and verify_trajectory.py need no
/// special case.
class TruthLogger
{
public:
  /// @param path            Destination CSV; parent directories are created.
  /// @param sim             Loaded simulation to read properties from.
  /// @param real_variables  Ecos identifiers of the reals to log, in order.
  /// @param bool_variables  Ecos identifiers of the booleans to log, in order.
  TruthLogger(const std::filesystem::path& path,
              const ecos::simulation& sim,
              const std::vector<std::string>& real_variables,
              const std::vector<std::string>& bool_variables)
  {
    if (path.has_parent_path())
    {
      std::filesystem::create_directories(path.parent_path());
    }
    out_.open(path);
    if (!out_)
    {
      throw std::runtime_error("cannot open " + path.string() + " for writing");
    }
    // Round-trip precision: the point of this class is that nothing is lost
    // between the plant's state and the file.
    out_ << std::setprecision(std::numeric_limits<double>::max_digits10);

    // ecos::variable_identifier is constructible from const char* but not from
    // std::string, hence the c_str(). A name the simulation does not know
    // yields a null property, which would only show up as a crash on the first
    // row -- name it here instead.
    out_ << "iterations, time";
    for (const std::string& name : real_variables)
    {
      auto* property = sim.get_real_property(name.c_str());
      if (property == nullptr)
      {
        throw std::runtime_error("no such real variable to log: " + name);
      }
      reals_.push_back(property);
      out_ << ", " << name << "[REAL]";
    }
    for (const std::string& name : bool_variables)
    {
      auto* property = sim.get_bool_property(name.c_str());
      if (property == nullptr)
      {
        throw std::runtime_error("no such boolean variable to log: " + name);
      }
      bools_.push_back(property);
      out_ << ", " << name << "[BOOL]";
    }
    out_ << "\n";
  }

  /// Appends one communication point.
  void write_row(std::uint64_t iterations, double time_s)
  {
    out_ << iterations << ", " << time_s;
    for (const auto* property : reals_)
    {
      out_ << ", " << property->get_value();
    }
    for (const auto* property : bools_)
    {
      out_ << ", " << (property->get_value() ? 1 : 0);
    }
    out_ << "\n";
  }

private:
  std::ofstream out_;
  std::vector<ecos::property_t<double>*> reals_;
  std::vector<ecos::property_t<bool>*> bools_;
};

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  if (!parse_args(argc, argv, options))
  {
    return EXIT_FAILURE;
  }

  for (const auto& [label, path] : { std::pair{ "rocket", options.rocket_fmu },
                                     std::pair{ "gps", options.gps_fmu },
                                     std::pair{ "imu", options.imu_fmu },
                                     std::pair{ "baro", options.baro_fmu },
                                     std::pair{ "mag", options.mag_fmu } })
  {
    if (path.empty() || !std::filesystem::exists(path))
    {
      std::cerr << "error: " << label << " FMU not found at '" << path.string() << "' -- pass --" << label << "\n";
      return EXIT_FAILURE;
    }
  }

  // All four archives take the same extraction path, so one probe settles it.
  // Cheapest to use our own GPS FMU: it is small and always in the build tree.
  if (!preflight_fmu_extraction(options.gps_fmu))
  {
    return EXIT_FAILURE;
  }

  ecos::log::set_logging_level(ecos::log::level::info);

  try
  {
    ecos::simulation_structure ss;
    // Deliberately the string-URI overload: ecos' std::filesystem::path overload runs the path through
    // std::filesystem::relative(), which yields an empty path on Windows when the FMU sits on a different
    // drive than the working directory. The file resolver handles absolute path strings as-is.
    ss.add_model("rocket", options.rocket_fmu.string());
    ss.add_model("gps", options.gps_fmu.string());
    ss.add_model("imu", options.imu_fmu.string());
    ss.add_model("baro", options.baro_fmu.string());
    ss.add_model("mag", options.mag_fmu.string());

    // Geodetic position needs no modifier: Aetherion >= 0.13.0 publishes
    // out.lat_deg/out.lon_deg in the degrees the GPS FMU's inputs are named
    // for. Both ends carry the unit in the port name, which is the only reason
    // a bare connection here can be read as correct rather than merely
    // plausible -- and an Aetherion older than that fails at ss.load() naming
    // the port it cannot find, rather than silently feeding radians.
    ss.make_connection<double>("rocket::out.lat_deg", "gps::latitude_deg");
    ss.make_connection<double>("rocket::out.lon_deg", "gps::longitude_deg");
    ss.make_connection<double>("rocket::out.alt_m", "gps::altitude_m");
    ss.make_connection<double>("rocket::out.v_north_m_s", "gps::v_north_mps");
    ss.make_connection<double>("rocket::out.v_east_m_s", "gps::v_east_mps");
    ss.make_connection<double>("rocket::out.v_down_m_s", "gps::v_down_mps");

    // Body angular rates feed the IMU's gyroscope triad 1:1, and since
    // Aetherion 0.14.0 so does specific force. out.specificForce_* is
    // (F_aero + F_thrust)/m at the CG in body axes, with gravitation
    // structurally excluded -- exactly what the IMU FMU's inputs are named
    // for. Before that this example summed thrust and aero itself and divided
    // by mass in the stepping loop, which meant asserting host-side that
    // thrust acts along body +X. The plant knows its own installation
    // geometry; the bench does not, and no longer has to guess.
    ss.make_connection<double>("rocket::out.p_rad_s", "imu::p_rad_s");
    ss.make_connection<double>("rocket::out.q_rad_s", "imu::q_rad_s");
    ss.make_connection<double>("rocket::out.r_rad_s", "imu::r_rad_s");
    ss.make_connection<double>("rocket::out.specificForce_x_m_s2", "imu::f_x_mps2");
    ss.make_connection<double>("rocket::out.specificForce_y_m_s2", "imu::f_y_mps2");
    ss.make_connection<double>("rocket::out.specificForce_z_m_s2", "imu::f_z_mps2");

    // The BMP390's whole truth interface is altitude: its measurement model
    // owns the atmosphere (ISA) and the part's error model, and everything
    // else about its behaviour -- rate included -- is register state the
    // flight computer programs over I2C.
    ss.make_connection<double>("rocket::out.alt_m", "baro::h_m");
    // And the air it is actually in. The part inverts the ISA from h_m when
    // nothing writes p_Pa, which is only right on a standard day; the plant
    // integrates its own atmosphere and publishes it, so on a non-standard
    // day (Aetherion's atm.deltaT_K / atm.deltaP_sl_Pa) the barometer reads
    // the day the aircraft is flying through rather than the book's.
    ss.make_connection<double>("rocket::out.P_Pa", "baro::p_Pa");
    const std::function<double(const double&)> kelvin2celsius = [](const double& kelvin) { return kelvin - 273.15; };
    ss.make_connection<double>("rocket::out.T_K", "baro::T_degC", kelvin2celsius);

    // The magnetometer's die temperature: ambient air, near enough for a part
    // whose temperature channel quantizes at 0.8 C. The field itself has no
    // rocket output to connect -- it is computed in the stepping loop below,
    // for the same reason specific force is.
    ss.make_connection<double>("rocket::out.T_K", "mag::temperature_c", kelvin2celsius);

    // NASA TM-2015-218675 Scenario 17's initial conditions: equatorial pad on
    // the prime meridian, due east, 55.22 deg nose-up. These are the
    // conditions the published check-case trajectory
    // (Aetherion's data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_06.csv) is
    // tabulated from, so departing from them means the run can no longer be
    // checked against anything. solver.max_step_s is left at its default (0 =
    // one implicit Radau step per communication step); the integrator is
    // L-stable, and capping sub-steps at 0.01 s costs ~10x the wall time for
    // no visible change in the 10 Hz GPS-grade outputs.
    ecos::parameter_set launch_site;
    launch_site["rocket::lat0_deg"] = options.lat0_deg;
    launch_site["rocket::lon0_deg"] = options.lon0_deg;
    launch_site["rocket::alt0_m"] = options.alt0_m;
    launch_site["rocket::azimuth0_deg"] = 90.0;
    launch_site["rocket::pitch0_deg"] = 55.22;
    // Without this the FMU ignites stage 2 the moment stage 1 separates, which
    // is a different flight entirely -- see the Options comment.
    launch_site["rocket::stg2.ignition_time_s"] = options.stg2_ignition_s;
    // Not launch-site geometry, but sim->init() applies exactly one named
    // parameter set, so the sensor configuration rides along here.
    launch_site["imu::sample_rate_hz"] = options.imu_rate_hz;
    // Written explicitly even though these match the GPS FMU's own defaults:
    // whether the receiver keeps a fix through this flight is the scenario's
    // most consequential setting, and it should be readable here rather than
    // inherited silently from modelDescription.xml.
    launch_site["gps::dynamic_platform"] = options.dynamic_platform;
    launch_site["gps::cocom_limits_enabled"] = options.cocom_limits;
    launch_site["gps::reacquisition_time_s"] = options.reacquisition_time_s;
    // The receiver's error model, in full: see GpsErrorModel.
    const GpsErrorModel& receiver = options.gps_correlated_errors ? kCorrelatedReceiver : kWhiteReceiver;
    // One stream per sensor, all derived from --seed; --gps-seed still wins for the receiver.
    launch_site["gps::seed"] = (options.gps_seed != 0) ? options.gps_seed : derived_seed(options.seed, 0);
    launch_site["imu::seed"] = derived_seed(options.seed, 1);
    launch_site["baro::seed"] = derived_seed(options.seed, 2);
    launch_site["mag::seed"] = derived_seed(options.seed, 3);
    // Each part's own oscillator, same magnitude and a different draw per part
    // (each FMU seeds its clock from its own seed). The magnetometer is absent
    // because the MMC5983MA has no time register to skew, and so is the
    // receiver: nothing downstream reads a GPS time of its own either. Both
    // show their clock error in when data arrives -- see host_time_s in the
    // flight computer's logs.
    launch_site["imu::clock_skew_sigma_ppm"] = options.sensor_clock_skew_ppm;
    launch_site["imu::clock_jitter_sigma_s"] = options.sensor_clock_jitter_s;
    launch_site["baro::clock_skew_sigma_ppm"] = options.sensor_clock_skew_ppm;
    launch_site["baro::clock_jitter_sigma_s"] = options.sensor_clock_jitter_s;
    launch_site["gps::horizontal_pos_noise_m"] = receiver.horizontal_pos_noise_m;
    launch_site["gps::vertical_pos_noise_m"] = receiver.vertical_pos_noise_m;
    launch_site["gps::speed_noise_mps"] = receiver.speed_noise_mps;
    launch_site["gps::course_noise_deg"] = receiver.course_noise_deg;
    launch_site["gps::horizontal_pos_correlated_m"] = receiver.horizontal_pos_correlated_m;
    launch_site["gps::vertical_pos_correlated_m"] = receiver.vertical_pos_correlated_m;
    launch_site["gps::position_correlation_time_s"] = receiver.position_correlation_time_s;
    launch_site["gps::speed_correlated_mps"] = receiver.speed_correlated_mps;
    launch_site["gps::course_correlated_deg"] = receiver.course_correlated_deg;
    launch_site["gps::velocity_correlation_time_s"] = receiver.velocity_correlation_time_s;
    launch_site["gps::accuracy_scale"] = receiver.accuracy_scale;
    ss.add_parameter_set("launchSite", launch_site);

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(options.step_s));

    sim->init("launchSite");

    // Logged by TruthLogger rather than Ecos' csv_writer, which rounds every
    // real -- geodetic angles included -- to six decimal places. See the class
    // comment for what that costs in degrees, and what it used to cost when
    // these two ports were radians.
    TruthLogger truth_log(options.csv_path,
                          *sim,
                          { "rocket::out.alt_m",
                            "rocket::out.lat_deg",
                            "rocket::out.lon_deg",
                            "rocket::out.v_north_m_s",
                            "rocket::out.v_east_m_s",
                            "rocket::out.v_down_m_s",
                            "rocket::out.p_rad_s",
                            "rocket::out.q_rad_s",
                            "rocket::out.r_rad_s",
                            "rocket::out.mach",
                            "rocket::out.qbar_Pa",
                            "rocket::out.thrust_N",
                            "rocket::out.mass_kg",
                            // What the IMU FMU actually receives: the plant's body-frame
                            // specific force, one step behind rocket truth like every
                            // other connection.
                            "imu::f_x_mps2",
                            "imu::f_y_mps2",
                            "imu::f_z_mps2",
                            // Likewise for the magnetometer: the body-frame truth field
                            // the host computed and wrote, so a decoded sample can be
                            // compared against what the part was actually given.
                            "mag::b_x_ut",
                            "mag::b_y_ut",
                            "mag::b_z_ut" },
                          { "rocket::out.staged" });
    write_run_config(options.csv_path, options);

    auto* altitude = sim->get_real_property("rocket::out.alt_m");
    auto* mach = sim->get_real_property("rocket::out.mach");
    auto* mass = sim->get_real_property("rocket::out.mass_kg");
    auto* staged = sim->get_bool_property("rocket::out.staged");

    // Magnetic-field plumbing: the field a magnetometer sees depends on where
    // the vehicle is *and* how it is pointing -- four rocket outputs -- and an
    // Ecos connection modifier sees only its single source variable. So the
    // host computes the field after every step and writes the magnetometer's
    // inputs directly, giving the same one-communication-step transport delay
    // a connection would. WorldMagneticModel owns the model: WMM2025 to degree
    // and order 12, from NOAA's own coefficients, evaluated at --mag-date. The
    // declination it produces is written into the .config sidecar so the
    // analysis reads what this evaluated rather than carrying a second copy of
    // the model -- which is how the two quietly stop agreeing.
    auto* latitude = sim->get_real_property("rocket::out.lat_deg");
    auto* longitude = sim->get_real_property("rocket::out.lon_deg");
    auto* yaw = sim->get_real_property("rocket::out.yaw_rad");
    auto* pitch = sim->get_real_property("rocket::out.pitch_rad");
    auto* roll = sim->get_real_property("rocket::out.roll_rad");
    auto* mag_bx = sim->get_real_property("mag::b_x_ut");
    auto* mag_by = sim->get_real_property("mag::b_y_ut");
    auto* mag_bz = sim->get_real_property("mag::b_z_ut");

    double apogee_m = 0.0;
    double apogee_time_s = 0.0;
    double staging_time_s = -1.0;
    const auto wall_start = std::chrono::steady_clock::now();

    std::cout << "[cosim] rocket: " << options.rocket_fmu.string() << "\n"
              << "[cosim] gps:    " << options.gps_fmu.string() << "\n"
              << "[cosim] imu:    " << options.imu_fmu.string() << "\n"
              << "[cosim] baro:   " << options.baro_fmu.string() << "\n"
              << "[cosim] mag:    " << options.mag_fmu.string() << "\n"
              << "[cosim] step " << options.step_s << " s (" << 1.0 / options.step_s << " Hz GPS, "
              << options.imu_rate_hz << " Hz IMU; BMP390 converts at the ODR the flight computer programs), stop "
              << options.stop_s << " s\n"
              << "[cosim] plant: launch " << options.lat0_deg << " deg N / " << options.lon0_deg << " deg E at "
              << options.alt0_m << " m, stage 2 ignition at t=" << options.stg2_ignition_s << " s\n"
              << "[cosim] receiver: ";
    // Name the platform model only when there is one. Reporting "dynModel -1"
    // on the default run announces a mechanism that is not running, which
    // reads as a limit the reader then tries to attribute the dropout to.
    if (options.dynamic_platform >= 0)
    {
      std::cout << "dynModel " << options.dynamic_platform << ", ";
    }
    std::cout << "COCOM limits " << (options.cocom_limits ? "in force (18000 m AND 515 m/s)" : "disabled")
              << ", re-acquisition " << options.reacquisition_time_s << " s\n";

    std::cout << "[cosim] GPS errors: "
              << (options.gps_correlated_errors ? "time-correlated (Gauss-Markov) + white, hAcc/vAcc at 70 %" :
                                                  "white per epoch, hAcc/vAcc honest")
              << (options.gps_seed != 0 ? " (seed " + std::to_string(options.gps_seed) + ")" : "") << "\n";

    long print_counter = 0;
    const long print_period = std::lround(10.0 / options.step_s);  // one status line per 10 s of sim time
    // Carried across steps so a refusal holds the last good field rather than
    // zeroing the part's input; see the write below.
    WmmFieldBody last_field_body;
    bool reported_field_gap = false;
    truth_log.write_row(sim->iterations(), sim->time());  // the state at t = 0, before any stepping
    while (sim->time() < options.stop_s)
    {
      sim->step();

      // Where the vehicle is and how it is pointing, turned into the body-frame
      // field the part is immersed in. Written after the step, so it carries the
      // same one-communication-step transport delay every Ecos connection does.
      WmmField field_ned;
      if (WorldMagneticModel::field_ned(
              latitude->get_value(), longitude->get_value(), altitude->get_value(), options.mag_date_year, field_ned) ==
          WmmError::kNone)
      {
        last_field_body = to_body(field_ned, yaw->get_value(), pitch->get_value(), roll->get_value());
      }
      else if (!reported_field_gap)
      {
        // Only reachable at a geographic pole, which none of these trajectories
        // visits. Holding the last good field beats writing a zero one into the
        // part as though it had measured that.
        std::cerr << "[cosim] warning: the field model refused this position; holding the last value\n";
        reported_field_gap = true;
      }
      const auto field_body_ut = last_field_body.to_microtesla();
      mag_bx->set_value(field_body_ut[0]);
      mag_by->set_value(field_body_ut[1]);
      mag_bz->set_value(field_body_ut[2]);
      // After the specific-force write, so the logged imu:: inputs are the ones
      // the FMU will sample on the next step -- matching what csv_writer
      // recorded as a post-step listener.
      truth_log.write_row(sim->iterations(), sim->time());

      const double altitude_m = altitude->get_value();
      if (altitude_m > apogee_m)
      {
        apogee_m = altitude_m;
        apogee_time_s = sim->time();
      }
      if (staging_time_s < 0.0 && staged->get_value())
      {
        staging_time_s = sim->time();
        std::cout << "[cosim] t=" << sim->time() << " s  stage 1 separated\n";
      }
      if (++print_counter % print_period == 0)
      {
        std::cout << "[cosim] t=" << sim->time() << " s  alt=" << altitude_m << " m  mach=" << mach->get_value()
                  << "  mass=" << mass->get_value() << " kg\n";
      }
      if (options.realtime_factor > 0.0)
      {
        const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(sim->time() / options.realtime_factor));
        std::this_thread::sleep_until(target);
      }
    }

    sim->terminate();

    const long imu_frames_per_step = std::lround(options.step_s * options.imu_rate_hz);
    // One NAV-PVT frame per step regardless of fix validity -- a receiver
    // outside its envelope keeps talking, it just stops claiming a solution.
    std::cout << "[cosim] done: " << sim->iterations() << " steps, " << sim->iterations()
              << " UBX-NAV-PVT frames emitted and " << sim->iterations() * std::max(1L, imu_frames_per_step)
              << " IMU samples buffered for the SPI controller\n"
              << "[cosim] the BMP390 and the MMC5983MA each answered at whatever rate the flight computer programmed "
                 "them to\n"
              // Scenario 17 is a rocket *to orbit*: inside the reference
              // window the vehicle is still climbing at cut-off, so this is
              // the highest altitude reached, not an apogee.
              << "[cosim] max altitude " << apogee_m << " m at t=" << apogee_time_s << " s";
    if (staging_time_s >= 0.0)
    {
      std::cout << ", staging at t=" << staging_time_s << " s";
    }
    std::cout << "\n[cosim] rocket truth written to " << options.csv_path.string() << "\n";
  }
  catch (const std::exception& ex)
  {
    std::cerr << "error: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
