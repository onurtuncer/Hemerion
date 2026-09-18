// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file cosim_host_main.cpp
/// @brief Ecos co-simulation host coupling Aetherion's F16Plant.fmu to all
/// five of Hemerion's hardware-simulator FMUs.
///
/// The scenarios are NASA TM-2015-218675's two open-loop **trim flyouts**,
/// selected with --case. Check-case 11 (the default): an F-16 trimmed for
/// subsonic straight-and-level flight over Kitty Hawk, NC at 10 013 ft and
/// 565.685 ft/s true airspeed (335 KTAS) on a 45 deg heading. Check-case 12:
/// the same aircraft from the same point on the same heading, trimmed at
/// 30 013 ft and Mach 2.01. Both fly open-loop for 200 s. `F16Plant.fmu` runs
/// its own trim solver during initialisation and seeds the control deflections
/// from the result, so nothing here closes a loop -- the aircraft simply flies
/// out of trim conditions, and the slow drift that follows (case 11: yaw
/// 45 -> 44.16 deg, +50 ft over the window in the published data; case 12: a
/// larger phugoid, +140 m in the drifting participant's solution) is the
/// check-case's content.
///
/// Topology (see examples/f16_trim_ecos/README.md):
///
///   F16Plant.fmu ──(FMI variables, Ecos connections)──> hemerion_gps_fmu.fmu
///    truth: lat/lon/alt, NED velocity   │                 noise + dynamics envelope
///           body rates, specific force  │                 + UBX-NAV-PVT
///           altitude, attitude          │                       │ UDP 127.0.0.1:5762
///                                       ├───────────> hemerion_imu_fmu.fmu
///                                       │                 noise + raw-count frames
///                                       │                       │ SPI over shared memory
///                                       ├───────────> hemerion_bmp390_fmu.fmu
///                                       │                 ISA + inverse compensation
///                                       │                       │ I2C over shared memory
///                                       ├───────────> hemerion_mmc5983ma_fmu.fmu
///                                       │                 SET/RESET + hard iron
///                                       │                       │ I2C over shared memory
///                                       └───────────> hemerion_radalt_fmu.fmu
///                                                         AGL return + dropout
///                                                               │ UDP 127.0.0.1:5765
///                                                               v
///                                                      f16_flight_computer
///
/// **Why these two check-cases.** Case 11 is the only scenario in the set
/// where all four environment-dependent sensor stacks are valid at the same
/// time. The receiver holds a fix for the whole run (172 m/s and 3.05 km
/// clear every limit it has); 10 013 ft is squarely inside the barometer's
/// useful band; 36 deg N gives the magnetometer a healthy horizontal field
/// component to find a heading in; and 3052 m AGL is inside the radar
/// altimeter's 6000 m tracking range. Scenario 17 -- the two-stage rocket in
/// examples/rocket_gps_ecos -- can do none of that: it loses the fix at 31 s,
/// leaves the atmosphere, and is out of radalt range within seconds of the
/// pad. That makes the case-11 run the cross-sensor consistency reference:
/// baro altitude against GPS altitude against radar height, magnetic heading
/// against GPS course, all on one trajectory.
///
/// Case 12 is the same wiring with two different numbers -- an altitude and
/// an airspeed -- and nearly the opposite sensor story: at 610 m/s the
/// receiver is past dynModel 8's 500 m/s platform limit from the very first
/// epoch and never reports a fix; at 9148 m the radar altimeter is past its
/// 6000 m tracking range and every return says so; and the barometer spends
/// most of the flight below the BMP390's 300 hPa rated pressure floor with
/// its die below the -40 C rating. Only the magnetometer stays in band. The
/// point is that all of this arrives through exactly the code paths case 11
/// exercises when everything works -- no wiring changes, just a flight the
/// parts were not specified for.
///
/// The plant reports geodetic latitude/longitude in degrees and NED velocity
/// in m/s, which is what the GPS FMU takes, so every truth->receiver
/// connection is 1:1. Body-frame specific force comes straight off
/// `out.specificForce_*` -- (F_aero + F_thrust)/m at the CG, gravitation
/// excluded -- and connects 1:1 as well, as do body rates p/q/r. Both
/// requirements are version floors on Aetherion; CMakeLists.txt tests for the
/// ports rather than for a version string.
///
/// **The radar altimeter measures height above ground, and there is no
/// terrain model here.** Its `h_agl_m` input is fed the plant's MSL altitude,
/// which is correct to within the terrain elevation at Kitty Hawk -- a few
/// metres of coastal North Carolina. On a scenario over mountains this
/// connection would be wrong, and it is wired directly rather than through a
/// terrain lookup precisely so that the assumption is visible in one line.
///
/// None of the sensor FMUs has FMI outputs -- their effect is the sensor bus
/// each part really uses: UDP stand-ins for the GPS receiver's and radar
/// altimeter's serial links, a shared-memory SPI bus (sim/spi_shm) the IMU
/// answers as a peripheral, and shared-memory I2C buses (sim/i2c_shm) the
/// BMP390 and MMC5983MA answer on as register-accurate parts. All five are
/// consumed by f16_flight_computer. Neither I2C part takes a rate parameter:
/// each converts at whatever ODR the flight computer programs into its
/// registers, which is the point of simulating the part instead of the
/// datasheet.
///
/// The GPS FMU's dynamics envelope is configured here rather than left at its
/// defaults, and unlike Scenario 17 the *realistic* setting is the
/// interesting one: an F-16 in trim pulls about 1 g, so u-blox dynModel 8
/// (airborne, <4 g) is the setting a firmware engineer would actually write.
/// On case 11 the receiver keeps its fix for all 200 s -- a whole flight
/// inside the envelope the rocket example exists to show biting. On case 12
/// the same setting takes the fix away entirely, and COCOM is *not* why:
/// COCOM needs 18 000 m AND 515 m/s, and 9.2 km is half its altitude
/// threshold. Rerun with --dyn-model -1 and the fix comes back at 610 m/s,
/// which makes case 12 the one flight in the NESC set that tells COCOM's AND
/// apart from an OR -- the rocket crosses both thresholds within seconds of
/// each other and cannot. See --dyn-model / --no-cocom.
///
/// Plant truth is logged so the flight computer's decoded fixes, IMU samples,
/// pressures, fields and radar heights can be compared against it.

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

#include "geomagnetic_field.hpp"

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
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
using hemerion::examples::f16_trim_ecos::FieldBody;
using hemerion::examples::f16_trim_ecos::FieldNed;
using hemerion::examples::f16_trim_ecos::GeomagneticDipole;

// Compile-time defaults injected by CMakeLists.txt; both can be overridden on
// the command line, so an empty default (FMU not found at configure time) is
// not fatal until the run actually needs it.
#ifndef HEMERION_F16_FMU_PATH
#define HEMERION_F16_FMU_PATH ""
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
#ifndef HEMERION_RADALT_FMU_PATH
#define HEMERION_RADALT_FMU_PATH ""
#endif

struct Options
{
  std::filesystem::path f16_fmu = HEMERION_F16_FMU_PATH;
  std::filesystem::path gps_fmu = HEMERION_GPS_FMU_PATH;
  std::filesystem::path imu_fmu = HEMERION_IMU_FMU_PATH;
  std::filesystem::path baro_fmu = HEMERION_BMP390_FMU_PATH;
  std::filesystem::path mag_fmu = HEMERION_MMC5983MA_FMU_PATH;
  std::filesystem::path radalt_fmu = HEMERION_RADALT_FMU_PATH;
  std::filesystem::path csv_path = "results/f16_truth.csv";
  // Which NASA TM-2015-218675 check-case to fly. Both trim flyouts run to 200 s, the window their reference
  // trajectories in Aetherion's data/Atmos_1{1,2}_*/ are tabulated over. --case only sets defaults: see
  // kTrimCases below for what differs, and parse_args() for why an explicit --alt0 still wins.
  std::string check_case = "11";
  double stop_s = 200.0;
  // The trim condition, written out here rather than inherited from the FMU's parameter defaults. Those
  // defaults *are* check-case 11, but they carry the geodetic position rounded to four decimals
  // (36.0192 / -75.6744) where the published initial condition has six. At this latitude the difference is
  // roughly 4 m of northing -- larger than the 1.5 m per axis GpsNoiseModel injects, so it would show up in
  // the very error figure this example exists to produce. A scenario's initial conditions should also be
  // readable in the scenario rather than looked up in modelDescription.xml. Both check-cases start from the
  // same point on the same heading; only altitude and airspeed differ.
  double lat0_deg = 36.019167;  // Kitty Hawk, NC
  double lon0_deg = -75.674444;
  double alt0_ft = 10013.0;
  double vt0_fps = 565.685;    // 335.15 KTAS
  double heading0_deg = 45.0;  // north-east
  // Aetherion's value for both cases. The published trim states read -0.1718 (case 11) and -0.1706 (case 12);
  // the difference tilts the heading by ~0.002 and ~0.005 deg over 200 s, invisible against a participant
  // spread measured in tenths of a degree, and staying on Aetherion's value is what lets this run agree with
  // Aetherion's own standalone examples to millimetres.
  double roll0_deg = -0.172;
  double step_s = 0.1;           // communication step == GPS output period (10 Hz)
  double imu_rate_hz = 100.0;    // IMU output data rate; the IMU FMU emits step * rate frames per step
  double radalt_rate_hz = 25.0;  // radar altimeter pulse rate
  double realtime_factor = 0.0;  // 0 = run as fast as possible
  // Receiver dynamics envelope. Both mechanisms are left in force, and the
  // setting is deliberately the same for both check-cases: it is a property
  // of the receiver bolted to the aircraft, not of the flight, and changing it
  // per case would hide the most interesting thing case 12 shows.
  //
  // dynModel 8 is "airborne, <4 g" -- the setting a firmware engineer would
  // actually write into a receiver on a fighter. Its platform envelope stops
  // navigation above 500 m/s.
  //
  // Case 11 (172 m/s, 3.05 km) sits far inside every limit: the receiver keeps
  // a fix for all 2001 epochs, which is what makes that run the cross-sensor
  // reference.
  //
  // Case 12 (610 m/s, 9.15 km) is past the platform's 500 m/s from t = 0, so
  // the receiver never reports a fix -- a Mach 2 aircraft with an airborne-
  // configured u-blox is flying without GPS. COCOM is *not* the reason: it
  // needs 18000 m AND 515 m/s, and 9.15 km is half the altitude threshold.
  // Rerun with --dyn-model -1 to take the platform envelope away and the fix
  // comes back, which is the one flight in the NESC set that tells COCOM's
  // AND apart from an OR -- the rocket crosses both thresholds within seconds
  // of each other and cannot.
  int dynamic_platform = 8;
  bool cocom_limits = true;
  double reacquisition_time_s = 2.0;
};

/// @brief The open-loop trim flyouts this example flies.
///
/// Both are "trim the aircraft at a point, then let it go for 200 s" from the
/// same place on the same heading, so the whole difference between them is an
/// altitude and an airspeed. Everything that makes case 12 interesting -- no
/// GPS fix, no radar return, a barometer at the edge of its datasheet range --
/// follows from those two numbers rather than from any wiring change, which is
/// why it is a row here and not a second example.
struct TrimCase
{
  std::string_view id;
  std::string_view summary;
  double alt0_ft;
  double vt0_fps;
};

constexpr std::array<TrimCase, 2> kTrimCases = { {
    { "11", "subsonic trim flyout, 10 013 ft, Mach 0.52", 10013.0, 565.685 },
    // The published initial condition is 1414.200033 ft/s north and east; the airspeed is their
    // resultant (1999.9999 ft/s, Mach 2.01), computed the same way Aetherion's F16SupersonicTrim does.
    { "12", "supersonic trim flyout, 30 013 ft, Mach 2.01", 30013.0, 1414.200033 * std::numbers::sqrt2 },
} };

void print_usage()
{
  std::cout << "usage: f16_trim_cosim [--case 11|12] [--f16 <F16Plant.fmu>] [--gps <hemerion_gps_fmu.fmu>]\n"
               "                      [--imu <hemerion_imu_fmu.fmu>] [--baro <hemerion_bmp390_fmu.fmu>]\n"
               "                      [--mag <hemerion_mmc5983ma_fmu.fmu>] [--radalt <hemerion_radalt_fmu.fmu>]\n"
               "                      [--imu-rate <hz>] [--radalt-rate <hz>]\n"
               "                      [--dyn-model <code>] [--no-cocom] [--reacq <s>]\n"
               "                      [--lat0 <deg>] [--lon0 <deg>] [--alt0 <ft>] [--vt0 <ft/s>]\n"
               "                      [--heading0 <deg>] [--roll0 <deg>]\n"
               "                      [--stop <s>] [--step <s>] [--csv <file>] [--rtf <x>]\n"
               "\n"
               "  --case      NASA TM-2015-218675 check-case: 11 = subsonic trim flyout at 10 013 ft (default),\n"
               "              12 = supersonic trim flyout at 30 013 ft. Sets the defaults below; explicit\n"
               "              options still override them wherever they appear on the command line\n"
               "  --f16       path to Aetherion's F16Plant.fmu (default: configure-time location)\n"
               "  --gps       path to the packaged hemerion_gps_fmu.fmu (default: build-tree artifact)\n"
               "  --imu       path to the packaged hemerion_imu_fmu.fmu (default: build-tree artifact)\n"
               "  --baro      path to the packaged hemerion_bmp390_fmu.fmu (default: build-tree artifact;\n"
               "              no rate option -- the part converts at the ODR the flight computer programs)\n"
               "  --mag       path to the packaged hemerion_mmc5983ma_fmu.fmu (default: build-tree artifact;\n"
               "              no rate option either -- the part measures at the rate the flight computer\n"
               "              programs into Internal control 2, and on command during its bring-up)\n"
               "  --radalt    path to the packaged hemerion_radalt_fmu.fmu (default: build-tree artifact)\n"
               "  --imu-rate  IMU output data rate [Hz] (default 100)\n"
               "  --radalt-rate  radar altimeter pulse rate [Hz] (default 25)\n"
               "  --dyn-model u-blox dynModel code for the receiver's platform model (default 8 = airborne <4 g,\n"
               "              which is what this aircraft is; it never trips, so the receiver holds a fix for\n"
               "              the whole run. -1 disables the platform envelope entirely)\n"
               "  --no-cocom  clear the COCOM export cut-off, as on an export-licensed receiver (default: in force,\n"
               "              so navigation output stops above 18000 m AND 515 m/s -- neither is approached here)\n"
               "  --reacq     re-acquisition hold-off after any limit trips [s] (default 2)\n"
               "  --lat0 --lon0  trim position [deg] (default 36.019167 / -75.674444, Kitty Hawk NC)\n"
               "  --alt0      trim altitude [ft] (default 10013, or 30013 for --case 12)\n"
               "  --vt0       trim true airspeed [ft/s] (default 565.685 = 335.15 KTAS, or 2000 for --case 12)\n"
               "  --heading0 --roll0  trim attitude [deg] (default 45 / -0.172)\n"
               "  --stop      simulation stop time [s] (default 200 = the reference trajectory's window)\n"
               "  --step      communication step size [s]; also the GPS fix period (default 0.1)\n"
               "  --csv       plant truth output CSV (default results/f16_truth.csv)\n"
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
    // Only records the choice; the case's defaults were applied in parse_args()'s first pass.
    { "--case", [](Options& o, const char* v) { o.check_case = v; } },
    { "--f16", [](Options& o, const char* v) { o.f16_fmu = v; } },
    { "--gps", [](Options& o, const char* v) { o.gps_fmu = v; } },
    { "--imu", [](Options& o, const char* v) { o.imu_fmu = v; } },
    { "--baro", [](Options& o, const char* v) { o.baro_fmu = v; } },
    { "--mag", [](Options& o, const char* v) { o.mag_fmu = v; } },
    { "--radalt", [](Options& o, const char* v) { o.radalt_fmu = v; } },
    { "--imu-rate", [](Options& o, const char* v) { o.imu_rate_hz = std::stod(v); } },
    { "--radalt-rate", [](Options& o, const char* v) { o.radalt_rate_hz = std::stod(v); } },
    { "--dyn-model", [](Options& o, const char* v) { o.dynamic_platform = std::stoi(v); } },
    { "--reacq", [](Options& o, const char* v) { o.reacquisition_time_s = std::stod(v); } },
    { "--lat0", [](Options& o, const char* v) { o.lat0_deg = std::stod(v); } },
    { "--lon0", [](Options& o, const char* v) { o.lon0_deg = std::stod(v); } },
    { "--alt0", [](Options& o, const char* v) { o.alt0_ft = std::stod(v); } },
    { "--vt0", [](Options& o, const char* v) { o.vt0_fps = std::stod(v); } },
    { "--heading0", [](Options& o, const char* v) { o.heading0_deg = std::stod(v); } },
    { "--roll0", [](Options& o, const char* v) { o.roll0_deg = std::stod(v); } },
    { "--stop", [](Options& o, const char* v) { o.stop_s = std::stod(v); } },
    { "--step", [](Options& o, const char* v) { o.step_s = std::stod(v); } },
    { "--csv", [](Options& o, const char* v) { o.csv_path = v; } },
    { "--rtf", [](Options& o, const char* v) { o.realtime_factor = std::stod(v); } },
} };

bool parse_args(int argc, char** argv, Options& options)
{
  // A check-case is a set of defaults, so it is applied before anything else on
  // the command line regardless of where --case appears. Applying it in order
  // would make `--alt0 25000 --case 12` silently discard the --alt0.
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (std::string_view(argv[i]) != "--case")
    {
      continue;
    }
    const auto trim_case = std::ranges::find(kTrimCases, std::string_view(argv[i + 1]), &TrimCase::id);
    if (trim_case == kTrimCases.end())
    {
      std::cerr << "unknown check-case: " << argv[i + 1] << " (this example flies the trim flyouts, 11 and 12; "
                << "the autopilot cases 13.x are examples/f16_autopilot_ecos)\n";
      return false;
    }
    options.alt0_ft = trim_case->alt0_ft;
    options.vt0_fps = trim_case->vt0_fps;
  }

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
    option->apply(options, argv[++i]);
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
  out << "check_case=" << options.check_case << "\n"
      << "dynamic_platform=" << options.dynamic_platform << "\n"
      << "cocom_limits_enabled=" << (options.cocom_limits ? 1 : 0) << "\n"
      << "reacquisition_time_s=" << options.reacquisition_time_s << "\n"
      << "lat0_deg=" << options.lat0_deg << "\n"
      << "lon0_deg=" << options.lon0_deg << "\n"
      << "alt0_ft=" << options.alt0_ft << "\n"
      << "vt0_fps=" << options.vt0_fps << "\n"
      << "heading0_deg=" << options.heading0_deg << "\n"
      << "roll0_deg=" << options.roll0_deg << "\n"
      << "step_s=" << options.step_s << "\n"
      << "stop_s=" << options.stop_s << "\n"
      << "imu_rate_hz=" << options.imu_rate_hz << "\n"
      << "radalt_rate_hz=" << options.radalt_rate_hz << "\n"
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

  for (const auto& [label, path] : { std::pair{ "f16", options.f16_fmu },
                                     std::pair{ "gps", options.gps_fmu },
                                     std::pair{ "imu", options.imu_fmu },
                                     std::pair{ "baro", options.baro_fmu },
                                     std::pair{ "mag", options.mag_fmu },
                                     std::pair{ "radalt", options.radalt_fmu } })
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
    ss.add_model("f16", options.f16_fmu.string());
    ss.add_model("gps", options.gps_fmu.string());
    ss.add_model("imu", options.imu_fmu.string());
    ss.add_model("baro", options.baro_fmu.string());
    ss.add_model("mag", options.mag_fmu.string());
    ss.add_model("radalt", options.radalt_fmu.string());

    // Geodetic position needs no modifier: Aetherion >= 0.13.0 publishes
    // out.lat_deg/out.lon_deg in the degrees the GPS FMU's inputs are named
    // for. Both ends carry the unit in the port name, which is the only reason
    // a bare connection here can be read as correct rather than merely
    // plausible -- and an Aetherion older than that fails at ss.load() naming
    // the port it cannot find, rather than silently feeding radians.
    ss.make_connection<double>("f16::out.lat_deg", "gps::latitude_deg");
    ss.make_connection<double>("f16::out.lon_deg", "gps::longitude_deg");
    ss.make_connection<double>("f16::out.alt_m", "gps::altitude_m");
    ss.make_connection<double>("f16::out.v_north_m_s", "gps::v_north_mps");
    ss.make_connection<double>("f16::out.v_east_m_s", "gps::v_east_mps");
    ss.make_connection<double>("f16::out.v_down_m_s", "gps::v_down_mps");

    // Body angular rates feed the IMU's gyroscope triad 1:1, and since
    // Aetherion 0.14.0 so does specific force. out.specificForce_* is
    // (F_aero + F_thrust)/m at the CG in body axes, with gravitation
    // structurally excluded -- exactly what the IMU FMU's inputs are named
    // for. Before that this example summed thrust and aero itself and divided
    // by mass in the stepping loop, which meant asserting host-side that
    // thrust acts along body +X. The plant knows its own installation
    // geometry; the bench does not, and no longer has to guess.
    ss.make_connection<double>("f16::out.p_rad_s", "imu::p_rad_s");
    ss.make_connection<double>("f16::out.q_rad_s", "imu::q_rad_s");
    ss.make_connection<double>("f16::out.r_rad_s", "imu::r_rad_s");
    ss.make_connection<double>("f16::out.specificForce_x_m_s2", "imu::f_x_mps2");
    ss.make_connection<double>("f16::out.specificForce_y_m_s2", "imu::f_y_mps2");
    ss.make_connection<double>("f16::out.specificForce_z_m_s2", "imu::f_z_mps2");

    // The BMP390's whole truth interface is altitude: its measurement model
    // owns the atmosphere (ISA) and the part's error model, and everything
    // else about its behaviour -- rate included -- is register state the
    // flight computer programs over I2C.
    ss.make_connection<double>("f16::out.alt_m", "baro::h_m");

    // The radar altimeter measures height above *ground*, and this scenario
    // carries no terrain model, so MSL altitude goes in unmodified. At Kitty
    // Hawk that is right to within a few metres of coastal elevation; over
    // terrain it would not be, and the connection is left bare rather than
    // routed through a nominal offset so that the approximation is one
    // readable line instead of a buried constant. Case 11's 10 013 ft is
    // 3052 m, well inside the part's 6000 m tracking range. Case 12's
    // 30 013 ft is 9148 m, well outside it, and the part reports loss of track
    // for the whole flight -- a sensor that is working and has nothing to say.
    ss.make_connection<double>("f16::out.alt_m", "radalt::h_agl_m");

    // The magnetometer's die temperature: ambient air, near enough for a part
    // whose temperature channel quantizes at 0.8 C. The field itself has no
    // plant output to connect -- it is computed in the stepping loop below,
    // because it depends on position *and* attitude and an Ecos connection
    // modifier sees only its single source variable.
    const std::function<double(const double&)> kelvin2celsius = [](const double& kelvin) { return kelvin - 273.15; };
    ss.make_connection<double>("f16::out.T_K", "mag::temperature_c", kelvin2celsius);

    // The check-case's trim condition: Kitty Hawk, NC, heading 45 deg, a
    // -0.172 deg residual bank, and the case's altitude and airspeed (see
    // kTrimCases). These are the conditions the published reference
    // trajectories (Aetherion's data/Atmos_11_TrimCheckSubsonicF16/ and
    // data/Atmos_12_TrimCheckSupersonicF16/) are tabulated from, so departing
    // from them means the run can no longer be checked against anything.
    //
    // There is no pitch parameter: pitch at trim *is* the angle of attack the
    // FMU's own trim solver finds, and forcing it from outside would either
    // duplicate that solve or contradict it. The published initial pitch
    // (2.643 deg for case 11, -0.737 deg nose-down for case 12) is therefore
    // an output to check, not an input to set -- which makes it one of the
    // more informative numbers in the whole run.
    //
    // solver.max_step_s is left at its default (0 = one implicit Radau step
    // per communication step); the integrator is L-stable, and capping
    // sub-steps costs wall time for no visible change at these dynamics.
    ecos::parameter_set trim_point;
    trim_point["f16::lat0_deg"] = options.lat0_deg;
    trim_point["f16::lon0_deg"] = options.lon0_deg;
    trim_point["f16::alt0_ft"] = options.alt0_ft;
    trim_point["f16::vt0_fps"] = options.vt0_fps;
    trim_point["f16::heading0_deg"] = options.heading0_deg;
    trim_point["f16::roll0_deg"] = options.roll0_deg;
    // Not trim geometry, but sim->init() applies exactly one named parameter
    // set, so the sensor configuration rides along here.
    trim_point["imu::sample_rate_hz"] = options.imu_rate_hz;
    trim_point["radalt::sample_rate_hz"] = options.radalt_rate_hz;
    // Written explicitly even though these match the GPS FMU's own defaults:
    // whether the receiver keeps a fix through this flight is the scenario's
    // most consequential setting, and it should be readable here rather than
    // inherited silently from modelDescription.xml. On case 11 the answer is
    // "yes, throughout" -- which is exactly why that run can serve as the
    // reference the other sensors are judged against. On case 12 it is
    // "never": same three settings, so the difference is the flight, not the
    // configuration.
    trim_point["gps::dynamic_platform"] = options.dynamic_platform;
    trim_point["gps::cocom_limits_enabled"] = options.cocom_limits;
    trim_point["gps::reacquisition_time_s"] = options.reacquisition_time_s;
    ss.add_parameter_set("trimPoint", trim_point);

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(options.step_s));

    sim->init("trimPoint");

    // Logged by TruthLogger rather than Ecos' csv_writer, which rounds every
    // real -- geodetic angles included -- to six decimal places. See the class
    // comment for what that costs in degrees, and what it used to cost when
    // these two ports were radians.
    TruthLogger truth_log(options.csv_path,
                          *sim,
                          { "f16::out.alt_m",
                            "f16::out.lat_deg",
                            "f16::out.lon_deg",
                            "f16::out.v_north_m_s",
                            "f16::out.v_east_m_s",
                            "f16::out.v_down_m_s",
                            // Attitude is check-case content in its own right here, not
                            // just an input to the magnetic-field computation: on an
                            // open-loop trim flyout the Euler angles are where the drift
                            // shows up first, and the published data tabulates all three.
                            "f16::out.yaw_rad",
                            "f16::out.pitch_rad",
                            "f16::out.roll_rad",
                            "f16::out.p_rad_s",
                            "f16::out.q_rad_s",
                            "f16::out.r_rad_s",
                            "f16::out.vt_m_s",
                            "f16::out.alpha_deg",
                            "f16::out.beta_deg",
                            "f16::out.mach",
                            "f16::out.qbar_Pa",
                            "f16::out.thrust_N",
                            "f16::out.mass_kg",
                            // What the IMU FMU actually receives: the plant's body-frame
                            // specific force, one step behind plant truth like every
                            // other connection.
                            "imu::f_x_mps2",
                            "imu::f_y_mps2",
                            "imu::f_z_mps2",
                            // Likewise for the magnetometer: the body-frame truth field
                            // the host computed and wrote, so a decoded sample can be
                            // compared against what the part was actually given.
                            "mag::b_x_ut",
                            "mag::b_y_ut",
                            "mag::b_z_ut",
                            // And the height the radar altimeter was handed, which on
                            // this scenario is MSL altitude -- logged separately from
                            // out.alt_m so the terrain approximation stays visible in
                            // the data rather than only in the connection comment.
                            "radalt::h_agl_m" },
                          {});
    write_run_config(options.csv_path, options);

    auto* altitude = sim->get_real_property("f16::out.alt_m");
    auto* mach = sim->get_real_property("f16::out.mach");
    auto* airspeed = sim->get_real_property("f16::out.vt_m_s");

    // Magnetic-field plumbing: the field a magnetometer sees depends on where
    // the vehicle is *and* how it is pointing -- four plant outputs -- and an
    // Ecos connection modifier sees only its single source variable. So the
    // host computes the field after every step and writes the magnetometer's
    // inputs directly, giving the same one-communication-step transport delay
    // a connection would. GeomagneticDipole owns the model;
    // geomagnetic_field.hpp is explicit about it being a centered dipole
    // rather than the WMM, and about what that costs at this particular launch
    // site.
    auto* latitude = sim->get_real_property("f16::out.lat_deg");
    auto* longitude = sim->get_real_property("f16::out.lon_deg");
    auto* yaw = sim->get_real_property("f16::out.yaw_rad");
    auto* pitch = sim->get_real_property("f16::out.pitch_rad");
    auto* roll = sim->get_real_property("f16::out.roll_rad");
    auto* mag_bx = sim->get_real_property("mag::b_x_ut");
    auto* mag_by = sim->get_real_property("mag::b_y_ut");
    auto* mag_bz = sim->get_real_property("mag::b_z_ut");

    // What the run is judged on. On a trim flyout nothing "happens", so the
    // summary reports how far the aircraft departed from the condition it was
    // trimmed at -- which is the whole content of the check-case.
    const double alt0_m = altitude->get_value();
    double max_alt_excursion_m = 0.0;
    double max_speed_excursion_mps = 0.0;
    const double vt0_mps = airspeed->get_value();
    const auto wall_start = std::chrono::steady_clock::now();

    std::cout << "[cosim] f16:    " << options.f16_fmu.string() << "\n"
              << "[cosim] gps:    " << options.gps_fmu.string() << "\n"
              << "[cosim] imu:    " << options.imu_fmu.string() << "\n"
              << "[cosim] baro:   " << options.baro_fmu.string() << "\n"
              << "[cosim] mag:    " << options.mag_fmu.string() << "\n"
              << "[cosim] radalt: " << options.radalt_fmu.string() << "\n"
              << "[cosim] step " << options.step_s << " s (" << 1.0 / options.step_s << " Hz GPS, "
              << options.imu_rate_hz << " Hz IMU, " << options.radalt_rate_hz
              << " Hz radalt; BMP390 and MMC5983MA at the rates the flight computer programs), stop " << options.stop_s
              << " s\n"
              << "[cosim] check-case " << options.check_case << ": "
              << std::ranges::find(kTrimCases, std::string_view(options.check_case), &TrimCase::id)->summary
              << "\n"
              // Six decimals, as published: the stream default of six *significant* digits prints 36.019167 as
              // 36.0192, which is exactly the rounded FMU default the Options comment explains this example avoids.
              << "[cosim] plant: trimmed at " << std::fixed << std::setprecision(6) << options.lat0_deg << " deg N / "
              << options.lon0_deg << " deg E, " << std::defaultfloat << options.alt0_ft << " ft, " << options.vt0_fps
              << " ft/s, heading " << options.heading0_deg << " deg\n"
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

    long print_counter = 0;
    const long print_period = std::lround(10.0 / options.step_s);  // one status line per 10 s of sim time
    truth_log.write_row(sim->iterations(), sim->time());           // the state at t = 0, before any stepping
    while (sim->time() < options.stop_s)
    {
      sim->step();

      // Where the vehicle is and how it is pointing, turned into the body-frame
      // field the part is immersed in. Written after the step, so it carries the
      // same one-communication-step transport delay every Ecos connection does.
      const FieldNed field_ned =
          GeomagneticDipole::field_ned(latitude->get_value(), longitude->get_value(), altitude->get_value());
      const FieldBody field_body =
          GeomagneticDipole::to_body(field_ned, yaw->get_value(), pitch->get_value(), roll->get_value());
      mag_bx->set_value(field_body.x_ut);
      mag_by->set_value(field_body.y_ut);
      mag_bz->set_value(field_body.z_ut);
      // After the specific-force write, so the logged imu:: inputs are the ones
      // the FMU will sample on the next step -- matching what csv_writer
      // recorded as a post-step listener.
      truth_log.write_row(sim->iterations(), sim->time());

      const double altitude_m = altitude->get_value();
      max_alt_excursion_m = std::max(max_alt_excursion_m, std::abs(altitude_m - alt0_m));
      max_speed_excursion_mps = std::max(max_speed_excursion_mps, std::abs(airspeed->get_value() - vt0_mps));
      if (++print_counter % print_period == 0)
      {
        std::cout << "[cosim] t=" << sim->time() << " s  alt=" << altitude_m << " m  vt=" << airspeed->get_value()
                  << " m/s  mach=" << mach->get_value() << "  hdg=" << yaw->get_value() * 180.0 / std::numbers::pi
                  << " deg\n";
      }
      if (options.realtime_factor > 0.0)
      {
        const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(sim->time() / options.realtime_factor));
        std::this_thread::sleep_until(target);
      }
    }

    // The BMP390 FMU's rating diagnostic, read while the instance is still
    // live. The part converts outside its rated envelope without complaint
    // -- these counters are how the bench knows it happened. Zero is a
    // result too: on case 11 it says the whole flight sat inside the
    // envelope, which is what makes that run the cross-sensor reference.
    const int baro_conversions = sim->get_int_property("baro::conversions")->get_value();
    const int baro_out_of_rating = sim->get_int_property("baro::conversions_out_of_rating")->get_value();

    sim->terminate();

    const long imu_frames_per_step = std::lround(options.step_s * options.imu_rate_hz);
    const long radalt_returns_per_step = std::lround(options.step_s * options.radalt_rate_hz);
    // One NAV-PVT frame per step regardless of fix validity -- a receiver
    // outside its envelope keeps talking, it just stops claiming a solution.
    // On case 11 it never stops claiming one; on case 12 it never starts.
    // Either way the frame count is the step count, which is the point.
    std::cout << "[cosim] done: " << sim->iterations() << " steps, " << sim->iterations()
              << " UBX-NAV-PVT frames emitted, " << sim->iterations() * std::max(1L, imu_frames_per_step)
              << " IMU samples buffered for the SPI controller and "
              << sim->iterations() * std::max(1L, radalt_returns_per_step) << " radar returns sent\n"
              << "[cosim] the BMP390 and the MMC5983MA each answered at whatever rate the flight computer programmed "
                 "them to\n"
              << "[cosim] the BMP390 latched " << baro_conversions << " conversions, " << baro_out_of_rating
              << " outside its rated envelope (300-1250 hPa, -40..+85 degC)\n"
              // A trim flyout has no events, so what is worth reporting is how
              // far it drifted from the condition it was trimmed at. The
              // published solutions do drift, by tens of metres and fractions
              // of a degree over the window; a run that holds altitude to the
              // centimetre has not reproduced the check-case, it has replaced
              // it with a different problem.
              << "[cosim] trim drift: altitude " << max_alt_excursion_m
              << " m max excursion, true airspeed "
              // Final minus initial, so the sign reads as the turn: an aircraft that drifts from 45 to 45.47 deg
              // reports +0.47. The first version subtracted the other way round and printed that as -0.47.
              << max_speed_excursion_mps << " m/s, heading change " << std::showpos
              << yaw->get_value() * 180.0 / std::numbers::pi - options.heading0_deg << std::noshowpos
              << " deg at cut-off\n"
              << "[cosim] plant truth written to " << options.csv_path.string() << "\n";
  }
  catch (const std::exception& ex)
  {
    std::cerr << "error: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
