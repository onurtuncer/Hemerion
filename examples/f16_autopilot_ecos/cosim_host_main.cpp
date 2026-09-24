// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file cosim_host_main.cpp
/// @brief Ecos co-simulation host closing the loop between Aetherion's
/// F16Plant.fmu and F16Autopilot.fmu, with all five of Hemerion's
/// hardware-simulator FMUs riding the same truth.
///
/// The scenarios are NASA TM-2015-218675's four **closed-loop autopilot
/// check-cases**, selected with --case. All four start from check-case 11's
/// trim condition (Kitty Hawk, 10 013 ft, 335 KTAS, heading 45 deg) and step
/// exactly one command:
///
///   13.1  altitude   +100 ft (10 013 -> 10 113 ft)      at t =  5 s
///   13.2  airspeed   KEAS -> 277 kt (from trim ~288)    at t =  5 s
///   13.3  course     45 -> 60 deg                       at t = 15 s
///   13.4  side-step  2000 ft right of the courseline    at t = 20 s
///
/// Topology: examples/f16_trim_ecos's open-loop wiring, plus the loop --
///
///   F16Plant.fmu  ── out.* (11 signals) ──>  F16Autopilot.fmu
///        ^                                        │
///        └────────── ctrl.* (4 surfaces) ─────────┘
///                                    cmd.* <── this host (the schedule)
///
/// and the five sensor FMUs consume plant truth exactly as in the trim
/// example, feeding the same f16_flight_computer executable -- this example
/// deliberately has no flight computer of its own.
///
/// **Why the communication step is 0.01 s here, not the trim example's
/// 0.1 s.** Aetherion's standalone 13.x examples evaluate their DML LQR as a
/// zero-order-hold discrete controller at the integration step rate --
/// feedback read from the *current* state, surfaces applied over that same
/// step, no transport delay -- at a recommended 0.02 s (their headers warn
/// that dt = 0.1 diverges: the LQR plant is stiff). Ecos cannot reproduce
/// zero delay: it is a Jacobi master -- every instance steps, *then*
/// connections transfer -- so the plant flies [t, t+h] on surfaces computed
/// from its state at t-h, one communication step late, and no stepping order
/// changes that. What the host can choose is h. A one-step delay at h costs
/// about the loop phase of zero-delay ZOH at 2h, so h = 0.01 s puts this
/// co-simulation at the standalone's own recommended cadence. The control
/// law itself is stateless (DAVEMLControlModel::evaluate() is const -- pure
/// LQR gains, no integrators), so cadence and delay are the *only*
/// differences between the two drivers of the same DML.
///
/// **The sensors do not follow the loop down to 0.01 s.** Ecos'
/// fixed_step_algorithm takes a per-instance step-size hint and steps that
/// instance every Nth base step with dt = N x base. The GPS FMU emits one
/// NAV-PVT per do_step whatever dt is, so a 0.1 s hint keeps the 10 Hz
/// navigation rate of the trim example; the radar altimeter emits
/// max(1, lround(dt x rate)) frames per step, which at dt = 0.01 would clamp
/// to 100 Hz, so its hint is one sample period (0.04 s at the default
/// 25 Hz). The IMU at 100 Hz emits exactly one frame per base step, and the
/// BMP390 and MMC5983MA convert at whatever ODR the flight computer programs
/// into their registers regardless of stepping -- base rate for all three.
///
/// **cmd.latOffset_ft is feedback, not a setpoint.** The standalone computes
/// the aircraft's lateral deviation from the original courseline every step
/// (flat earth about the initial position, R = 6 371 000 m) and feeds the
/// controller `deviation - commanded_step`. This host does the same, writing
/// the input after every step exactly like the magnetometer field -- the one
/// signal in the loop the plant cannot publish because it depends on where
/// the flight *began*. Cases 13.1-13.3 write a constant 0.
///
/// **Start-up.** The plant's ctrl.* inputs start at 0 and it does not
/// publish its trim deflections, while Ecos' init rounds hand the plant
/// whatever the autopilot computed during its own initialisation. The
/// trimPoint parameter set therefore seeds the autopilot's fb.*/cmd.* inputs
/// with the trim condition, so that its init-time output is the LQR's own
/// answer at trim -- approximately the trim deflections, the same property
/// the standalone relies on when it engages the controller at t = 0. The
/// initial KEAS command is then refined after init() from the plant's actual
/// density and airspeed, the way the standalone computes it.
///
/// Everything else -- FMU preflight, the truth logger, run-config sidecar,
/// the geomagnetic dipole -- matches examples/f16_trim_ecos, and the
/// magnetic-field model is included from there rather than copied.

#include "ecos/algorithm/fixed_step_algorithm.hpp"
#include "ecos/logger/logger.hpp"
#include "ecos/structure/simulation_structure.hpp"

// Deliberately one of Ecos' *internal* headers (its src/ tree, put on this
// target's include path by CMakeLists.txt) -- same preflight, same reasoning
// as examples/f16_trim_ecos/cosim_host_main.cpp.
#include "util/unzipper.hpp"

// The centered-dipole field model, shared with the sibling example rather
// than copied: both examples fly the same aircraft from the same site, and a
// third copy of the model would be a third place for it to drift.
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

// Compile-time defaults injected by CMakeLists.txt; every one can be
// overridden on the command line, so an empty default (FMU not found at
// configure time) is not fatal until the run actually needs it.
#ifndef HEMERION_F16_FMU_PATH
#define HEMERION_F16_FMU_PATH ""
#endif
#ifndef HEMERION_AP_FMU_PATH
#define HEMERION_AP_FMU_PATH ""
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

constexpr double kPi = std::numbers::pi;
constexpr double kFtPerM = 1.0 / 0.3048;
// The standalone's own conversion constants (F16AltitudeChangeSimulator):
// matching them exactly is what makes a KEAS printed here comparable to one
// printed there.
constexpr double kKtMps = 0.5144444;
constexpr double kRhoSlKgM3 = 1.225;
// The standalone's flat-earth radius for the lateral-deviation feedback.
constexpr double kLatDevEarthRadiusM = 6'371'000.0;

/// The GPS FMU's error-model parameters as one record, so the Ecos parameter
/// set and the run's .config sidecar cannot disagree about what the receiver
/// was -- and plot_results.py can draw the autocorrelation a run *should*
/// show from the sidecar alone.
struct GpsErrorModel
{
  const char* name;
  double horizontal_pos_noise_m;
  double vertical_pos_noise_m;
  double speed_noise_mps;
  double course_noise_deg;
  double horizontal_pos_correlated_m;
  double vertical_pos_correlated_m;
  double position_correlation_time_s;
  double speed_correlated_mps;
  double course_correlated_deg;
  double velocity_correlation_time_s;
  double accuracy_scale;
};

/// The FMU's own defaults: white per epoch, an honest hAcc/vAcc. Written to
/// the parameter set explicitly even though the FMU would default to them, so
/// the sidecar is always a complete record.
constexpr GpsErrorModel kWhiteReceiver{ "white", 1.5, 3.0, 0.1, 1.0, 0.0, 0.0, 100.0, 0.0, 0.0, 10.0, 1.0 };

/// The realistic receiver. The total 1-sigma per channel is within 2 % of the
/// default's -- nothing on a page changes by magnitude -- but most of it now
/// lives in a slow Gauss-Markov term (tau 100 s on position, 10 s on
/// velocity), and hAcc/vAcc report 70 % of the truth, as a receiver's own
/// estimate tends to. Spelled out here rather than as an FMU-side preset so
/// the example reads end to end; the values are tabulated in
/// doc/sensor_models.rst.
constexpr GpsErrorModel kCorrelatedReceiver{ "correlated", 0.3, 0.6, 0.05, 0.3, 1.5, 3.0, 100.0, 0.1, 1.0, 10.0, 0.7 };

struct Options
{
  std::filesystem::path f16_fmu = HEMERION_F16_FMU_PATH;
  std::filesystem::path ap_fmu = HEMERION_AP_FMU_PATH;
  std::filesystem::path gps_fmu = HEMERION_GPS_FMU_PATH;
  std::filesystem::path imu_fmu = HEMERION_IMU_FMU_PATH;
  std::filesystem::path baro_fmu = HEMERION_BMP390_FMU_PATH;
  std::filesystem::path mag_fmu = HEMERION_MMC5983MA_FMU_PATH;
  std::filesystem::path radalt_fmu = HEMERION_RADALT_FMU_PATH;
  std::filesystem::path csv_path = "results/f16_truth.csv";
  std::string check_case = "13.1";
  // Per-case default (the longest published reference window); --stop overrides.
  double stop_s = 60.0;
  // Check-case 11's trim condition, shared by all four autopilot cases. Six
  // decimals, as published -- see the trim example's Options for why the
  // FMU's own four-decimal defaults are not used.
  double lat0_deg = 36.019167;  // Kitty Hawk, NC
  double lon0_deg = -75.674444;
  double alt0_ft = 10013.0;
  double vt0_fps = 565.685;    // 335.15 KTAS
  double heading0_deg = 45.0;  // north-east; also the reference courseline for 13.4
  double roll0_deg = -0.172;
  // Base communication step: the closed loop's cadence (see the file header
  // for why 0.01 s and not the trim example's 0.1 s).
  double step_s = 0.01;
  double gps_period_s = 0.1;     // GPS step-size hint == NAV-PVT period (10 Hz)
  double imu_rate_hz = 100.0;    // IMU output data rate
  double radalt_rate_hz = 25.0;  // radar altimeter pulse rate; its hint is 1/rate
  double realtime_factor = 0.0;  // 0 = run as fast as possible
  // Receiver dynamics envelope -- identical to the trim example, and far
  // inside every limit on all four cases (the maneuvers stay subsonic at
  // ~3 km).
  int dynamic_platform = 8;
  bool cocom_limits = true;
  double reacquisition_time_s = 2.0;
  // GPS error model: the FMU's default white-only receiver, or the realistic
  // time-correlated one (doc/sensor_models.rst). Off by default so the
  // receiver -- and every figure drawn from it -- stays the one this example
  // was verified with until the change is asked for.
  bool gps_correlated_errors = false;
  int gps_seed = 0;  // 0 = nondeterministic; any other value reproduces the receiver's errors
};

/// @brief The four closed-loop autopilot check-cases.
///
/// Every command schedule below is read out of Aetherion's standalone
/// sources (src/Examples/F16{Altitude,Airspeed,Heading,LateralSideStep}*
/// .cpp), which in turn implement the NASA scenario definitions. A NaN KEAS
/// command means "hold the trim KEAS", whose value only exists after the
/// plant has trimmed -- it is computed post-init from the plant's own
/// density and airspeed, the way the standalone computes it.
struct AutopilotCase
{
  std::string_view id;
  std::string_view summary;
  double stop_s;            ///< Longest published reference window [s].
  double alt_cmd_ft;        ///< Altitude command after alt_step_time_s [ft].
  double alt_step_time_s;   ///< 0 = commanded from the start.
  double keas_cmd_kt;       ///< KEAS command after keas_step_time_s; NaN = hold trim.
  double keas_step_time_s;  ///< 0 = commanded from the start.
  double chi_cmd_deg;       ///< Course command after chi_step_time_s [deg].
  double chi_step_time_s;   ///< 0 = commanded from the start.
  double lat_step_ft;       ///< Lateral side-step, +right of course; 0 = none.
  double lat_step_time_s;   ///< When the side-step is commanded [s].
};

constexpr double kHoldTrimKeas = std::numeric_limits<double>::quiet_NaN();

constexpr std::array<AutopilotCase, 4> kAutopilotCases = { {
    { "13.1",
      "subsonic altitude change, +100 ft at t = 5 s",
      60.0,  //
      10113.0,
      5.0,
      kHoldTrimKeas,
      0.0,
      45.0,
      0.0,
      0.0,
      0.0 },
    { "13.2",
      "subsonic airspeed change, KEAS -> 277 kt at t = 5 s",
      60.0,  //
      10013.0,
      0.0,
      277.0,
      5.0,
      45.0,
      0.0,
      0.0,
      0.0 },
    { "13.3",
      "subsonic heading change, course 45 -> 60 deg at t = 15 s",
      240.0,  //
      10013.0,
      0.0,
      kHoldTrimKeas,
      0.0,
      60.0,
      15.0,
      0.0,
      0.0 },
    { "13.4",
      "subsonic lateral side-step, 2000 ft right at t = 20 s",
      240.0,  //
      10013.0,
      0.0,
      kHoldTrimKeas,
      0.0,
      45.0,
      0.0,
      2000.0,
      20.0 },
} };

void print_usage()
{
  std::cout << "usage: f16_autopilot_cosim [--case 13.1|13.2|13.3|13.4] [--f16 <F16Plant.fmu>] [--ap "
               "<F16Autopilot.fmu>]\n"
               "                           [--gps <fmu>] [--imu <fmu>] [--baro <fmu>] [--mag <fmu>] [--radalt <fmu>]\n"
               "                           [--imu-rate <hz>] [--radalt-rate <hz>] [--gps-period <s>]\n"
               "                           [--dyn-model <code>] [--no-cocom] [--reacq <s>]\n"
               "                           [--gps-errors white|correlated] [--gps-seed <n>]\n"
               "                           [--stop <s>] [--step <s>] [--csv <file>] [--rtf <x>]\n"
               "\n"
               "  --case      NASA TM-2015-218675 closed-loop check-case (default 13.1):\n"
               "                13.1 = +100 ft altitude step at t=5 s        (60 s window)\n"
               "                13.2 = KEAS step to 277 kt at t=5 s          (60 s window)\n"
               "                13.3 = course step 45 -> 60 deg at t=15 s    (240 s window)\n"
               "                13.4 = 2000 ft lateral side-step at t=20 s   (240 s window)\n"
               "              Sets the default stop time; an explicit --stop still wins\n"
               "  --f16       path to Aetherion's F16Plant.fmu (default: configure-time location)\n"
               "  --ap        path to Aetherion's F16Autopilot.fmu (default: configure-time location)\n"
               "  --gps/--imu/--baro/--mag/--radalt  paths to the packaged Hemerion sensor FMUs\n"
               "  --imu-rate  IMU output data rate [Hz] (default 100 = one frame per base step)\n"
               "  --radalt-rate  radar altimeter pulse rate [Hz] (default 25; the FMU is stepped\n"
               "              once per sample period, so rates whose period is not a whole number\n"
               "              of base steps are rounded to one that is)\n"
               "  --gps-period  NAV-PVT period [s] (default 0.1 = the 10 Hz of the trim example;\n"
               "              must be a whole number of base steps)\n"
               "  --dyn-model u-blox dynModel platform code (default 8 = airborne <4 g; every\n"
               "              case here stays far inside it)\n"
               "  --no-cocom  clear the COCOM export cut-off (default: in force, never approached)\n"
               "  --reacq     re-acquisition hold-off after any limit trips [s] (default 2)\n"
               "  --gps-errors  white (default: the FMU's white-only receiver) or correlated: time-correlated\n"
               "              Gauss-Markov position/velocity errors and an optimistic reported accuracy\n"
               "  --gps-seed  error-model RNG seed (default 0 = nondeterministic)\n"
               "  --stop      simulation stop time [s] (default: the case's reference window)\n"
               "  --step      base communication step [s] (default 0.01; this is the closed\n"
               "              loop's sample period AND its transport delay -- see the file header)\n"
               "  --csv       plant truth output CSV (default results/f16_truth.csv)\n"
               "  --rtf       real-time factor pacing, e.g. 1 = wall-clock speed; 0 = unpaced (default)\n";
}

struct ValueOption
{
  std::string_view name;
  void (*apply)(Options&, const char*);
};

constexpr std::array<ValueOption, 21> kValueOptions = { {
    // Only records the choice; the case's defaults were applied in parse_args()'s first pass.
    { "--case", [](Options& o, const char* v) { o.check_case = v; } },
    { "--f16", [](Options& o, const char* v) { o.f16_fmu = v; } },
    { "--ap", [](Options& o, const char* v) { o.ap_fmu = v; } },
    { "--gps", [](Options& o, const char* v) { o.gps_fmu = v; } },
    { "--imu", [](Options& o, const char* v) { o.imu_fmu = v; } },
    { "--baro", [](Options& o, const char* v) { o.baro_fmu = v; } },
    { "--mag", [](Options& o, const char* v) { o.mag_fmu = v; } },
    { "--radalt", [](Options& o, const char* v) { o.radalt_fmu = v; } },
    { "--imu-rate", [](Options& o, const char* v) { o.imu_rate_hz = std::stod(v); } },
    { "--radalt-rate", [](Options& o, const char* v) { o.radalt_rate_hz = std::stod(v); } },
    { "--gps-period", [](Options& o, const char* v) { o.gps_period_s = std::stod(v); } },
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
    { "--gps-seed", [](Options& o, const char* v) { o.gps_seed = std::stoi(v); } },
    { "--stop", [](Options& o, const char* v) { o.stop_s = std::stod(v); } },
    { "--step", [](Options& o, const char* v) { o.step_s = std::stod(v); } },
    { "--csv", [](Options& o, const char* v) { o.csv_path = v; } },
    { "--rtf", [](Options& o, const char* v) { o.realtime_factor = std::stod(v); } },
    // Position only -- kept for the same reason the trim example has it (a
    // standalone comparison needs the standalone's five-decimal rounding).
    // Altitude, airspeed, heading and roll are deliberately NOT options:
    // all four cases *are* the case-11 trim, and letting those drift would
    // silently detach the run from every reference it can be checked against.
    { "--lat0", [](Options& o, const char* v) { o.lat0_deg = std::stod(v); } },
    { "--lon0", [](Options& o, const char* v) { o.lon0_deg = std::stod(v); } },
} };

const AutopilotCase* find_case(std::string_view id)
{
  const auto found = std::ranges::find(kAutopilotCases, id, &AutopilotCase::id);
  return found != kAutopilotCases.end() ? &*found : nullptr;
}

bool parse_args(int argc, char** argv, Options& options)
{
  // A check-case is a set of defaults, so it is applied before anything else
  // on the command line regardless of where --case appears (same reasoning
  // as the trim example: `--stop 30 --case 13.3` must keep the 30).
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (std::string_view(argv[i]) != "--case")
    {
      continue;
    }
    const AutopilotCase* trim_case = find_case(argv[i + 1]);
    if (trim_case == nullptr)
    {
      std::cerr << "unknown check-case: " << argv[i + 1]
                << " (this example flies the closed-loop autopilot cases 13.1-13.4; "
                << "the open-loop trim flyouts 11 and 12 are examples/f16_trim_ecos)\n";
      return false;
    }
    options.check_case = trim_case->id;
    options.stop_s = trim_case->stop_s;
  }

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      print_usage();
      return false;
    }
    if (arg == "--no-cocom")
    {
      options.cocom_limits = false;
      continue;
    }
    const auto option = std::ranges::find(kValueOptions, std::string_view(arg), &ValueOption::name);
    if (option == kValueOptions.end())
    {
      std::cerr << "unknown option: " << arg << "\n";
      print_usage();
      return false;
    }
    if (i + 1 >= argc)
    {
      std::cerr << arg << " needs a value\n";
      return false;
    }
    try
    {
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
    catch (const std::exception&)
    {
      std::cerr << "invalid value for " << arg << ": " << argv[i] << "\n";
      return false;
    }
  }
  return true;
}

/// Same environmental preflight as the trim example: prove one .fmu can be
/// unpacked with Ecos' own unzip() before six loads depend on it, and name
/// the archiver and the fix when it cannot. See f16_trim_ecos for the two
/// Windows PATH/SHELL traps this catches.
[[nodiscard]] bool preflight_fmu_extraction(const std::filesystem::path& archive)
{
  std::error_code ec;
  const std::filesystem::path probe_dir =
      std::filesystem::temp_directory_path(ec) /
      ("hemerion_unzip_probe_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  if (ec || !std::filesystem::create_directories(probe_dir, ec) || ec)
  {
    return true;  // a temp-dir problem is Ecos' to report, not this probe's
  }
  const bool extracted = ecos::unzip(archive, probe_dir);
  std::filesystem::remove_all(probe_dir, ec);
  if (!extracted)
  {
    std::cerr << "error: cannot unpack " << archive.string() << "\n"
              << "       Ecos spawns 'tar' (or 'unzip' under a bash SHELL) for every .fmu; that call\n"
                 "       failed before the simulation started. See examples/f16_trim_ecos/README.md\n"
                 "       for the two Windows PATH/SHELL traps that produce this and their fixes.\n";
    return false;
  }
  return true;
}

/// Run-configuration sidecar, `<truth stem>.config` beside the CSV -- same
/// shape and same reasoning as the trim example: a figure that does not say
/// which case and which schedule produced it will be read as another one.
void write_run_config(const std::filesystem::path& csv_path,
                      const Options& options,
                      const AutopilotCase& ap_case,
                      double keas_trim_kt)
{
  std::filesystem::path config_path = csv_path;
  config_path.replace_extension(".config");
  std::ofstream out(config_path);
  if (!out)
  {
    std::cerr << "warning: cannot write " << config_path.string() << "; figures will be unlabelled\n";
    return;
  }
  // Written in full so a figure can be checked against the model that made it.
  const GpsErrorModel& receiver = options.gps_correlated_errors ? kCorrelatedReceiver : kWhiteReceiver;
  out << "check_case=" << options.check_case << "\n"
      << "alt_cmd_ft=" << ap_case.alt_cmd_ft << "\n"
      << "alt_step_time_s=" << ap_case.alt_step_time_s << "\n"
      << "keas_cmd_kt=" << (std::isnan(ap_case.keas_cmd_kt) ? keas_trim_kt : ap_case.keas_cmd_kt) << "\n"
      << "keas_step_time_s=" << ap_case.keas_step_time_s << "\n"
      << "keas_trim_kt=" << keas_trim_kt << "\n"
      << "chi_cmd_deg=" << ap_case.chi_cmd_deg << "\n"
      << "chi_step_time_s=" << ap_case.chi_step_time_s << "\n"
      << "lat_step_ft=" << ap_case.lat_step_ft << "\n"
      << "lat_step_time_s=" << ap_case.lat_step_time_s << "\n"
      << "dynamic_platform=" << options.dynamic_platform << "\n"
      << "cocom_limits_enabled=" << (options.cocom_limits ? 1 : 0) << "\n"
      << "reacquisition_time_s=" << options.reacquisition_time_s << "\n"
      << "gps_error_model=" << receiver.name << "\n"
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
      << "lat0_deg=" << options.lat0_deg << "\n"
      << "lon0_deg=" << options.lon0_deg << "\n"
      << "alt0_ft=" << options.alt0_ft << "\n"
      << "vt0_fps=" << options.vt0_fps << "\n"
      << "heading0_deg=" << options.heading0_deg << "\n"
      << "roll0_deg=" << options.roll0_deg << "\n"
      << "step_s=" << options.step_s << "\n"
      << "gps_period_s=" << options.gps_period_s << "\n"
      << "stop_s=" << options.stop_s << "\n"
      << "imu_rate_hz=" << options.imu_rate_hz << "\n"
      << "radalt_rate_hz=" << options.radalt_rate_hz << "\n"
      << "realtime_factor=" << options.realtime_factor << "\n";
}

/// Truth log written by the host, not Ecos' csv_writer -- same class, same
/// full-precision reasoning as the trim example (geodetic angles must not
/// carry the logger's own rounding).
class TruthLogger
{
public:
  TruthLogger(const std::filesystem::path& path,
              const ecos::simulation& sim,
              const std::vector<std::string>& real_variables)
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
    out_ << std::setprecision(std::numeric_limits<double>::max_digits10);
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
    out_ << "\n";
  }

  void write_row(std::uint64_t iterations, double time_s)
  {
    out_ << iterations << ", " << time_s;
    for (const auto* property : reals_)
    {
      out_ << ", " << property->get_value();
    }
    out_ << "\n";
  }

private:
  std::ofstream out_;
  std::vector<ecos::property_t<double>*> reals_;
};

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  if (!parse_args(argc, argv, options))
  {
    return EXIT_FAILURE;
  }
  const AutopilotCase& ap_case = *find_case(options.check_case);

  for (const auto& [label, path] : { std::pair{ "f16", options.f16_fmu },
                                     std::pair{ "ap", options.ap_fmu },
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

  if (!preflight_fmu_extraction(options.gps_fmu))
  {
    return EXIT_FAILURE;
  }

  ecos::log::set_logging_level(ecos::log::level::info);

  try
  {
    ecos::simulation_structure ss;
    // String-URI overload for the same cross-drive reason as the trim
    // example. The third argument is the fixed-step algorithm's per-instance
    // step-size hint: the instance steps every ceil(hint/base) base steps
    // with the scaled dt. The plant, the autopilot, the IMU and both I2C
    // parts run at base rate; the GPS and the radar altimeter are decimated
    // to their sample periods (see the file header for why each).
    ss.add_model("f16", options.f16_fmu.string());
    ss.add_model("ap", options.ap_fmu.string());
    ss.add_model("gps", options.gps_fmu.string(), options.gps_period_s);
    ss.add_model("imu", options.imu_fmu.string());
    ss.add_model("baro", options.baro_fmu.string());
    ss.add_model("mag", options.mag_fmu.string());
    ss.add_model("radalt", options.radalt_fmu.string(), 1.0 / options.radalt_rate_hz);

    // ---- truth -> sensors: identical to the trim example ----
    ss.make_connection<double>("f16::out.lat_deg", "gps::latitude_deg");
    ss.make_connection<double>("f16::out.lon_deg", "gps::longitude_deg");
    ss.make_connection<double>("f16::out.alt_m", "gps::altitude_m");
    ss.make_connection<double>("f16::out.v_north_m_s", "gps::v_north_mps");
    ss.make_connection<double>("f16::out.v_east_m_s", "gps::v_east_mps");
    ss.make_connection<double>("f16::out.v_down_m_s", "gps::v_down_mps");
    ss.make_connection<double>("f16::out.p_rad_s", "imu::p_rad_s");
    ss.make_connection<double>("f16::out.q_rad_s", "imu::q_rad_s");
    ss.make_connection<double>("f16::out.r_rad_s", "imu::r_rad_s");
    ss.make_connection<double>("f16::out.specificForce_x_m_s2", "imu::f_x_mps2");
    ss.make_connection<double>("f16::out.specificForce_y_m_s2", "imu::f_y_mps2");
    ss.make_connection<double>("f16::out.specificForce_z_m_s2", "imu::f_z_mps2");
    ss.make_connection<double>("f16::out.alt_m", "baro::h_m");
    // MSL as AGL, bare, for the trim example's documented reason -- and all
    // four cases hold roughly 3 km AGL, well inside the 6 km range.
    ss.make_connection<double>("f16::out.alt_m", "radalt::h_agl_m");
    const std::function<double(const double&)> kelvin2celsius = [](const double& kelvin) { return kelvin - 273.15; };
    ss.make_connection<double>("f16::out.T_K", "mag::temperature_c", kelvin2celsius);

    // ---- the loop: plant state -> autopilot feedback, 1:1 by name ----
    ss.make_connection<double>("f16::out.alt_m", "ap::fb.alt_m");
    ss.make_connection<double>("f16::out.vt_m_s", "ap::fb.vt_m_s");
    ss.make_connection<double>("f16::out.rho_kg_m3", "ap::fb.rho_kg_m3");
    ss.make_connection<double>("f16::out.alpha_deg", "ap::fb.alpha_deg");
    ss.make_connection<double>("f16::out.beta_deg", "ap::fb.beta_deg");
    ss.make_connection<double>("f16::out.roll_rad", "ap::fb.roll_rad");
    ss.make_connection<double>("f16::out.pitch_rad", "ap::fb.pitch_rad");
    ss.make_connection<double>("f16::out.yaw_rad", "ap::fb.yaw_rad");
    ss.make_connection<double>("f16::out.p_rad_s", "ap::fb.p_rad_s");
    ss.make_connection<double>("f16::out.q_rad_s", "ap::fb.q_rad_s");
    ss.make_connection<double>("f16::out.r_rad_s", "ap::fb.r_rad_s");

    // ---- the loop closed: autopilot surfaces -> plant controls ----
    ss.make_connection<double>("ap::ctrl.el_deg", "f16::ctrl.el_deg");
    ss.make_connection<double>("ap::ctrl.ail_deg", "f16::ctrl.ail_deg");
    ss.make_connection<double>("ap::ctrl.rdr_deg", "f16::ctrl.rdr_deg");
    ss.make_connection<double>("ap::ctrl.pwr_pct", "f16::ctrl.pwr_pct");

    // The trim condition and sensor configuration, plus the start-up seeds
    // for the autopilot described in the file header. The seeds only have
    // to put the LQR near its trim operating point for the evaluation Ecos
    // runs during initialisation; from the first step onward every fb.*
    // value comes off the plant.
    ecos::parameter_set trim_point;
    trim_point["f16::lat0_deg"] = options.lat0_deg;
    trim_point["f16::lon0_deg"] = options.lon0_deg;
    trim_point["f16::alt0_ft"] = options.alt0_ft;
    trim_point["f16::vt0_fps"] = options.vt0_fps;
    trim_point["f16::heading0_deg"] = options.heading0_deg;
    trim_point["f16::roll0_deg"] = options.roll0_deg;
    trim_point["imu::sample_rate_hz"] = options.imu_rate_hz;
    trim_point["radalt::sample_rate_hz"] = options.radalt_rate_hz;
    trim_point["gps::dynamic_platform"] = options.dynamic_platform;
    trim_point["gps::cocom_limits_enabled"] = options.cocom_limits;
    trim_point["gps::reacquisition_time_s"] = options.reacquisition_time_s;
    // The receiver's error model, in full: see GpsErrorModel.
    const GpsErrorModel& receiver = options.gps_correlated_errors ? kCorrelatedReceiver : kWhiteReceiver;
    trim_point["gps::seed"] = options.gps_seed;
    trim_point["gps::horizontal_pos_noise_m"] = receiver.horizontal_pos_noise_m;
    trim_point["gps::vertical_pos_noise_m"] = receiver.vertical_pos_noise_m;
    trim_point["gps::speed_noise_mps"] = receiver.speed_noise_mps;
    trim_point["gps::course_noise_deg"] = receiver.course_noise_deg;
    trim_point["gps::horizontal_pos_correlated_m"] = receiver.horizontal_pos_correlated_m;
    trim_point["gps::vertical_pos_correlated_m"] = receiver.vertical_pos_correlated_m;
    trim_point["gps::position_correlation_time_s"] = receiver.position_correlation_time_s;
    trim_point["gps::speed_correlated_mps"] = receiver.speed_correlated_mps;
    trim_point["gps::course_correlated_deg"] = receiver.course_correlated_deg;
    trim_point["gps::velocity_correlation_time_s"] = receiver.velocity_correlation_time_s;
    trim_point["gps::accuracy_scale"] = receiver.accuracy_scale;
    trim_point["ap::fb.alt_m"] = options.alt0_ft / kFtPerM;
    trim_point["ap::fb.vt_m_s"] = options.vt0_fps / kFtPerM;
    trim_point["ap::fb.rho_kg_m3"] = 0.9046;  // ISA at 3 052 m; a seed, not truth
    trim_point["ap::fb.alpha_deg"] = 2.656;   // Aetherion's trim alpha (case 11)
    trim_point["ap::fb.beta_deg"] = 0.0;
    trim_point["ap::fb.roll_rad"] = options.roll0_deg * kPi / 180.0;
    trim_point["ap::fb.pitch_rad"] = 2.656 * kPi / 180.0;  // pitch = alpha in level trim
    trim_point["ap::fb.yaw_rad"] = options.heading0_deg * kPi / 180.0;
    trim_point["ap::fb.p_rad_s"] = 0.0;
    trim_point["ap::fb.q_rad_s"] = 0.0;
    trim_point["ap::fb.r_rad_s"] = 0.0;
    trim_point["ap::cmd.altCmd_ft"] = options.alt0_ft;
    // The standalone's trim-KEAS value for this exact condition
    // (F16AltitudeChangeCmds); replaced after init() by the value computed
    // from the plant's own density and airspeed.
    trim_point["ap::cmd.keasCmd_kt"] = 287.8088596053291;
    trim_point["ap::cmd.baseChiCmd_deg"] = options.heading0_deg;
    trim_point["ap::cmd.latOffset_ft"] = 0.0;
    ss.add_parameter_set("trimPoint", trim_point);

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(options.step_s));
    sim->init("trimPoint");

    auto* altitude = sim->get_real_property("f16::out.alt_m");
    auto* airspeed = sim->get_real_property("f16::out.vt_m_s");
    auto* density = sim->get_real_property("f16::out.rho_kg_m3");
    auto* latitude = sim->get_real_property("f16::out.lat_deg");
    auto* longitude = sim->get_real_property("f16::out.lon_deg");
    auto* yaw = sim->get_real_property("f16::out.yaw_rad");
    auto* pitch = sim->get_real_property("f16::out.pitch_rad");
    auto* roll = sim->get_real_property("f16::out.roll_rad");
    auto* mag_bx = sim->get_real_property("mag::b_x_ut");
    auto* mag_by = sim->get_real_property("mag::b_y_ut");
    auto* mag_bz = sim->get_real_property("mag::b_z_ut");
    auto* cmd_alt = sim->get_real_property("ap::cmd.altCmd_ft");
    auto* cmd_keas = sim->get_real_property("ap::cmd.keasCmd_kt");
    auto* cmd_chi = sim->get_real_property("ap::cmd.baseChiCmd_deg");
    auto* cmd_lat = sim->get_real_property("ap::cmd.latOffset_ft");

    // The hold-KEAS command, from the plant's own trim -- the standalone's
    // formula with the standalone's constants, so the two drivers command
    // the same equivalent airspeed to the same DML.
    const double keas_trim_kt = (airspeed->get_value() / kKtMps) * std::sqrt(density->get_value() / kRhoSlKgM3);

    // The courseline reference for the 13.4 lateral-deviation feedback: the
    // aircraft's *actual* initial geodetic position, as in the standalone.
    const double lat0_rad = latitude->get_value() * kPi / 180.0;
    const double lon0_rad = longitude->get_value() * kPi / 180.0;
    const double course_psi_rad = options.heading0_deg * kPi / 180.0;

    // Lateral deviation from the original courseline, +right, in feet --
    // the standalone's flat-earth formula verbatim.
    const auto lateral_deviation_ft = [&]() {
      const double dp_north_m = (latitude->get_value() * kPi / 180.0 - lat0_rad) * kLatDevEarthRadiusM;
      const double dp_east_m =
          (longitude->get_value() * kPi / 180.0 - lon0_rad) * kLatDevEarthRadiusM * std::cos(lat0_rad);
      return (-dp_north_m * std::sin(course_psi_rad) + dp_east_m * std::cos(course_psi_rad)) * kFtPerM;
    };

    // The command schedule. Written after every step (and once before the
    // first), so the autopilot's step starting at time t sees the commands
    // the standalone's controller sees when its clock reads t.
    const auto write_commands = [&](double t) {
      cmd_alt->set_value(t >= ap_case.alt_step_time_s ? ap_case.alt_cmd_ft : options.alt0_ft);
      const double keas_cmd = std::isnan(ap_case.keas_cmd_kt) ?
                                  keas_trim_kt :
                                  (t >= ap_case.keas_step_time_s ? ap_case.keas_cmd_kt : keas_trim_kt);
      cmd_keas->set_value(keas_cmd);
      cmd_chi->set_value(t >= ap_case.chi_step_time_s ? ap_case.chi_cmd_deg : options.heading0_deg);
      double lat_offset_ft = 0.0;
      if (ap_case.lat_step_ft != 0.0)
      {
        lat_offset_ft = lateral_deviation_ft() - (t >= ap_case.lat_step_time_s ? ap_case.lat_step_ft : 0.0);
      }
      cmd_lat->set_value(lat_offset_ft);
    };
    write_commands(0.0);

    TruthLogger truth_log(options.csv_path,
                          *sim,
                          { "f16::out.alt_m",
                            "f16::out.lat_deg",
                            "f16::out.lon_deg",
                            "f16::out.v_north_m_s",
                            "f16::out.v_east_m_s",
                            "f16::out.v_down_m_s",
                            "f16::out.yaw_rad",
                            "f16::out.pitch_rad",
                            "f16::out.roll_rad",
                            "f16::out.p_rad_s",
                            "f16::out.q_rad_s",
                            "f16::out.r_rad_s",
                            "f16::out.vt_m_s",
                            "f16::out.rho_kg_m3",
                            "f16::out.alpha_deg",
                            "f16::out.beta_deg",
                            "f16::out.mach",
                            "f16::out.qbar_Pa",
                            "f16::out.thrust_N",
                            "f16::out.mass_kg",
                            // The loop itself: what the autopilot commanded the
                            // surfaces to, and what this host commanded the
                            // autopilot to -- so a response plot can carry its
                            // own cause.
                            "ap::ctrl.el_deg",
                            "ap::ctrl.ail_deg",
                            "ap::ctrl.rdr_deg",
                            "ap::ctrl.pwr_pct",
                            "ap::cmd.altCmd_ft",
                            "ap::cmd.keasCmd_kt",
                            "ap::cmd.baseChiCmd_deg",
                            "ap::cmd.latOffset_ft",
                            // What the sensor FMUs were actually given, as in
                            // the trim example.
                            "imu::f_x_mps2",
                            "imu::f_y_mps2",
                            "imu::f_z_mps2",
                            "mag::b_x_ut",
                            "mag::b_y_ut",
                            "mag::b_z_ut",
                            "radalt::h_agl_m" });
    write_run_config(options.csv_path, options, ap_case, keas_trim_kt);

    const auto current_keas_kt = [&]() {
      return (airspeed->get_value() / kKtMps) * std::sqrt(density->get_value() / kRhoSlKgM3);
    };

    std::cout << "[cosim] f16:    " << options.f16_fmu.string() << "\n"
              << "[cosim] ap:     " << options.ap_fmu.string() << "\n"
              << "[cosim] gps:    " << options.gps_fmu.string() << "\n"
              << "[cosim] imu:    " << options.imu_fmu.string() << "\n"
              << "[cosim] baro:   " << options.baro_fmu.string() << "\n"
              << "[cosim] mag:    " << options.mag_fmu.string() << "\n"
              << "[cosim] radalt: " << options.radalt_fmu.string() << "\n"
              << "[cosim] check-case " << options.check_case << ": " << ap_case.summary << "\n"
              << "[cosim] loop: " << options.step_s << " s base step (" << 1.0 / options.step_s
              << " Hz closed loop, one-step transport delay); GPS every " << options.gps_period_s << " s, radalt every "
              << 1.0 / options.radalt_rate_hz << " s, " << options.imu_rate_hz
              << " Hz IMU; BMP390 and MMC5983MA at the rates the flight computer programs\n"
              << "[cosim] plant: trimmed at " << std::fixed << std::setprecision(6) << options.lat0_deg << " deg N / "
              << options.lon0_deg << " deg E, " << std::defaultfloat << options.alt0_ft << " ft, " << options.vt0_fps
              << " ft/s, heading " << options.heading0_deg << " deg\n"
              << "[cosim] autopilot: holding trim KEAS " << keas_trim_kt << " kt (from the plant's rho and vt)\n"
              << "[cosim] stop " << options.stop_s << " s (the case's longest published reference window)\n";

    const auto wall_start = std::chrono::steady_clock::now();
    long print_counter = 0;
    const long print_period = std::lround(10.0 / options.step_s);  // one status line per 10 s of sim time

    truth_log.write_row(sim->iterations(), sim->time());  // the state at t = 0
    while (sim->time() < options.stop_s)
    {
      sim->step();

      // Post-step writes, all carrying the same one-communication-step
      // transport delay: the dipole field for the magnetometer (as in the
      // trim example), and the command schedule for the autopilot.
      const FieldNed field_ned =
          GeomagneticDipole::field_ned(latitude->get_value(), longitude->get_value(), altitude->get_value());
      const FieldBody field_body =
          GeomagneticDipole::to_body(field_ned, yaw->get_value(), pitch->get_value(), roll->get_value());
      mag_bx->set_value(field_body.x_ut);
      mag_by->set_value(field_body.y_ut);
      mag_bz->set_value(field_body.z_ut);
      write_commands(sim->time());

      truth_log.write_row(sim->iterations(), sim->time());

      if (++print_counter % print_period == 0)
      {
        std::cout << "[cosim] t=" << sim->time() << " s  alt=" << altitude->get_value() << " m ("
                  << altitude->get_value() * kFtPerM << " ft)  keas=" << current_keas_kt()
                  << " kt  hdg=" << yaw->get_value() * 180.0 / kPi << " deg";
        if (ap_case.lat_step_ft != 0.0)
        {
          std::cout << "  latdev=" << lateral_deviation_ft() << " ft";
        }
        std::cout << "\n";
      }
      if (options.realtime_factor > 0.0)
      {
        const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(sim->time() / options.realtime_factor));
        std::this_thread::sleep_until(target);
      }
    }

    // The BMP390 rating diagnostic, as in the trim example -- informational
    // here (all four cases hold ~3 km, comfortably inside the envelope), and
    // zero is that result stated rather than assumed.
    const int baro_conversions = sim->get_int_property("baro::conversions")->get_value();
    const int baro_out_of_rating = sim->get_int_property("baro::conversions_out_of_rating")->get_value();

    // The response summary: every commanded quantity's final value beside
    // its command, whichever one this case stepped. The verifier judges
    // them; this prints them.
    const double final_alt_ft = altitude->get_value() * kFtPerM;
    const double final_keas_kt = current_keas_kt();
    const double final_hdg_deg = yaw->get_value() * 180.0 / kPi;
    const double final_latdev_ft = lateral_deviation_ft();

    sim->terminate();

    std::cout << "[cosim] done: " << sim->iterations() << " steps of " << options.step_s << " s\n"
              << "[cosim] the BMP390 latched " << baro_conversions << " conversions, " << baro_out_of_rating
              << " outside its rated envelope (300-1250 hPa, -40..+85 degC)\n"
              << "[cosim] response at cut-off (commanded value in brackets):\n"
              << "[cosim]   altitude  " << final_alt_ft << " ft  [" << ap_case.alt_cmd_ft << " ft]\n"
              << "[cosim]   KEAS      " << final_keas_kt << " kt  ["
              << (std::isnan(ap_case.keas_cmd_kt) ? keas_trim_kt : ap_case.keas_cmd_kt) << " kt]\n"
              << "[cosim]   course    " << final_hdg_deg << " deg  [" << ap_case.chi_cmd_deg << " deg]\n"
              << "[cosim]   lat. dev  " << final_latdev_ft << " ft  [" << ap_case.lat_step_ft << " ft right]\n"
              << "[cosim] plant truth written to " << options.csv_path.string() << "\n";
  }
  catch (const std::exception& ex)
  {
    std::cerr << "error: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
