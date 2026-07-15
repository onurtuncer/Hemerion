// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file cosim_host_main.cpp
/// @brief Ecos co-simulation host coupling Aetherion's TwoStageRocket.fmu to
/// Hemerion's GPS UBX hardware-simulator FMU.
///
/// Topology (see examples/rocket_gps_ecos/README.md):
///
///   TwoStageRocket.fmu ──(FMI variables, Ecos connections)──> hemerion_gps_fmu.fmu
///        truth: lat/lon/alt, NED velocity                       noise + UBX-NAV-PVT
///                                                                     │ UDP 127.0.0.1:5762
///                                                                     v
///                                                          gps_flight_computer
///                                                    (GpsDriver/UbxParser, the same
///                                                     stack that runs on the STM32H743)
///
/// The rocket reports geodetic latitude/longitude in radians and NED velocity
/// in m/s; the GPS FMU takes degrees and NED velocity directly, so the two
/// rad->deg conversions are attached as Ecos connection modifiers and every
/// other connection is 1:1. The GPS FMU has no FMI outputs -- its effect is
/// the UBX-NAV-PVT UDP side channel consumed by gps_flight_computer.
///
/// Rocket truth is logged through Ecos' csv_writer so the flight computer's
/// decoded fixes can be compared against it (plot_results.py).

#include "ecos/algorithm/fixed_step_algorithm.hpp"
#include "ecos/listeners/csv_writer.hpp"
#include "ecos/logger/logger.hpp"
#include "ecos/structure/simulation_structure.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <thread>

namespace {

// Compile-time defaults injected by CMakeLists.txt; both can be overridden on
// the command line, so an empty default (FMU not found at configure time) is
// not fatal until the run actually needs it.
#ifndef HEMERION_ROCKET_FMU_PATH
#define HEMERION_ROCKET_FMU_PATH ""
#endif
#ifndef HEMERION_GPS_FMU_PATH
#define HEMERION_GPS_FMU_PATH ""
#endif

struct Options {
  std::filesystem::path rocket_fmu = HEMERION_ROCKET_FMU_PATH;
  std::filesystem::path gps_fmu = HEMERION_GPS_FMU_PATH;
  std::filesystem::path csv_path = "results/rocket_truth.csv";
  // The TwoStageRocket FMU (NASA TM-2015-218675 Scenario 17) reaches apogee around t = 232 s and holds its
  // state constant from there on, so 240 s covers the whole interesting flight.
  double stop_s = 240.0;
  double step_s = 0.1;       // communication step == GPS output period (10 Hz)
  double realtime_factor = 0.0;  // 0 = run as fast as possible
};

void print_usage() {
  std::cout << "usage: rocket_gps_cosim [--rocket <TwoStageRocket.fmu>] [--gps <hemerion_gps_fmu.fmu>]\n"
               "                        [--stop <s>] [--step <s>] [--csv <file>] [--rtf <x>]\n"
               "\n"
               "  --rocket  path to Aetherion's TwoStageRocket.fmu (default: configure-time location)\n"
               "  --gps     path to the packaged hemerion_gps_fmu.fmu (default: build-tree artifact)\n"
               "  --stop    simulation stop time [s] (default 240; the rocket holds state after apogee ~232 s)\n"
               "  --step    communication step size [s]; also the GPS fix period (default 0.1)\n"
               "  --csv     rocket truth output CSV (default results/rocket_truth.csv)\n"
               "  --rtf     real-time factor pacing, e.g. 1 = wall-clock speed; 0 = unpaced (default)\n";
}

bool parse_args(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return false;
    }
    const char* value = nullptr;
    if (arg == "--rocket" && (value = next())) {
      options.rocket_fmu = value;
    } else if (arg == "--gps" && (value = next())) {
      options.gps_fmu = value;
    } else if (arg == "--stop" && (value = next())) {
      options.stop_s = std::stod(value);
    } else if (arg == "--step" && (value = next())) {
      options.step_s = std::stod(value);
    } else if (arg == "--csv" && (value = next())) {
      options.csv_path = value;
    } else if (arg == "--rtf" && (value = next())) {
      options.realtime_factor = std::stod(value);
    } else {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      print_usage();
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse_args(argc, argv, options)) {
    return EXIT_FAILURE;
  }

  for (const auto& [label, path] : {std::pair{"rocket", options.rocket_fmu}, std::pair{"gps", options.gps_fmu}}) {
    if (path.empty() || !std::filesystem::exists(path)) {
      std::cerr << "error: " << label << " FMU not found at '" << path.string() << "' -- pass --" << label << "\n";
      return EXIT_FAILURE;
    }
  }

  ecos::log::set_logging_level(ecos::log::level::info);

  try {
    ecos::simulation_structure ss;
    // Deliberately the string-URI overload: ecos' std::filesystem::path overload runs the path through
    // std::filesystem::relative(), which yields an empty path on Windows when the FMU sits on a different
    // drive than the working directory. The file resolver handles absolute path strings as-is.
    ss.add_model("rocket", options.rocket_fmu.string());
    ss.add_model("gps", options.gps_fmu.string());

    // The rocket reports geodetic position in radians; the GPS FMU takes degrees.
    const std::function<double(const double&)> rad2deg = [](const double& rad) {
      return rad * (180.0 / std::numbers::pi);
    };
    ss.make_connection<double>("rocket::out.lat_rad", "gps::latitude_deg", rad2deg);
    ss.make_connection<double>("rocket::out.lon_rad", "gps::longitude_deg", rad2deg);
    ss.make_connection<double>("rocket::out.alt_m", "gps::altitude_m");
    ss.make_connection<double>("rocket::out.v_north_m_s", "gps::v_north_mps");
    ss.make_connection<double>("rocket::out.v_east_m_s", "gps::v_east_mps");
    ss.make_connection<double>("rocket::out.v_down_m_s", "gps::v_down_mps");

    // NASA Wallops Flight Facility launch range, firing due east over the
    // Atlantic. pitch0/azimuth0 follow the FMU's NASA TM-2015-218675
    // Scenario 17 defaults. solver.max_step_s is left at its default (0 =
    // one implicit Radau step per communication step); the integrator is
    // L-stable, and capping sub-steps at 0.01 s costs ~10x the wall time for
    // no visible change in the 10 Hz GPS-grade outputs.
    ecos::parameter_set launch_site;
    launch_site["rocket::lat0_deg"] = 37.8338;
    launch_site["rocket::lon0_deg"] = -75.4879;
    launch_site["rocket::alt0_m"] = 3.0;
    launch_site["rocket::azimuth0_deg"] = 90.0;
    launch_site["rocket::pitch0_deg"] = 55.22;
    ss.add_parameter_set("launchSite", launch_site);

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(options.step_s));

    auto csv = std::make_unique<ecos::csv_writer>(options.csv_path);
    ecos::csv_config& csv_config = csv->config();
    for (const char* variable : {"rocket::out.alt_m", "rocket::out.lat_rad", "rocket::out.lon_rad",
                                 "rocket::out.v_north_m_s", "rocket::out.v_east_m_s", "rocket::out.v_down_m_s",
                                 "rocket::out.mach", "rocket::out.qbar_Pa", "rocket::out.thrust_N",
                                 "rocket::out.mass_kg", "rocket::out.staged"}) {
      csv_config.register_variable(variable);
    }
    sim->add_listener("csv_writer", std::move(csv));

    sim->init("launchSite");

    auto* altitude = sim->get_real_property("rocket::out.alt_m");
    auto* mach = sim->get_real_property("rocket::out.mach");
    auto* mass = sim->get_real_property("rocket::out.mass_kg");
    auto* staged = sim->get_bool_property("rocket::out.staged");

    double apogee_m = 0.0;
    double apogee_time_s = 0.0;
    double staging_time_s = -1.0;
    const auto wall_start = std::chrono::steady_clock::now();

    std::cout << "[cosim] rocket: " << options.rocket_fmu.string() << "\n"
              << "[cosim] gps:    " << options.gps_fmu.string() << "\n"
              << "[cosim] step " << options.step_s << " s (" << 1.0 / options.step_s << " Hz GPS), stop "
              << options.stop_s << " s\n";

    long print_counter = 0;
    const long print_period = std::lround(10.0 / options.step_s);  // one status line per 10 s of sim time
    while (sim->time() < options.stop_s) {
      sim->step();

      const double altitude_m = altitude->get_value();
      if (altitude_m > apogee_m) {
        apogee_m = altitude_m;
        apogee_time_s = sim->time();
      }
      if (staging_time_s < 0.0 && staged->get_value()) {
        staging_time_s = sim->time();
        std::cout << "[cosim] t=" << sim->time() << " s  stage 1 separated\n";
      }
      if (++print_counter % print_period == 0) {
        std::cout << "[cosim] t=" << sim->time() << " s  alt=" << altitude_m << " m  mach=" << mach->get_value()
                  << "  mass=" << mass->get_value() << " kg\n";
      }
      if (options.realtime_factor > 0.0) {
        const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(sim->time() / options.realtime_factor));
        std::this_thread::sleep_until(target);
      }
    }

    sim->terminate();

    std::cout << "[cosim] done: " << sim->iterations() << " steps, " << sim->iterations()
              << " UBX-NAV-PVT frames emitted\n"
              << "[cosim] apogee " << apogee_m << " m at t=" << apogee_time_s << " s";
    if (staging_time_s >= 0.0) {
      std::cout << ", staging at t=" << staging_time_s << " s";
    }
    std::cout << "\n[cosim] rocket truth written to " << options.csv_path.string() << "\n";
  } catch (const std::exception& ex) {
    std::cerr << "error: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
