// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file cosim_host_main.cpp
/// @brief Ecos co-simulation host coupling Aetherion's TwoStageRocket.fmu to
/// Hemerion's GPS and IMU hardware-simulator FMUs.
///
/// Topology (see examples/rocket_gps_ecos/README.md):
///
///   TwoStageRocket.fmu ──(FMI variables, Ecos connections)──> hemerion_gps_fmu.fmu
///        truth: lat/lon/alt, NED velocity        │              noise + UBX-NAV-PVT
///               body rates, forces, mass         │                    │ UDP 127.0.0.1:5762
///                                                │                    v
///                                                └──────────> hemerion_imu_fmu.fmu
///                                                               noise + raw-count frames
///                                                                     │ UDP 127.0.0.1:5763
///                                                                     v
///                                                          gps_flight_computer
///                                                    (GpsDriver/UbxParser + ImuPacketParser/
///                                                     convert_raw_to_si, the same stacks
///                                                     that run on the STM32H743)
///
/// The rocket reports geodetic latitude/longitude in radians and NED velocity
/// in m/s; the GPS FMU takes degrees and NED velocity directly, so the two
/// rad->deg conversions are attached as Ecos connection modifiers and every
/// other connection is 1:1. The IMU FMU takes true body-frame specific force,
/// which the rocket does not expose directly; the host computes
/// f = (thrust + F_aero) / m from the rocket's outputs after every step and
/// writes it to the IMU's inputs through Ecos properties (same one-step
/// transport delay as an Ecos connection). Body rates p/q/r connect 1:1.
/// Neither sensor FMU has FMI outputs -- their effect is the UDP side
/// channels consumed by gps_flight_computer.
///
/// Rocket truth is logged through Ecos' csv_writer so the flight computer's
/// decoded fixes and IMU samples can be compared against it
/// (plot_results.py).

#include "ecos/algorithm/fixed_step_algorithm.hpp"
#include "ecos/listeners/csv_writer.hpp"
#include "ecos/logger/logger.hpp"
#include "ecos/structure/simulation_structure.hpp"

#include <algorithm>
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

namespace
{

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

struct Options
{
  std::filesystem::path rocket_fmu = HEMERION_ROCKET_FMU_PATH;
  std::filesystem::path gps_fmu = HEMERION_GPS_FMU_PATH;
  std::filesystem::path imu_fmu = HEMERION_IMU_FMU_PATH;
  std::filesystem::path csv_path = "results/rocket_truth.csv";
  // The TwoStageRocket FMU (NASA TM-2015-218675 Scenario 17) reaches apogee around t = 232 s and holds its
  // state constant from there on, so 240 s covers the whole interesting flight.
  double stop_s = 240.0;
  double step_s = 0.1;           // communication step == GPS output period (10 Hz)
  double imu_rate_hz = 100.0;    // IMU output data rate; the IMU FMU emits step * rate frames per step
  double realtime_factor = 0.0;  // 0 = run as fast as possible
};

void print_usage()
{
  std::cout << "usage: rocket_gps_cosim [--rocket <TwoStageRocket.fmu>] [--gps <hemerion_gps_fmu.fmu>]\n"
               "                        [--imu <hemerion_imu_fmu.fmu>] [--imu-rate <hz>]\n"
               "                        [--stop <s>] [--step <s>] [--csv <file>] [--rtf <x>]\n"
               "\n"
               "  --rocket   path to Aetherion's TwoStageRocket.fmu (default: configure-time location)\n"
               "  --gps      path to the packaged hemerion_gps_fmu.fmu (default: build-tree artifact)\n"
               "  --imu      path to the packaged hemerion_imu_fmu.fmu (default: build-tree artifact)\n"
               "  --imu-rate IMU output data rate [Hz] (default 100)\n"
               "  --stop     simulation stop time [s] (default 240; the rocket holds state after apogee ~232 s)\n"
               "  --step     communication step size [s]; also the GPS fix period (default 0.1)\n"
               "  --csv      rocket truth output CSV (default results/rocket_truth.csv)\n"
               "  --rtf      real-time factor pacing, e.g. 1 = wall-clock speed; 0 = unpaced (default)\n";
}

bool parse_args(int argc, char** argv, Options& options)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    if (arg == "--help" || arg == "-h")
    {
      print_usage();
      return false;
    }
    const char* value = nullptr;
    if (arg == "--rocket" && (value = next()))
    {
      options.rocket_fmu = value;
    }
    else if (arg == "--gps" && (value = next()))
    {
      options.gps_fmu = value;
    }
    else if (arg == "--imu" && (value = next()))
    {
      options.imu_fmu = value;
    }
    else if (arg == "--imu-rate" && (value = next()))
    {
      options.imu_rate_hz = std::stod(value);
    }
    else if (arg == "--stop" && (value = next()))
    {
      options.stop_s = std::stod(value);
    }
    else if (arg == "--step" && (value = next()))
    {
      options.step_s = std::stod(value);
    }
    else if (arg == "--csv" && (value = next()))
    {
      options.csv_path = value;
    }
    else if (arg == "--rtf" && (value = next()))
    {
      options.realtime_factor = std::stod(value);
    }
    else
    {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      print_usage();
      return false;
    }
  }
  return true;
}

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
                                     std::pair{ "imu", options.imu_fmu } })
  {
    if (path.empty() || !std::filesystem::exists(path))
    {
      std::cerr << "error: " << label << " FMU not found at '" << path.string() << "' -- pass --" << label << "\n";
      return EXIT_FAILURE;
    }
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

    // Body angular rates feed the IMU's gyroscope triad 1:1. Specific force
    // has no direct rocket output -- it is computed from thrust/aero/mass in
    // the stepping loop below, because an Ecos connection modifier sees only
    // its single source variable.
    ss.make_connection<double>("rocket::out.p_rad_s", "imu::p_rad_s");
    ss.make_connection<double>("rocket::out.q_rad_s", "imu::q_rad_s");
    ss.make_connection<double>("rocket::out.r_rad_s", "imu::r_rad_s");

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
    // Not launch-site geometry, but sim->init() applies exactly one named
    // parameter set, so the IMU's output data rate rides along here.
    launch_site["imu::sample_rate_hz"] = options.imu_rate_hz;
    ss.add_parameter_set("launchSite", launch_site);

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(options.step_s));

    auto csv = std::make_unique<ecos::csv_writer>(options.csv_path);
    ecos::csv_config& csv_config = csv->config();
    for (const char* variable : { "rocket::out.alt_m",
                                  "rocket::out.lat_rad",
                                  "rocket::out.lon_rad",
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
                                  "rocket::out.staged",
                                  // What the IMU FMU actually receives: the host-computed
                                  // body-frame specific force (one step behind rocket truth,
                                  // like every other connection).
                                  "imu::f_x_mps2",
                                  "imu::f_y_mps2",
                                  "imu::f_z_mps2" })
    {
      csv_config.register_variable(variable);
    }
    sim->add_listener("csv_writer", std::move(csv));

    sim->init("launchSite");

    auto* altitude = sim->get_real_property("rocket::out.alt_m");
    auto* mach = sim->get_real_property("rocket::out.mach");
    auto* mass = sim->get_real_property("rocket::out.mass_kg");
    auto* staged = sim->get_bool_property("rocket::out.staged");

    // Specific-force plumbing: what an ideal accelerometer reads is the sum
    // of the non-gravitational forces over mass, f = (F_thrust + F_aero)/m,
    // in body axes (thrust acts along body +X on this vehicle). The rocket
    // exposes the ingredients but not f itself, and an Ecos connection
    // modifier cannot combine three source variables, so the host computes f
    // after every step and writes the IMU inputs directly -- giving the same
    // one-communication-step transport delay a connection would.
    auto* thrust = sim->get_real_property("rocket::out.thrust_N");
    auto* aero_fx = sim->get_real_property("rocket::out.aero_Fx_N");
    auto* aero_fy = sim->get_real_property("rocket::out.aero_Fy_N");
    auto* aero_fz = sim->get_real_property("rocket::out.aero_Fz_N");
    auto* imu_fx = sim->get_real_property("imu::f_x_mps2");
    auto* imu_fy = sim->get_real_property("imu::f_y_mps2");
    auto* imu_fz = sim->get_real_property("imu::f_z_mps2");

    double apogee_m = 0.0;
    double apogee_time_s = 0.0;
    double staging_time_s = -1.0;
    const auto wall_start = std::chrono::steady_clock::now();

    std::cout << "[cosim] rocket: " << options.rocket_fmu.string() << "\n"
              << "[cosim] gps:    " << options.gps_fmu.string() << "\n"
              << "[cosim] imu:    " << options.imu_fmu.string() << "\n"
              << "[cosim] step " << options.step_s << " s (" << 1.0 / options.step_s << " Hz GPS, "
              << options.imu_rate_hz << " Hz IMU), stop " << options.stop_s << " s\n";

    long print_counter = 0;
    const long print_period = std::lround(10.0 / options.step_s);  // one status line per 10 s of sim time
    while (sim->time() < options.stop_s)
    {
      sim->step();

      const double mass_kg = mass->get_value();
      if (mass_kg > 0.0)
      {
        imu_fx->set_value((thrust->get_value() + aero_fx->get_value()) / mass_kg);
        imu_fy->set_value(aero_fy->get_value() / mass_kg);
        imu_fz->set_value(aero_fz->get_value() / mass_kg);
      }

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
    std::cout << "[cosim] done: " << sim->iterations() << " steps, " << sim->iterations()
              << " UBX-NAV-PVT frames and " << sim->iterations() * std::max(1L, imu_frames_per_step)
              << " IMU frames emitted\n"
              << "[cosim] apogee " << apogee_m << " m at t=" << apogee_time_s << " s";
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
