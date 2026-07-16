// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file flight_computer_main.cpp
/// @brief Host stand-in for the STM32 flight computer's GPS ingest path.
///
/// Consumes the GPS FMU's UBX-NAV-PVT UDP stream with the exact
/// GpsDriver/UbxParser stack that runs on the STM32H743 target -- only the
/// byte transport differs: a UDP socket here, a UART RX line (or Renode's
/// emulated UART fed by the same FMU, see tests/swil) on hardware. Every
/// decoded fix is printed and appended to a CSV so plot_results.py can
/// compare what the flight software *believes* against the rocket truth the
/// co-simulation logged.
///
/// Exits on its own once the UBX stream goes quiet (the co-simulation
/// finished) or a hard wall-clock cap is reached, so a scripted run never
/// hangs.

#include "Hemerion/gps/gpsDriver.hpp"
#include "udpReceiver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace
{

using hemerion::examples::rocket_gps_ecos::UdpReceiver;
using hemerion::sensors::gps::GpsDriver;
using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::GpsProtocol;

struct Options
{
  std::uint16_t port = 5762;  // GPS FMU default UDP destination
  std::filesystem::path csv_path = "results/gps_fixes.csv";
  double fix_period_s = 0.1;  // co-sim communication step; maps fix index -> sim time for the CSV
  int print_every = 50;       // console line every N fixes (50 = every 5 s of sim time at 10 Hz)
  long quiet_ms = 3000;       // exit after this long with no datagrams, once at least one fix arrived
  long max_wall_s = 900;      // hard wall-clock cap
};

void print_usage()
{
  std::cout << "usage: gps_flight_computer [--port <udp port>] [--csv <file>] [--fix-period <s>]\n"
               "                           [--print-every <n>] [--quiet-ms <ms>] [--max-wall-s <s>]\n";
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
    if (arg == "--port" && (value = next()))
    {
      options.port = static_cast<std::uint16_t>(std::stoi(value));
    }
    else if (arg == "--csv" && (value = next()))
    {
      options.csv_path = value;
    }
    else if (arg == "--fix-period" && (value = next()))
    {
      options.fix_period_s = std::stod(value);
    }
    else if (arg == "--print-every" && (value = next()))
    {
      options.print_every = std::stoi(value);
    }
    else if (arg == "--quiet-ms" && (value = next()))
    {
      options.quiet_ms = std::stol(value);
    }
    else if (arg == "--max-wall-s" && (value = next()))
    {
      options.max_wall_s = std::stol(value);
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

void print_fix(long index, double sim_time_s, const GpsFix& fix)
{
  std::printf("[fc] fix %5ld  t=%7.1f s  lat=%11.7f  lon=%12.7f  alt=%9.1f m  vel=%7.1f m/s  crs=%5.1f deg  sats=%u\n",
              index,
              sim_time_s,
              fix.latitude_deg,
              fix.longitude_deg,
              static_cast<double>(fix.altitude_m),
              static_cast<double>(fix.ground_speed_mps),
              static_cast<double>(fix.course_deg),
              static_cast<unsigned>(fix.num_satellites));
}

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  if (!parse_args(argc, argv, options))
  {
    return EXIT_FAILURE;
  }

  auto receiver = UdpReceiver::create(options.port);
  if (!receiver.has_value())
  {
    std::cerr << "error: could not bind UDP port " << options.port << " (already in use?)\n";
    return EXIT_FAILURE;
  }

  if (options.csv_path.has_parent_path())
  {
    std::filesystem::create_directories(options.csv_path.parent_path());
  }
  std::ofstream csv(options.csv_path);
  if (!csv)
  {
    std::cerr << "error: cannot open " << options.csv_path.string() << " for writing\n";
    return EXIT_FAILURE;
  }
  csv << "fix_index,sim_time_s,latitude_deg,longitude_deg,altitude_m,ground_speed_mps,course_deg,"
         "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type\n";

  std::cout << "[fc] listening for UBX-NAV-PVT on UDP port " << options.port << " (GpsDriver, protocol=UBX)\n";

  GpsDriver driver(GpsProtocol::kUbx);
  std::array<std::uint8_t, 2048> datagram{};
  GpsFix fix;
  long fix_count = 0;
  long checksum_errors = 0;
  double max_altitude_m = 0.0;
  float max_speed_mps = 0.0F;

  const auto wall_start = std::chrono::steady_clock::now();
  auto last_datagram = wall_start;
  bool any_fix = false;

  while (true)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - wall_start > std::chrono::seconds(options.max_wall_s))
    {
      std::cout << "[fc] wall-clock cap reached\n";
      break;
    }
    if (any_fix && now - last_datagram > std::chrono::milliseconds(options.quiet_ms))
    {
      std::cout << "[fc] UBX stream quiet for " << options.quiet_ms << " ms -- co-simulation finished\n";
      break;
    }

    const auto received = receiver->receive(datagram.data(), datagram.size(), std::chrono::milliseconds(250));
    if (!received.has_value())
    {
      continue;
    }
    last_datagram = std::chrono::steady_clock::now();

    // Same per-byte feed the firmware's GPS task performs on UART RX data.
    for (std::size_t i = 0; i < *received; ++i)
    {
      const auto local_clock_us = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(last_datagram - wall_start).count());
      const GpsParseError result = driver.feed(datagram[i], local_clock_us, fix);
      if (result == GpsParseError::kNone)
      {
        ++fix_count;
        any_fix = true;
        // One NAV-PVT frame is emitted per communication step, so the fix
        // index maps directly to co-simulation time.
        const double sim_time_s = static_cast<double>(fix_count) * options.fix_period_s;
        max_altitude_m = std::max(max_altitude_m, static_cast<double>(fix.altitude_m));
        max_speed_mps = std::max(max_speed_mps, fix.ground_speed_mps);
        csv << fix_count << ',' << sim_time_s << ',' << fix.latitude_deg << ',' << fix.longitude_deg << ','
            << fix.altitude_m << ',' << fix.ground_speed_mps << ',' << fix.course_deg << ','
            << fix.horizontal_accuracy_m << ',' << fix.vertical_accuracy_m << ','
            << static_cast<unsigned>(fix.num_satellites) << ',' << static_cast<unsigned>(fix.fix_type) << '\n';
        if (fix_count == 1 || fix_count % options.print_every == 0)
        {
          print_fix(fix_count, sim_time_s, fix);
        }
      }
      else if (result == GpsParseError::kChecksumMismatch)
      {
        ++checksum_errors;
      }
    }
  }

  std::cout << "[fc] summary: " << fix_count << " fixes decoded, " << checksum_errors << " checksum errors\n"
            << "[fc] max altitude " << max_altitude_m << " m, max ground speed " << max_speed_mps << " m/s\n"
            << "[fc] fixes written to " << options.csv_path.string() << "\n";
  return (fix_count > 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
