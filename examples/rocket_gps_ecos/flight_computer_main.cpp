// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file flight_computer_main.cpp
/// @brief Host stand-in for the STM32 flight computer's sensor ingest paths.
///
/// Consumes the GPS FMU's UBX-NAV-PVT UDP stream with the exact
/// GpsDriver/UbxParser stack that runs on the STM32H743 target, and the IMU
/// FMU's raw-sample UDP stream with the exact ImuPacketParser +
/// convert_raw_to_si() stack -- only the byte transport differs: UDP
/// sockets here, UART/SPI RX lines (or Renode's emulated peripherals fed by
/// the same FMUs, see tests/swil) on hardware. Every decoded fix and IMU
/// sample is appended to a CSV so plot_results.py can compare what the
/// flight software *believes* against the rocket truth the co-simulation
/// logged.
///
/// Exits on its own once both streams go quiet (the co-simulation finished)
/// or a hard wall-clock cap is reached, so a scripted run never hangs.

#include "Hemerion/gps/gpsDriver.hpp"
#include "Hemerion/imu/fmu/imu_noise_model.h"  // ImuNoiseConfig: the simulated part's register sensitivity
#include "Hemerion/imu/imu_conversion.h"
#include "Hemerion/imu/imu_packet.h"
#include "udpReceiver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
using hemerion::sensors::imu::convert_raw_to_si;
using hemerion::sensors::imu::ImuConversionError;
using hemerion::sensors::imu::ImuPacketError;
using hemerion::sensors::imu::ImuPacketParser;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSample;

struct Options
{
  std::uint16_t gps_port = 5762;  // GPS FMU default UDP destination
  std::uint16_t imu_port = 5763;  // IMU FMU default UDP destination
  std::filesystem::path gps_csv_path = "results/gps_fixes.csv";
  std::filesystem::path imu_csv_path = "results/imu_samples.csv";
  double fix_period_s = 0.1;   // co-sim communication step; maps fix index -> sim time for the CSV
  int print_every = 50;        // console line every N fixes (50 = every 5 s of sim time at 10 Hz)
  int imu_print_every = 1000;  // console line every N IMU samples (1000 = every 10 s at 100 Hz)
  long quiet_ms = 3000;        // exit after this long with no datagrams, once at least one fix arrived
  long max_wall_s = 900;       // hard wall-clock cap
};

void print_usage()
{
  std::cout << "usage: gps_flight_computer [--port <udp port>] [--imu-port <udp port>]\n"
               "                           [--csv <file>] [--imu-csv <file>] [--fix-period <s>]\n"
               "                           [--print-every <n>] [--imu-print-every <n>]\n"
               "                           [--quiet-ms <ms>] [--max-wall-s <s>]\n";
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
      options.gps_port = static_cast<std::uint16_t>(std::stoi(value));
    }
    else if (arg == "--imu-port" && (value = next()))
    {
      options.imu_port = static_cast<std::uint16_t>(std::stoi(value));
    }
    else if (arg == "--csv" && (value = next()))
    {
      options.gps_csv_path = value;
    }
    else if (arg == "--imu-csv" && (value = next()))
    {
      options.imu_csv_path = value;
    }
    else if (arg == "--fix-period" && (value = next()))
    {
      options.fix_period_s = std::stod(value);
    }
    else if (arg == "--print-every" && (value = next()))
    {
      options.print_every = std::stoi(value);
    }
    else if (arg == "--imu-print-every" && (value = next()))
    {
      options.imu_print_every = std::stoi(value);
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

void print_imu_sample(long index, double sim_time_s, const ImuSample& sample)
{
  std::printf("[fc] imu %6ld  t=%7.1f s  f=[%8.2f %7.2f %7.2f] m/s2  w=[%8.4f %7.4f %7.4f] rad/s\n",
              index,
              sim_time_s,
              static_cast<double>(sample.accel_x),
              static_cast<double>(sample.accel_y),
              static_cast<double>(sample.accel_z),
              static_cast<double>(sample.gyro_x),
              static_cast<double>(sample.gyro_y),
              static_cast<double>(sample.gyro_z));
}

std::ofstream open_csv(const std::filesystem::path& path)
{
  if (path.has_parent_path())
  {
    std::filesystem::create_directories(path.parent_path());
  }
  return std::ofstream(path);
}

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  if (!parse_args(argc, argv, options))
  {
    return EXIT_FAILURE;
  }

  auto gps_receiver = UdpReceiver::create(options.gps_port);
  if (!gps_receiver.has_value())
  {
    std::cerr << "error: could not bind UDP port " << options.gps_port << " (already in use?)\n";
    return EXIT_FAILURE;
  }
  auto imu_receiver = UdpReceiver::create(options.imu_port);
  if (!imu_receiver.has_value())
  {
    std::cerr << "error: could not bind UDP port " << options.imu_port << " (already in use?)\n";
    return EXIT_FAILURE;
  }

  std::ofstream gps_csv = open_csv(options.gps_csv_path);
  if (!gps_csv)
  {
    std::cerr << "error: cannot open " << options.gps_csv_path.string() << " for writing\n";
    return EXIT_FAILURE;
  }
  gps_csv << "fix_index,sim_time_s,latitude_deg,longitude_deg,altitude_m,ground_speed_mps,course_deg,"
             "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type\n";

  std::ofstream imu_csv = open_csv(options.imu_csv_path);
  if (!imu_csv)
  {
    std::cerr << "error: cannot open " << options.imu_csv_path.string() << " for writing\n";
    return EXIT_FAILURE;
  }
  imu_csv << "sample_index,sim_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s\n";

  std::cout << "[fc] listening for UBX-NAV-PVT on UDP port " << options.gps_port
            << " (GpsDriver, protocol=UBX) and IMU raw-sample frames on UDP port " << options.imu_port
            << " (ImuPacketParser)\n";

  GpsDriver gps_driver(GpsProtocol::kUbx);
  ImuPacketParser imu_parser;
  // On real hardware the IMU driver knows the full-scale ranges because it
  // configured the part's registers itself; here the "configuration" is the
  // IMU FMU's default ImuNoiseConfig, so its scale is the one that converts
  // these counts back to SI.
  const hemerion::sensors::imu::ImuScale imu_scale = hemerion::sensors::imu::fmu::ImuNoiseConfig{}.scale;

  std::array<std::uint8_t, 2048> datagram{};
  GpsFix fix;
  ImuRawSample imu_raw;
  long fix_count = 0;
  long imu_sample_count = 0;
  long checksum_errors = 0;
  long imu_checksum_errors = 0;
  double max_altitude_m = 0.0;
  float max_speed_mps = 0.0F;
  double max_specific_force_mps2 = 0.0;
  double max_body_rate_rad_s = 0.0;

  const auto wall_start = std::chrono::steady_clock::now();
  auto last_datagram = wall_start;
  bool any_data = false;

  // Same per-byte feeds the firmware's sensor tasks perform on UART/SPI RX
  // data.
  auto feed_gps = [&](std::size_t received) {
    for (std::size_t i = 0; i < received; ++i)
    {
      const auto local_clock_us = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(last_datagram - wall_start).count());
      const GpsParseError result = gps_driver.feed(datagram[i], local_clock_us, fix);
      if (result == GpsParseError::kNone)
      {
        ++fix_count;
        any_data = true;
        // One NAV-PVT frame is emitted per communication step, so the fix
        // index maps directly to co-simulation time.
        const double sim_time_s = static_cast<double>(fix_count) * options.fix_period_s;
        max_altitude_m = std::max(max_altitude_m, static_cast<double>(fix.altitude_m));
        max_speed_mps = std::max(max_speed_mps, fix.ground_speed_mps);
        gps_csv << fix_count << ',' << sim_time_s << ',' << fix.latitude_deg << ',' << fix.longitude_deg << ','
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
  };

  auto feed_imu = [&](std::size_t received) {
    for (std::size_t i = 0; i < received; ++i)
    {
      const ImuPacketError result = imu_parser.parse_byte(datagram[i], imu_raw);
      if (result == ImuPacketError::kNone)
      {
        ImuSample sample;
        if (convert_raw_to_si(imu_raw, imu_scale, sample) != ImuConversionError::kNone)
        {
          continue;
        }
        ++imu_sample_count;
        any_data = true;
        // IMU frames carry the simulation clock in their payload.
        const double sim_time_s = static_cast<double>(sample.timestamp_us) * 1e-6;
        const double specific_force_mps2 = std::sqrt(
            static_cast<double>(sample.accel_x) * sample.accel_x + static_cast<double>(sample.accel_y) * sample.accel_y +
            static_cast<double>(sample.accel_z) * sample.accel_z);
        const double body_rate_rad_s = std::sqrt(
            static_cast<double>(sample.gyro_x) * sample.gyro_x + static_cast<double>(sample.gyro_y) * sample.gyro_y +
            static_cast<double>(sample.gyro_z) * sample.gyro_z);
        max_specific_force_mps2 = std::max(max_specific_force_mps2, specific_force_mps2);
        max_body_rate_rad_s = std::max(max_body_rate_rad_s, body_rate_rad_s);
        imu_csv << imu_sample_count << ',' << sim_time_s << ',' << sample.accel_x << ',' << sample.accel_y << ','
                << sample.accel_z << ',' << sample.gyro_x << ',' << sample.gyro_y << ',' << sample.gyro_z << '\n';
        if (imu_sample_count == 1 || imu_sample_count % options.imu_print_every == 0)
        {
          print_imu_sample(imu_sample_count, sim_time_s, sample);
        }
      }
      else if (result == ImuPacketError::kChecksumMismatch)
      {
        ++imu_checksum_errors;
      }
    }
  };

  while (true)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - wall_start > std::chrono::seconds(options.max_wall_s))
    {
      std::cout << "[fc] wall-clock cap reached\n";
      break;
    }
    if (any_data && now - last_datagram > std::chrono::milliseconds(options.quiet_ms))
    {
      std::cout << "[fc] sensor streams quiet for " << options.quiet_ms << " ms -- co-simulation finished\n";
      break;
    }

    // Alternate between the two sockets: block briefly on the first receive,
    // then drain whatever else is already queued without blocking, so the
    // IMU's 10-frames-per-step bursts never back up behind the GPS poll.
    auto drain = [&](const UdpReceiver& receiver, auto&& feed) {
      for (int drained = 0; drained < 512; ++drained)
      {
        const auto received =
            receiver.receive(datagram.data(), datagram.size(), std::chrono::milliseconds(drained == 0 ? 25 : 0));
        if (!received.has_value())
        {
          break;
        }
        last_datagram = std::chrono::steady_clock::now();
        feed(*received);
      }
    };
    drain(*gps_receiver, feed_gps);
    drain(*imu_receiver, feed_imu);
  }

  std::cout << "[fc] summary: " << fix_count << " fixes decoded (" << checksum_errors << " checksum errors), "
            << imu_sample_count << " IMU samples decoded (" << imu_checksum_errors << " checksum errors)\n"
            << "[fc] max altitude " << max_altitude_m << " m, max ground speed " << max_speed_mps << " m/s\n"
            << "[fc] max |specific force| " << max_specific_force_mps2 << " m/s2, max |body rate| "
            << max_body_rate_rad_s << " rad/s\n"
            << "[fc] fixes written to " << options.gps_csv_path.string() << ", IMU samples to "
            << options.imu_csv_path.string() << "\n";
  return (fix_count > 0 && imu_sample_count > 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
