// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file flight_computer_main.cpp
/// @brief Host stand-in for the STM32 flight computer's sensor ingest paths.
///
/// Runs the exact sensor stacks the STM32H743 target runs, over host
/// transports:
///
/// * **GPS** -- the GPS FMU's UBX-NAV-PVT stream arrives over UDP and goes
///   through the unmodified GpsDriver/UbxParser. On hardware those bytes come
///   off a UART RX line instead. The receiver's dynamics envelope is in force
///   -- by default the COCOM export limits alone -- so the stream stops dead
///   partway up and never resumes; this program logs and counts the outage
///   rather than pretending otherwise.
/// * **IMU** -- the IMU is an SPI part, so the flight computer drives it with
///   the unmodified on-target ImuSpiDriver: sample the data-ready line, read
///   STATUS/FIFO_COUNT, burst the FIFO port, feed every byte to
///   ImuPacketParser and convert with convert_raw_to_si(). The only piece
///   swapped out is the ImuSpiBus implementation below, which puts the
///   chip-select-framed transfers on a shared-memory bus (sim/spi_shm) the
///   hardware-simulator FMU answers; on the target that same interface wraps
///   HAL_SPI_TransmitReceive plus the CS and DRDY GPIOs.
/// * **Barometer** -- the BMP390 is an I2C part, driven with the unmodified
///   on-target Bmp390Driver: probe CHIP_ID, soft-reset, read the calibration
///   NVM, program OSR/ODR/INT_CTRL/PWR_CTRL, then poll STATUS and burst the
///   shadowed data block, compensating with the part's own coefficients. The
///   swapped piece is again only the bus: ShmI2cBus below puts the
///   transactions on a shared-memory I2C bus (sim/i2c_shm); on the target the
///   same interface wraps HAL_I2C_Mem_Read/Write
///   (Hemerion/baro/bmp390/bmp390_hal_i2c_bus.h).
///
/// Everything *around* those two stacks is harness, not firmware, and has no
/// counterpart on the target: the command line, the UDP socket, std::filesystem
/// and std::ofstream, and the std::cout/std::printf progress and summary lines.
/// On the STM32H743 this file's job is done by a pair of FreeRTOS tasks whose
/// output goes to a telemetry downlink and flash, and whose diagnostics go to
/// SEGGER RTT or a lock-free ring buffer drained over DMA UART -- not to
/// iostream, which costs tens of kilobytes of flash for locale machinery,
/// runs its static initializer before the scheduler starts, is task-safe only
/// with configUSE_NEWLIB_REENTRANT, and blocks the calling task on the UART.
/// The boundary is the driver layer: everything below it (GpsDriver, UbxParser,
/// ImuSpiDriver, ImuPacketParser, convert_raw_to_si) is freestanding -- caller-
/// owned buffers, no heap, no exceptions, no stdio -- and crosses unchanged.
///
/// Every decoded fix and IMU sample is appended to a CSV so plot_results.py
/// can compare what the flight software *believes* against the rocket truth
/// the co-simulation logged.
///
/// Exits on its own once both streams go quiet (the co-simulation finished)
/// or a hard wall-clock cap is reached, so a scripted run never hangs.

#include "Hemerion/baro/baro_types.h"
#include "Hemerion/baro/bmp390/bmp390_driver.h"
#include "Hemerion/baro/bmp390/bmp390_registers.h"
#include "Hemerion/gps/gpsDriver.hpp"
#include "Hemerion/imu/fmu/imu_noise_model.h"  // ImuNoiseConfig: the simulated part's register sensitivity
#include "Hemerion/imu/imu_conversion.h"
#include "Hemerion/imu/imu_spi_driver.h"
#include "udpReceiver.hpp"

#include <hemerion/sim/i2c_shm/i2c_shm_link.h>
#include <hemerion/sim/spi_shm/spi_shm_link.h>

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
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

namespace
{

using hemerion::examples::rocket_gps_ecos::UdpReceiver;
using hemerion::sensors::baro::BaroSample;
using hemerion::sensors::baro::bmp390::bmp390_odr_period_us;
using hemerion::sensors::baro::bmp390::Bmp390Config;
using hemerion::sensors::baro::bmp390::Bmp390Driver;
using hemerion::sensors::baro::bmp390::Bmp390Error;
using hemerion::sensors::baro::bmp390::Bmp390I2cBus;
using hemerion::sensors::baro::bmp390::Bmp390ReadResult;
using hemerion::sensors::baro::bmp390::kBmp390I2cAddressPrimary;
using hemerion::sensors::gps::GpsDriver;
using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsFixType;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::GpsProtocol;
using hemerion::sensors::imu::convert_raw_to_si;
using hemerion::sensors::imu::ImuConversionError;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSample;
using hemerion::sensors::imu::ImuSpiBus;
using hemerion::sensors::imu::ImuSpiDriver;
using hemerion::sensors::imu::ImuSpiError;
using hemerion::sim::i2c_shm::I2cShmController;
using hemerion::sim::spi_shm::SpiShmController;

struct Options
{
  std::uint16_t gps_port = 5762;                 // GPS FMU default UDP destination
  std::string imu_bus = "hemerion_imu_spi";      // IMU FMU default SPI bus name
  std::string baro_bus = "hemerion_bmp390_i2c";  // BMP390 FMU default I2C bus name
  long imu_wait_s = 120;                         // how long to wait for the sensor FMUs to bring their buses up
  std::filesystem::path gps_csv_path = "results/gps_fixes.csv";
  std::filesystem::path imu_csv_path = "results/imu_samples.csv";
  std::filesystem::path baro_csv_path = "results/baro_samples.csv";
  double fix_period_s = 0.1;   // co-sim communication step; maps fix index -> sim time for the CSV
  int print_every = 50;        // console line every N fixes (50 = every 5 s of sim time at 10 Hz)
  int imu_print_every = 1000;  // console line every N IMU samples (1000 = every 10 s at 100 Hz)
  int baro_print_every = 500;  // console line every N baro conversions (500 = every 10 s at 50 Hz)
  long quiet_ms = 3000;        // exit after this long with no sensor data, once at least one sample arrived
  long max_wall_s = 900;       // hard wall-clock cap
};

void print_usage()
{
  std::cout << "usage: gps_flight_computer [--port <udp port>] [--imu-bus <name>] [--baro-bus <name>]\n"
               "                           [--imu-wait-s <s>] [--csv <file>] [--imu-csv <file>]\n"
               "                           [--baro-csv <file>] [--fix-period <s>]\n"
               "                           [--print-every <n>] [--imu-print-every <n>] [--baro-print-every <n>]\n"
               "                           [--quiet-ms <ms>] [--max-wall-s <s>]\n"
               "\n"
               "  --port       UDP port the GPS FMU sends UBX-NAV-PVT to (default 5762)\n"
               "  --imu-bus    shared-memory SPI bus the IMU FMU answers on (default hemerion_imu_spi;\n"
               "               the FMU reads the same name from HEMERION_IMU_FMU_SPI_BUS)\n"
               "  --baro-bus   shared-memory I2C bus the BMP390 FMU answers on (default hemerion_bmp390_i2c;\n"
               "               the FMU reads the same name from HEMERION_BMP390_FMU_I2C_BUS)\n"
               "  --imu-wait-s how long to wait for those buses to appear, so either process may start first\n";
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

constexpr std::array<ValueOption, 13> kValueOptions = { {
    { "--port", [](Options& o, const char* v) { o.gps_port = static_cast<std::uint16_t>(std::stoi(v)); } },
    { "--imu-bus", [](Options& o, const char* v) { o.imu_bus = v; } },
    { "--baro-bus", [](Options& o, const char* v) { o.baro_bus = v; } },
    { "--imu-wait-s", [](Options& o, const char* v) { o.imu_wait_s = std::stol(v); } },
    { "--csv", [](Options& o, const char* v) { o.gps_csv_path = v; } },
    { "--imu-csv", [](Options& o, const char* v) { o.imu_csv_path = v; } },
    { "--baro-csv", [](Options& o, const char* v) { o.baro_csv_path = v; } },
    { "--fix-period", [](Options& o, const char* v) { o.fix_period_s = std::stod(v); } },
    { "--print-every", [](Options& o, const char* v) { o.print_every = std::stoi(v); } },
    { "--imu-print-every", [](Options& o, const char* v) { o.imu_print_every = std::stoi(v); } },
    { "--baro-print-every", [](Options& o, const char* v) { o.baro_print_every = std::stoi(v); } },
    { "--quiet-ms", [](Options& o, const char* v) { o.quiet_ms = std::stol(v); } },
    { "--max-wall-s", [](Options& o, const char* v) { o.max_wall_s = std::stol(v); } },
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

/// @brief The board, for the on-target IMU driver: chip-select-framed SPI
/// transfers over the shared-memory bus, plus the part's data-ready line.
///
/// This class is the whole difference between running the driver here and
/// running it on the STM32, where the same three operations are
/// HAL_SPI_TransmitReceive() between two GPIO writes, and a read of the DRDY
/// input pin.
class ShmSpiBus final : public ImuSpiBus
{
public:
  ShmSpiBus(SpiShmController& controller, std::chrono::milliseconds timeout)
    : controller_(controller), timeout_(timeout)
  {
  }

  bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t length) override
  {
    if (!controller_.transfer(tx, rx, length, timeout_))
    {
      // A transfer that fails because the part is no longer there is the
      // co-simulation ending, not a bus fault -- the FMU's terminate() powers
      // the peripheral down, and whichever poll is in flight at that moment
      // sees it first. Only a failure against a *live* peripheral is a fault.
      if (controller_.peripheral_present())
      {
        ++failed_transfers_;
      }
      return false;
    }
    ++transfers_;
    return true;
  }

  [[nodiscard]] bool data_ready_line() const override { return controller_.data_ready(); }

  [[nodiscard]] bool peripheral_present() const { return controller_.peripheral_present(); }

  [[nodiscard]] std::size_t transfers() const { return transfers_; }
  [[nodiscard]] std::size_t failed_transfers() const { return failed_transfers_; }

private:
  SpiShmController& controller_;
  std::chrono::milliseconds timeout_;
  std::size_t transfers_ = 0;
  std::size_t failed_transfers_ = 0;
};

/// @brief The board, for the on-target BMP390 driver: whole I2C transactions
/// over the shared-memory bus, plus the part's INT line.
///
/// This class is the whole difference between running the driver here and
/// running it on the STM32, where the same operations are
/// HAL_I2C_Mem_Read/Write and a GPIO read
/// (Hemerion/baro/bmp390/bmp390_hal_i2c_bus.h).
class ShmI2cBus final : public Bmp390I2cBus
{
public:
  ShmI2cBus(I2cShmController& controller, std::chrono::milliseconds timeout)
    : controller_(controller), timeout_(timeout)
  {
  }

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const std::array<std::uint8_t, 2> frame = { reg, value };
    return complete(
        controller_.transaction(kBmp390I2cAddressPrimary, frame.data(), frame.size(), nullptr, 0, timeout_));
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    return complete(controller_.transaction(kBmp390I2cAddressPrimary, &reg, 1, out, count, timeout_));
  }

  void delay_ms(std::uint32_t milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  [[nodiscard]] bool interrupt_line() const override { return controller_.interrupt_line(); }

  [[nodiscard]] bool peripheral_present() const { return controller_.peripheral_present(); }

  [[nodiscard]] std::size_t transactions() const { return transactions_; }
  [[nodiscard]] std::size_t failed_transactions() const { return failed_transactions_; }

private:
  bool complete(I2cShmController::Result result)
  {
    if (result != I2cShmController::Result::kOk)
    {
      // Same policy as ShmSpiBus: a transaction that fails because the part
      // powered down is the co-simulation ending, not a bus fault.
      if (controller_.peripheral_present())
      {
        ++failed_transactions_;
      }
      return false;
    }
    ++transactions_;
    return true;
  }

  I2cShmController& controller_;
  std::chrono::milliseconds timeout_;
  std::size_t transactions_ = 0;
  std::size_t failed_transactions_ = 0;
};

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

  // Bind the GPS socket before waiting on the IMU bus: datagrams that arrive
  // while this process is still bringing the SPI part up are then queued by
  // the OS rather than dropped on the floor.
  auto gps_receiver = UdpReceiver::create(options.gps_port);
  if (!gps_receiver.has_value())
  {
    std::cerr << "error: could not bind UDP port " << options.gps_port << " (already in use?)\n";
    return EXIT_FAILURE;
  }

  std::cout << "[fc] listening for UBX-NAV-PVT on UDP port " << options.gps_port << " (GpsDriver, protocol=UBX)\n"
            << "[fc] waiting up to " << options.imu_wait_s << " s for the IMU on SPI bus '" << options.imu_bus
            << "' and the BMP390 on I2C bus '" << options.baro_bus << "'\n";

  std::optional<SpiShmController> spi =
      SpiShmController::attach_within(options.imu_bus, std::chrono::seconds(options.imu_wait_s));
  if (!spi.has_value())
  {
    std::cerr << "error: no IMU answered on SPI bus '" << options.imu_bus
              << "' -- start rocket_gps_cosim (or point --imu-bus / HEMERION_IMU_FMU_SPI_BUS at the right bus)\n";
    return EXIT_FAILURE;
  }

  // The IMU wait above absorbs the co-simulation's startup; once one FMU has
  // its bus up the others are moments behind, so this wait is short only in
  // the happy path, not in what it tolerates.
  std::optional<I2cShmController> i2c =
      I2cShmController::attach_within(options.baro_bus, std::chrono::seconds(options.imu_wait_s));
  if (!i2c.has_value())
  {
    std::cerr << "error: no BMP390 answered on I2C bus '" << options.baro_bus
              << "' -- start rocket_gps_cosim (or point --baro-bus / HEMERION_BMP390_FMU_I2C_BUS at the right bus)\n";
    return EXIT_FAILURE;
  }

  // A transfer that a live peripheral does not answer within a second means
  // the simulated part is hung, not merely busy.
  ShmSpiBus imu_bus(*spi, std::chrono::milliseconds(1000));
  ImuSpiDriver imu_driver(imu_bus);
  switch (imu_driver.probe())
  {
    case ImuSpiError::kNone:
      std::cout << "[fc] IMU identified on SPI (WHO_AM_I matched), FIFO enabled\n";
      break;
    case ImuSpiError::kIdentityMismatch:
      std::cerr << "error: the part on SPI bus '" << options.imu_bus << "' is not the expected IMU\n";
      return EXIT_FAILURE;
    case ImuSpiError::kTransferFailed:
      std::cerr << "error: SPI transfer to the IMU failed during probe\n";
      return EXIT_FAILURE;
  }

  // The full BMP390 bring-up the STM32 task runs: identity, soft reset,
  // calibration NVM, OSR/ODR/INT_CTRL, normal mode. The default Bmp390Config
  // ODR (50 Hz) is what the simulated part will convert at from here on --
  // the FMU has no rate parameter to agree with.
  const Bmp390Config baro_config;
  ShmI2cBus baro_bus(*i2c, std::chrono::milliseconds(1000));
  Bmp390Driver baro_driver(baro_bus);
  switch (baro_driver.probe(baro_config))
  {
    case Bmp390Error::kNone:
      std::cout << "[fc] BMP390 identified on I2C (CHIP_ID matched), calibrated, converting at "
                << 1e6 / static_cast<double>(bmp390_odr_period_us(baro_config.odr_sel)) << " Hz\n";
      break;
    case Bmp390Error::kIdentityMismatch:
      std::cerr << "error: the part on I2C bus '" << options.baro_bus << "' is not a BMP390\n";
      return EXIT_FAILURE;
    case Bmp390Error::kTransferFailed:
      std::cerr << "error: I2C transaction to the BMP390 failed during probe\n";
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

  std::ofstream baro_csv = open_csv(options.baro_csv_path);
  if (!baro_csv)
  {
    std::cerr << "error: cannot open " << options.baro_csv_path.string() << " for writing\n";
    return EXIT_FAILURE;
  }
  baro_csv << "sample_index,sim_time_s,pressure_pa,temperature_c\n";

  GpsDriver gps_driver(GpsProtocol::kUbx);
  // On real hardware the IMU driver knows the full-scale ranges because it
  // configured the part's registers itself; here the "configuration" is the
  // IMU FMU's default ImuNoiseConfig, so its scale is the one that converts
  // these counts back to SI.
  const hemerion::sensors::imu::ImuScale imu_scale = hemerion::sensors::imu::fmu::ImuNoiseConfig{}.scale;

  std::array<std::uint8_t, 2048> datagram{};
  std::array<ImuRawSample, 64> imu_batch{};
  GpsFix fix;
  long fix_count = 0;        // NAV-PVT epochs decoded, valid or not
  long valid_fix_count = 0;  // epochs that actually carried a solution
  long imu_sample_count = 0;
  long baro_sample_count = 0;
  long checksum_errors = 0;
  long imu_checksum_errors = 0;
  long imu_fifo_overflows = 0;
  double first_outage_s = -1.0;
  double last_outage_s = -1.0;
  double max_altitude_m = 0.0;
  float max_speed_mps = 0.0F;
  double max_specific_force_mps2 = 0.0;
  double max_body_rate_rad_s = 0.0;
  double min_pressure_pa = std::numeric_limits<double>::infinity();
  double min_temperature_c = std::numeric_limits<double>::infinity();
  bool imu_bus_failed = false;
  bool baro_gone = false;

  const auto wall_start = std::chrono::steady_clock::now();
  auto last_sensor_data = wall_start;
  bool any_data = false;

  // The same per-byte feed the firmware's GPS task performs on UART RX data.
  auto feed_gps = [&](std::size_t received) {
    for (std::size_t i = 0; i < received; ++i)
    {
      const auto local_clock_us = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(last_sensor_data - wall_start).count());
      const GpsParseError result = gps_driver.feed(datagram[i], local_clock_us, fix);
      if (result == GpsParseError::kNone)
      {
        ++fix_count;
        any_data = true;
        // One NAV-PVT frame is emitted per communication step, so the epoch
        // index maps directly to co-simulation time -- including the epochs
        // the receiver reports with no solution, which is exactly why the
        // index and not the fix count is used here.
        const double sim_time_s = static_cast<double>(fix_count) * options.fix_period_s;
        if (fix.fix_type == GpsFixType::kNoFix)
        {
          // The dynamics envelope took the fix away. A real receiver keeps
          // filling NAV-PVT's position and velocity fields through a dropout,
          // and the parser faithfully decodes whatever is in them -- but they
          // are not a measurement of anything, so they are not logged as one.
          // The epoch is still recorded (its index carries the time mapping,
          // and fix_type marks the outage); it simply has no data in it.
          // Leaving the numbers in would invite exactly the mistake of
          // plotting them, which is how a 236 km "position" ends up in a
          // figure.
          if (first_outage_s < 0.0)
          {
            first_outage_s = sim_time_s;
            std::printf(
                "[fc] fix %5ld  t=%7.1f s  no fix -- receiver outside its dynamics envelope\n", fix_count, sim_time_s);
          }
          last_outage_s = sim_time_s;
          gps_csv << fix_count << ',' << sim_time_s << ",,,,,,,," << static_cast<unsigned>(fix.num_satellites) << ','
                  << static_cast<unsigned>(fix.fix_type) << '\n';
        }
        else
        {
          ++valid_fix_count;
          max_altitude_m = std::max(max_altitude_m, static_cast<double>(fix.altitude_m));
          max_speed_mps = std::max(max_speed_mps, fix.ground_speed_mps);
          if (valid_fix_count == 1 || fix_count % options.print_every == 0)
          {
            print_fix(fix_count, sim_time_s, fix);
          }
          gps_csv << fix_count << ',' << sim_time_s << ',' << fix.latitude_deg << ',' << fix.longitude_deg << ','
                  << fix.altitude_m << ',' << fix.ground_speed_mps << ',' << fix.course_deg << ','
                  << fix.horizontal_accuracy_m << ',' << fix.vertical_accuracy_m << ','
                  << static_cast<unsigned>(fix.num_satellites) << ',' << static_cast<unsigned>(fix.fix_type) << '\n';
        }
      }
      else if (result == GpsParseError::kChecksumMismatch)
      {
        ++checksum_errors;
      }
    }
  };

  // Block briefly on the first receive, then take whatever else is already
  // queued without blocking, so the IMU poll is never starved by a quiet GPS
  // stream and a burst never backs up in the socket.
  auto drain_gps = [&](std::chrono::milliseconds first_wait) {
    for (int drained = 0; drained < 512; ++drained)
    {
      const auto received = gps_receiver->receive(
          datagram.data(), datagram.size(), drained == 0 ? first_wait : std::chrono::milliseconds(0));
      if (!received.has_value())
      {
        break;
      }
      last_sensor_data = std::chrono::steady_clock::now();
      feed_gps(*received);
    }
  };

  // The same polling sequence the firmware's IMU task performs: DRDY, then
  // STATUS/FIFO_COUNT, then burst reads of the FIFO port.
  auto poll_imu = [&]() {
    while (imu_driver.data_ready())
    {
      const ImuSpiDriver::PollResult result = imu_driver.poll(imu_batch.data(), imu_batch.size());
      if (result.error == ImuSpiError::kTransferFailed)
      {
        // Told apart in the loop below by whether the part is still there.
        imu_bus_failed = imu_bus.peripheral_present();
        return;
      }
      if (result.fifo_overflow)
      {
        ++imu_fifo_overflows;
      }
      imu_checksum_errors += static_cast<long>(result.checksum_errors);
      if (result.samples == 0)
      {
        return;
      }
      last_sensor_data = std::chrono::steady_clock::now();
      any_data = true;

      for (std::size_t i = 0; i < result.samples; ++i)
      {
        ImuSample sample;
        if (convert_raw_to_si(imu_batch[i], imu_scale, sample) != ImuConversionError::kNone)
        {
          continue;
        }
        ++imu_sample_count;
        // IMU frames carry the simulation clock in their payload.
        const double sim_time_s = static_cast<double>(sample.timestamp_us) * 1e-6;
        const double specific_force_mps2 = std::sqrt(static_cast<double>(sample.accel_x) * sample.accel_x +
                                                     static_cast<double>(sample.accel_y) * sample.accel_y +
                                                     static_cast<double>(sample.accel_z) * sample.accel_z);
        const double body_rate_rad_s = std::sqrt(static_cast<double>(sample.gyro_x) * sample.gyro_x +
                                                 static_cast<double>(sample.gyro_y) * sample.gyro_y +
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
    }
  };

  // The same polling sequence the firmware's baro task performs: INT line,
  // then STATUS + the shadowed data/SENSORTIME burst. The data registers are
  // not a FIFO, so with the co-simulation exchanging variables every 0.1 s a
  // 50 Hz part's intermediate conversions are overwritten before any poll
  // could observe them -- the stream lands at ~one conversion per
  // communication step, each carrying the part's own SENSORTIME stamp, which
  // is what makes the log's time base right anyway.
  auto poll_baro = [&]() {
    if (baro_gone)
    {
      return;
    }
    while (baro_driver.data_ready())
    {
      BaroSample sample;
      const Bmp390ReadResult result = baro_driver.read_sample(sample);
      if (result == Bmp390ReadResult::kTransferFailed)
      {
        // Same split as the IMU: against a live part this is a fault; against
        // a powered-down part it is the co-simulation ending.
        baro_gone = true;
        return;
      }
      if (result != Bmp390ReadResult::kSample)
      {
        return;
      }
      last_sensor_data = std::chrono::steady_clock::now();
      any_data = true;
      ++baro_sample_count;
      // SENSORTIME wraps every 512 s; this flight fits inside one wrap.
      const double sim_time_s = static_cast<double>(sample.timestamp_us) * 1e-6;
      min_pressure_pa = std::min(min_pressure_pa, static_cast<double>(sample.pressure_pa));
      min_temperature_c = std::min(min_temperature_c, static_cast<double>(sample.temperature_c));
      baro_csv << baro_sample_count << ',' << sim_time_s << ',' << sample.pressure_pa << ',' << sample.temperature_c
               << '\n';
      if (baro_sample_count == 1 || baro_sample_count % options.baro_print_every == 0)
      {
        std::printf("[fc] baro %5ld  t=%7.1f s  p=%9.1f Pa  T=%6.2f C\n",
                    baro_sample_count,
                    sim_time_s,
                    static_cast<double>(sample.pressure_pa),
                    static_cast<double>(sample.temperature_c));
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
    if (any_data && now - last_sensor_data > std::chrono::milliseconds(options.quiet_ms))
    {
      std::cout << "[fc] sensor streams quiet for " << options.quiet_ms << " ms -- co-simulation finished\n";
      break;
    }
    if (imu_bus_failed)
    {
      std::cout << "[fc] SPI transfer to the IMU failed -- the simulated part is gone\n";
      break;
    }
    if (!spi->peripheral_present())
    {
      // The FMU terminated: the part has been powered down. Drain whatever
      // the GPS socket still holds and stop.
      std::cout << "[fc] IMU powered down (FMU terminated) -- co-simulation finished\n";
      break;
    }

    drain_gps(std::chrono::milliseconds(5));
    poll_imu();
    poll_baro();
  }

  // The co-simulation stopping does not mean the last datagrams have been
  // read: UDP queues them in the socket, and the IMU FMU's power-down is what
  // ends the loop above. Take whatever is still queued before summarising, so
  // the counts reflect what was sent rather than where the loop happened to
  // exit.
  for (int idle = 0; idle < 20; ++idle)
  {
    const long before = fix_count;
    drain_gps(std::chrono::milliseconds(10));
    if (fix_count == before)
    {
      break;
    }
    idle = 0;
  }

  const long no_fix_epochs = fix_count - valid_fix_count;
  std::cout << "[fc] summary: " << fix_count << " NAV-PVT epochs decoded (" << checksum_errors << " checksum errors), "
            << valid_fix_count << " carried a fix, " << no_fix_epochs << " did not\n";
  if (no_fix_epochs > 0)
  {
    // Which limit did it is the receiver's business, not the flight computer's:
    // NAV-PVT carries a cleared gnssFixOK flag and nothing about why.
    std::cout << "[fc] no-fix window: t=" << first_outage_s << " s to t=" << last_outage_s
              << " s (receiver outside its dynamics envelope)\n";
  }
  std::cout << "[fc] " << imu_sample_count << " IMU samples decoded over " << imu_bus.transfers() << " SPI transfers ("
            << imu_checksum_errors << " checksum errors, " << imu_fifo_overflows << " FIFO overflows, "
            << imu_bus.failed_transfers() << " failed transfers)\n"
            << "[fc] " << baro_sample_count << " BMP390 conversions read over " << baro_bus.transactions()
            << " I2C transactions (" << baro_bus.failed_transactions() << " failed)\n";
  if (baro_sample_count > 0)
  {
    std::cout << "[fc] min pressure " << min_pressure_pa << " Pa, min temperature " << min_temperature_c
              << " C (both at the top of the observed trajectory)\n";
  }
  // With a launch-vehicle envelope in force the receiver can hold no usable
  // fix at all, and "max altitude 0 m" would read as a measurement rather than
  // as the absence of one.
  if (valid_fix_count > 0)
  {
    std::cout << "[fc] max altitude " << max_altitude_m << " m, max ground speed " << max_speed_mps << " m/s (over "
              << valid_fix_count << (valid_fix_count == 1 ? " fix" : " fixes") << " carrying a solution)\n";
  }
  else
  {
    std::cout << "[fc] no position summary: the receiver never reported a usable fix\n";
  }
  std::cout << "[fc] max |specific force| " << max_specific_force_mps2 << " m/s2, max |body rate| "
            << max_body_rate_rad_s << " rad/s\n"
            << "[fc] fixes written to " << options.gps_csv_path.string() << ", IMU samples to "
            << options.imu_csv_path.string() << ", baro samples to " << options.baro_csv_path.string() << "\n";
  return (fix_count > 0 && imu_sample_count > 0 && baro_sample_count > 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
