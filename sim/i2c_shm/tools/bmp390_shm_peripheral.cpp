// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_shm_peripheral.cpp
/// @brief Standalone BMP390 on a shared-memory I2C bus: the FMU minus FMI.
///
/// Runs the exact device stack hemerion_bmp390_fmu packages -- Bmp390I2cSlave
/// (register file + bus state machine) fed by Bmp390MeasurementModel (ISA +
/// noise + inverse compensation) through I2cPeripheralEndpoint -- but paced
/// by wall time at a fixed truth altitude instead of by a co-simulation
/// master. Exists for harnesses that need the *part* without an FMI master
/// in the room: tests/swil/test_baro_logger.py runs Renode-emulated firmware
/// against it through i2c_shm_tcp_bridge.
///
/// Faithful to the FMU's behaviour: nothing is latched until the controller
/// programs PWR_CTRL into normal mode, conversions then come at the ODR the
/// controller selected, one per forced-mode trigger, and SENSORTIME carries
/// this process's clock. Runs until --duration-s elapses, or until SIGINT /
/// SIGTERM asks it to stop -- the bus segment is only unlinked when this
/// process returns from main(), so a harness must signal rather than SIGKILL.
///
/// Host-only tool, like everything under sim/.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"
#include "hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h"
#include "tool_args.h"

namespace
{

using hemerion::sensors::baro::bmp390::fmu::Bmp390I2cSlave;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementModel;
using hemerion::sim::i2c_shm::I2cPeripheralConfig;
using hemerion::sim::i2c_shm::I2cPeripheralEndpoint;
using hemerion::sim::i2c_shm::tools::parse_number;

/// Raised by the SIGINT/SIGTERM handler, polled by the pacing loop.
/// volatile sig_atomic_t is the only object a signal handler may portably
/// touch, and it is all this needs: the handler writes, the loop reads.
volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void request_stop(int /*signal_number*/) { g_stop_requested = 1; }

}  // namespace

int main(int argc, char** argv)
{
  std::string bus_name = "hemerion_bmp390_i2c";
  double altitude_m = 500.0;
  std::uint64_t seed = 42;
  long duration_s = 0;  // 0 = run until signalled

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    const char* value = nullptr;
    if (arg == "--bus" && (value = next()))
    {
      bus_name = value;
    }
    else if (arg == "--alt" && (value = next()))
    {
      if (!parse_number(value, altitude_m))
      {
        std::fprintf(stderr, "[bmp390_peripheral] --alt '%s' is not a number of metres\n", value);
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--seed" && (value = next()))
    {
      if (!parse_number(value, seed))
      {
        std::fprintf(stderr, "[bmp390_peripheral] --seed '%s' is not a non-negative integer\n", value);
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--duration-s" && (value = next()))
    {
      if (!parse_number(value, duration_s))
      {
        std::fprintf(stderr, "[bmp390_peripheral] --duration-s '%s' is not a number of seconds\n", value);
        return EXIT_FAILURE;
      }
    }
    else
    {
      std::fprintf(stderr,
                   "usage: bmp390_shm_peripheral [--bus <name>] [--alt <m>] [--seed <n>] [--duration-s <s>]\n"
                   "  a BMP390 at fixed truth altitude <m> (default 500) answering shared-memory\n"
                   "  I2C bus <name> (default hemerion_bmp390_i2c); 0 s duration (default) runs\n"
                   "  until SIGINT/SIGTERM (SIGKILL leaks the bus segment)\n");
      return EXIT_FAILURE;
    }
  }

  Bmp390MeasurementModel model({}, seed);
  Bmp390I2cSlave slave(model.calibration());
  I2cPeripheralEndpoint<Bmp390I2cSlave> endpoint(slave, I2cPeripheralConfig{ bus_name, "" });
  if (!endpoint.attach())
  {
    std::fprintf(
        stderr, "[bmp390_peripheral] cannot create bus '%s' (already in use by another run?)\n", bus_name.c_str());
    return EXIT_FAILURE;
  }

  // The default --duration-s 0 mode has no other way out, and the way out
  // matters: the bus lives in a shared-memory segment that is only unlinked
  // when `endpoint` is destroyed at the end of main(). A SIGKILL skips that
  // and strands the segment (and its name) until the machine reboots, so the
  // documented shutdown route is a signal these handlers can observe.
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  std::printf("[bmp390_peripheral] BMP390 at %.1f m answering on bus '%s'\n", altitude_m, bus_name.c_str());
  std::fflush(stdout);

  // The measurement engine, paced by this process's clock: the same
  // conversion schedule the FMU derives from its communication steps, with
  // wall time standing in for simulation time.
  const auto start = std::chrono::steady_clock::now();
  auto previous = start;
  double time_into_period_s = 0.0;
  for (;;)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;

    const auto elapsed_us =
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());

    if (slave.take_forced_conversion())
    {
      const auto conversion = model.measure(altitude_m);
      slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp, elapsed_us);
    }

    const std::uint64_t period_us = slave.sampling_period_us();
    if (period_us == 0)
    {
      time_into_period_s = 0.0;  // sleep mode: the measurement engine is idle
    }
    else
    {
      const double period_s = static_cast<double>(period_us) * 1e-6;
      time_into_period_s += dt;
      // Wall pacing can fall behind (a laggy scheduler tick); latch the
      // newest conversion and drop the arrears rather than bursting stale
      // ones -- the data registers hold only one anyway.
      if (time_into_period_s >= period_s)
      {
        const auto conversion = model.measure(altitude_m);
        slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp, elapsed_us);
        time_into_period_s = std::fmod(time_into_period_s, period_s);
      }
    }

    endpoint.publish_interrupt(slave.interrupt_asserted());

    if (g_stop_requested != 0)
    {
      std::fprintf(stderr, "[bmp390_peripheral] stop requested -- detaching from bus '%s'\n", bus_name.c_str());
      break;
    }

    if (duration_s > 0 && now - start > std::chrono::seconds(duration_s))
    {
      break;
    }
  }

  endpoint.detach();
  return EXIT_SUCCESS;
}
