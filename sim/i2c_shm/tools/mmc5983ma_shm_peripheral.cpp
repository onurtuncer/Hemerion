// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_shm_peripheral.cpp
/// @brief Standalone MMC5983MA on a shared-memory I2C bus: the FMU minus FMI.
///
/// Runs the exact device stack hemerion_mmc5983ma_fmu packages --
/// Mmc5983maI2cSlave (register file, SET/RESET magnetization state, bus state
/// machine) fed by Mmc5983maMeasurementModel (hard iron + bridge offset +
/// noise + fixed-scale quantization) through I2cPeripheralEndpoint -- but
/// paced by wall time at a fixed truth field instead of by a co-simulation
/// master. Exists for harnesses that need the *part* without an FMI master in
/// the room: tests/swil/test_mag_logger.py runs Renode-emulated firmware
/// against it through i2c_shm_tcp_bridge.
///
/// Faithful to the FMU's behaviour: one measurement per TM_M trigger (the
/// path the controller's bring-up handshake blocks on), one per TM_T, and
/// otherwise nothing at all until the controller programs Cmm_en and a rate
/// into `Internal control 2`. The default truth field is roughly Istanbul's:
/// ~48 uT total, steeply inclined.
///
/// The bridge offset matters more here than any other knob. It is drawn once
/// per run from --seed at the datasheet's +/-0.5 gauss null-field tolerance,
/// so a controller that skips the SET/RESET pair reads a field wrong by about
/// its own magnitude -- which is the whole reason this part is worth
/// simulating rather than stubbing. Pass --seed to reproduce a specific one.
///
/// Runs until --duration-s elapses, or until SIGINT / SIGTERM asks it to stop
/// -- the bus segment is only unlinked when this process returns from main(),
/// so a harness must signal rather than SIGKILL.
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

#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h"
#include "tool_args.h"

namespace
{

using hemerion::sensors::mag::mmc5983ma::kMmc5983maLsbPerMicrotesla;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maI2cSlave;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;
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
  std::string bus_name = "hemerion_mmc5983ma_i2c";
  double field_x_ut = 22.0;
  double field_y_ut = -6.0;
  double field_z_ut = 41.0;
  double temperature_c = 25.0;
  std::uint64_t seed = 42;
  long duration_s = 0;  // 0 = run until signalled

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    const char* value = nullptr;
    auto number = [&](double& target, const char* what) {
      if (!parse_number(value, target))
      {
        std::fprintf(stderr, "[mmc5983ma_peripheral] %s '%s' is not a number\n", what, value);
        return false;
      }
      return true;
    };

    if (arg == "--bus" && (value = next()))
    {
      bus_name = value;
    }
    else if (arg == "--bx" && (value = next()))
    {
      if (!number(field_x_ut, "--bx"))
      {
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--by" && (value = next()))
    {
      if (!number(field_y_ut, "--by"))
      {
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--bz" && (value = next()))
    {
      if (!number(field_z_ut, "--bz"))
      {
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--temp" && (value = next()))
    {
      if (!number(temperature_c, "--temp"))
      {
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--seed" && (value = next()))
    {
      if (!parse_number(value, seed))
      {
        std::fprintf(stderr, "[mmc5983ma_peripheral] --seed '%s' is not a non-negative integer\n", value);
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--duration-s" && (value = next()))
    {
      if (!parse_number(value, duration_s))
      {
        std::fprintf(stderr, "[mmc5983ma_peripheral] --duration-s '%s' is not a number of seconds\n", value);
        return EXIT_FAILURE;
      }
    }
    else
    {
      std::fprintf(stderr,
                   "usage: mmc5983ma_shm_peripheral [--bus <name>] [--bx <uT>] [--by <uT>] [--bz <uT>]\n"
                   "                                [--temp <C>] [--seed <n>] [--duration-s <s>]\n"
                   "  an MMC5983MA in a fixed truth field (default 22 / -6 / 41 uT) answering\n"
                   "  shared-memory I2C bus <name> (default hemerion_mmc5983ma_i2c); 0 s duration\n"
                   "  (default) runs until SIGINT/SIGTERM (SIGKILL leaks the bus segment)\n");
      return EXIT_FAILURE;
    }
  }

  Mmc5983maMeasurementModel model({}, seed);
  Mmc5983maI2cSlave slave;
  I2cPeripheralEndpoint<Mmc5983maI2cSlave> endpoint(slave, I2cPeripheralConfig{ bus_name, "" });
  if (!endpoint.attach())
  {
    std::fprintf(
        stderr, "[mmc5983ma_peripheral] cannot create bus '%s' (already in use by another run?)\n", bus_name.c_str());
    return EXIT_FAILURE;
  }

  // The default --duration-s 0 mode has no other way out, and the way out
  // matters: the bus lives in a shared-memory segment that is only unlinked
  // when `endpoint` is destroyed at the end of main(). A SIGKILL skips that
  // and strands the segment (and its name) until the machine reboots, so the
  // documented shutdown route is a signal these handlers can observe.
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  // Printed because a harness asserting on the controller's recovered offset
  // needs the truth to compare against, and because an offset this large is
  // startling if you have not read the datasheet's null-field tolerance.
  const auto& offset = model.bridge_offset();
  std::printf("[mmc5983ma_peripheral] MMC5983MA at %.1f/%.1f/%.1f uT answering on bus '%s'\n",
              field_x_ut,
              field_y_ut,
              field_z_ut,
              bus_name.c_str());
  std::printf("[mmc5983ma_peripheral] bridge offset (seed %llu): %ld/%ld/%ld LSB = %.2f/%.2f/%.2f uT\n",
              static_cast<unsigned long long>(seed),
              static_cast<long>(offset.x),
              static_cast<long>(offset.y),
              static_cast<long>(offset.z),
              static_cast<double>(offset.x) / kMmc5983maLsbPerMicrotesla,
              static_cast<double>(offset.y) / kMmc5983maLsbPerMicrotesla,
              static_cast<double>(offset.z) / kMmc5983maLsbPerMicrotesla);
  std::fflush(stdout);

  // The measurement engine, paced by this process's clock: the same schedule
  // the FMU derives from its communication steps, with wall time standing in
  // for simulation time. One millisecond, not the BMP390 tool's two, because
  // the controller's bring-up blocks on a triggered measurement and every
  // iteration of this loop is a millisecond of that wait.
  const auto start = std::chrono::steady_clock::now();
  auto previous = start;
  double time_into_period_s = 0.0;
  for (;;)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;

    // The sensing state is read at the moment of the measurement, never
    // cached: SET and RESET arrive as register writes on the bus service
    // thread, and a measurement taken after a RESET must come back negated.
    auto measure = [&] {
      slave.latch_measurement(model.measure(field_x_ut, field_y_ut, field_z_ut, slave.sensing_state()));
    };

    if (slave.take_triggered_measurement())
    {
      measure();
    }
    if (slave.take_triggered_temperature())
    {
      slave.latch_temperature(model.measure_temperature(temperature_c));
    }

    const std::uint64_t period_us = slave.sampling_period_us();
    if (period_us == 0)
    {
      time_into_period_s = 0.0;  // one-shot: the measurement engine is idle
    }
    else
    {
      const double period_s = static_cast<double>(period_us) * 1e-6;
      time_into_period_s += dt;
      // Wall pacing can fall behind (a laggy scheduler tick); latch the
      // newest measurement and drop the arrears rather than bursting stale
      // ones -- the data registers hold only one anyway.
      if (time_into_period_s >= period_s)
      {
        measure();
        time_into_period_s = std::fmod(time_into_period_s, period_s);
      }
    }

    endpoint.publish_interrupt(slave.interrupt_asserted());

    if (g_stop_requested != 0)
    {
      std::fprintf(stderr, "[mmc5983ma_peripheral] stop requested -- detaching from bus '%s'\n", bus_name.c_str());
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
