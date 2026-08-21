// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief BMP390 barometer hardware simulator, exported as an FMI
/// Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is an **I2C side
/// channel**. Each step maps the current truth altitude through the ICAO
/// Standard Atmosphere and the part's error model to raw 24-bit conversion
/// words (Bmp390MeasurementModel, which numerically inverts the real Bosch
/// compensation), and latches them into the simulated part's data registers
/// (Bmp390I2cSlave). A controller -- a host flight computer process, or
/// emulated firmware under Renode once an I2C bridge exists -- then reads
/// the part the way real firmware does: probe CHIP_ID, soft-reset, read the
/// calibration NVM, program OSR/ODR/INT_CTRL/PWR_CTRL, poll STATUS, burst
/// the shadowed data block. The words it recovers compensate back to the
/// noisy truth through the unmodified on-target Bmp390Driver +
/// Bmp390Compensator, exactly as bytes from physical silicon would.
///
/// Three things have to be true for that, and each belongs somewhere
/// different -- this file is only the third:
///
/// * **the datasheet** -- register map, calibration NVM, bus state machine:
///   Bmp390I2cSlave (bmp390_i2c_slave.h), which knows nothing about how
///   bytes reach it and is unit-tested against the real on-target driver
///   with no transport in the picture;
/// * **the board** -- which bus the part sits on, when it comes up and goes
///   down, how the INT line is driven: I2cPeripheralEndpoint (sim/i2c_shm),
///   sensor-agnostic and shared by any FMU that models an I2C part;
/// * **the part number** -- which device model on which bus, plus the FMI
///   variables and the physics feeding it. That is all this file does.
///
/// The bus name defaults to `hemerion_bmp390_i2c` and is overridable through
/// HEMERION_BMP390_FMU_I2C_BUS, matching how the other sensor FMUs take
/// their destination from the environment (no FMI String-typed variables,
/// retargetable without repackaging the archive).
///
/// **The sampling rate is the driver's, not the FMU's.** Unlike the generic
/// sensor FMUs with their `sample_rate_hz` parameter, this part converts at
/// the ODR the firmware programs into it: do_step() asks the device model
/// for the period its PWR_CTRL/ODR registers currently select and latches
/// conversions on that schedule (none in sleep mode, one per forced-mode
/// trigger). Configuration observable in the sample stream is exactly the
/// kind of coupling a protocol-accurate simulator exists to exercise.
///
/// The INT line level is re-published once per step; a clear-on-read inside
/// a step therefore reaches the shared bus line at the next communication
/// point. The STATUS register, which the driver polls, clears immediately.
///
/// All the FMI plumbing -- entry points, variable marshalling, GUID handling
/// and modelDescription.xml generation -- belongs to the vendored fmu4cpp
/// export layer (vendor/fmu4cpp). This file only registers the variables and
/// implements do_step(); see cmake/generate_fmu.cmake for how the two halves
/// are compiled and packaged into an .fmu archive.

#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"

#include <hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h>

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <cstdint>

namespace hemerion::sensors::baro::bmp390::fmu
{

namespace
{

using fmu4cpp::causality_t;

/// Where this part sits: the bus it creates, and the environment variable a
/// launch script can retarget it with.
const sim::i2c_shm::I2cPeripheralConfig kI2cBus{ "hemerion_bmp390_i2c", "HEMERION_BMP390_FMU_I2C_BUS" };

}  // namespace

/// @brief Co-simulation slave turning truth altitude into raw BMP390
/// conversion words a controller reads over a simulated I2C bus.
class Bmp390SimulatorFmu final : public fmu4cpp::fmu_base
{
public:
  FMU4CPP_CTOR(Bmp390SimulatorFmu)
  {
    // The measurement model maps this through the ICAO Standard Atmosphere
    // (troposphere + isothermal stratosphere layers) to the ambient pressure
    // and temperature the part would convert.
    register_real("h_m", &altitude_m_)
        .setCausality(causality_t::INPUT)
        .setDescription("True geometric altitude above mean sea level [m]");
  }

  /// Brings the simulated part up on its bus. Deliberately not done in the
  /// constructor: the build-time modelDescription.xml generator instantiates
  /// the model purely to enumerate its variables, and that must not create
  /// shared-memory objects or spawn threads.
  void exit_initialisation_mode() override
  {
    if (!endpoint_.attach())
    {
      throw fmu4cpp::fatal_error("[hemerion_bmp390_fmu] Unable to create the I2C bus '" + endpoint_.bus_name() +
                                 "' (already in use by another run?)");
    }
    debugLog(fmiOK, "[hemerion_bmp390_fmu] I2C peripheral ready on bus '" + endpoint_.bus_name() + "'");
  }

  /// Powers the part down. The data registers hold at most one conversion,
  /// so unlike the FIFO-buffering IMU there is nothing to let a controller
  /// drain: whatever is unread at power-down is lost, as on the real board.
  void terminate() override { endpoint_.detach(); }

  /// fmi2Reset equivalent. The turn-on biases Bmp390MeasurementModel drew at
  /// construction are kept: they model this instance's physical part, which
  /// a reset does not swap out.
  void reset() override
  {
    altitude_m_ = 0.0;
    time_into_period_s_ = 0.0;
    slave_.reset();
    endpoint_.detach();
  }

protected:
  bool do_step(double dt) override
  {
    if (!endpoint_.attached())
    {
      throw fmu4cpp::fatal_error("[hemerion_bmp390_fmu] Stepped before initialisation mode was exited");
    }

    // One conversion per forced-mode trigger, regardless of the ODR.
    if (slave_.take_forced_conversion())
    {
      latch_one(currentTime());
    }

    // Normal mode: the part free-runs at the period its ODR register
    // selects. Whole periods elapsed within this step each latch a fresh
    // conversion (truth zero-order-held, noise redrawn) stamped with its own
    // conversion time, and the remainder carries into the next step so a
    // 50 Hz part stepped at 10 Hz really converts at 50 Hz. The data
    // registers are not a FIFO -- a controller that polls slower than the
    // ODR observes only the newest conversion, and SENSORTIME is how it
    // still knows when that conversion happened.
    const std::uint64_t period_us = slave_.sampling_period_us();
    if (period_us == 0)
    {
      time_into_period_s_ = 0.0;  // sleep: the measurement engine is idle
    }
    else
    {
      const double period_s = static_cast<double>(period_us) * 1e-6;
      double remaining_s = dt;
      while (time_into_period_s_ + remaining_s >= period_s)
      {
        remaining_s -= period_s - time_into_period_s_;
        time_into_period_s_ = 0.0;
        latch_one(currentTime() + (dt - remaining_s));
      }
      time_into_period_s_ += remaining_s;
    }

    // The INT line is a level; re-drive it so clear-on-read inside the step
    // reaches the bus line here.
    endpoint_.publish_interrupt(slave_.interrupt_asserted());
    return true;
  }

private:
  void latch_one(double sample_time_s)
  {
    const Bmp390MeasurementModel::Conversion conversion = measurement_model_.measure(altitude_m_);
    slave_.latch_conversion(
        conversion.uncomp_press, conversion.uncomp_temp, static_cast<std::uint64_t>(sample_time_s * 1e6));
  }

  Bmp390MeasurementModel measurement_model_;
  Bmp390I2cSlave slave_{ measurement_model_.calibration() };
  sim::i2c_shm::I2cPeripheralEndpoint<Bmp390I2cSlave> endpoint_{ slave_, kI2cBus };
  double altitude_m_ = 0.0;
  double time_into_period_s_ = 0.0;
};

}  // namespace hemerion::sensors::baro::bmp390::fmu

/// @cond FMI_ENTRY_POINTS
/// fmu4cpp's C entry points. Excluded from the API reference: they are
/// the FMI 2.0/3.0 ABI the packaging layer requires, not Hemerion API,
/// and fmu4cpp's own headers are not part of the Doxygen input.
fmu4cpp::model_info fmu4cpp::get_model_info()
{
  model_info info;
  info.modelName = "HemerionBmp390Simulator";
  info.author = "Onur Tuncer, Istanbul Technical University";
  info.description = "Register-accurate Bosch BMP390 barometer hardware simulator on a simulated I2C bus "
                     "for SWIL/HIL co-simulation";
  // h_m carries no FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::baro::bmp390::fmu::Bmp390SimulatorFmu);
/// @endcond
