// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief MEMSIC MMC5983MA magnetometer hardware simulator, exported as an
/// FMI Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is an **I2C side
/// channel**. Each step maps the current truth body-frame field through the
/// part's error model to raw 18-bit counts (Mmc5983maMeasurementModel) and
/// latches them into the simulated part's data registers (Mmc5983maI2cSlave).
/// A controller -- a host flight computer process, or emulated firmware under
/// Renode over the I2C bridge -- then reads the part the way real firmware
/// does: identify, software-reset, program bandwidth, SET, take a
/// measurement, RESET, take another, difference the pair for the bridge
/// offset, enter continuous mode, poll `Status`, burst the seven data bytes.
/// What it recovers converts back to the noisy truth through the unmodified
/// on-target Mmc5983maDriver + convert_raw_to_si(), exactly as bytes from
/// physical silicon would.
///
/// Three things have to be true for that, and each belongs somewhere
/// different -- this file is only the third:
///
/// * **the datasheet** -- register map, write-only control registers,
///   SET/RESET magnetization state, bus state machine: Mmc5983maI2cSlave
///   (mmc5983ma_i2c_slave.h), which knows nothing about how bytes reach it
///   and is unit-tested against the real on-target driver with no transport
///   in the picture;
/// * **the board** -- which bus the part sits on, when it comes up and goes
///   down, how the INT line is driven: I2cPeripheralEndpoint (sim/i2c_shm),
///   sensor-agnostic and shared by any FMU that models an I2C part;
/// * **the part number** -- which device model on which bus, plus the FMI
///   variables and the physics feeding it. That is all this file does.
///
/// The bus name defaults to `hemerion_mmc5983ma_i2c` and is overridable
/// through HEMERION_MMC5983MA_FMU_I2C_BUS, matching how the other sensor FMUs
/// take their destination from the environment (no FMI String-typed
/// variables, retargetable without repackaging the archive).
///
/// **The sampling rate is the driver's, not the FMU's.** Unlike the generic
/// magnetometer FMU with its `sample_rate_hz` parameter, this part measures
/// when the firmware tells it to: do_step() serves one measurement per TM_M
/// trigger, one per TM_T trigger, and otherwise free-runs at the period the
/// `Internal control 2` registers currently select (none at all while
/// continuous mode is off). Configuration observable in the sample stream is
/// exactly the kind of coupling a protocol-accurate simulator exists to
/// exercise.
///
/// **One co-simulation constraint worth stating.** The driver's blocking
/// paths -- calibrate_offset() and measure_once() -- trigger a measurement
/// and then poll, and this part only advances when the FMI master steps this
/// FMU. The master's communication step must therefore be shorter than
/// Mmc5983maDriver::kMeasurementPollAttempts x kMeasurementPollIntervalMs
/// (50 ms as configured), or bring-up times out against a part that is
/// merely paused.
///
/// The INT line level is re-published once per step; a clear-on-write inside
/// a step therefore reaches the shared bus line at the next communication
/// point. The `Status` register, which the driver polls, clears immediately.
///
/// All the FMI plumbing -- entry points, variable marshalling, GUID handling
/// and modelDescription.xml generation -- belongs to the vendored fmu4cpp
/// export layer (vendor/fmu4cpp). This file only registers the variables and
/// implements do_step(); see cmake/generate_fmu.cmake for how the two halves
/// are compiled and packaged into an .fmu archive.

#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"

#include <hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h>

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <cstdint>

namespace hemerion::sensors::mag::mmc5983ma::fmu
{

namespace
{

using fmu4cpp::causality_t;

/// Where this part sits: the bus it creates, and the environment variable a
/// launch script can retarget it with.
const sim::i2c_shm::I2cPeripheralConfig kI2cBus{ "hemerion_mmc5983ma_i2c", "HEMERION_MMC5983MA_FMU_I2C_BUS" };

/// Die temperature the part reports when the host does not drive it.
constexpr double kDefaultTemperatureC = 25.0;

}  // namespace

/// @brief Co-simulation slave turning body-frame truth field into raw
/// MMC5983MA counts a controller reads over a simulated I2C bus.
class Mmc5983maSimulatorFmu final : public fmu4cpp::fmu_base
{
public:
  FMU4CPP_CTOR(Mmc5983maSimulatorFmu)
  {
    // The local geomagnetic reference field rotated into the body frame by
    // the plant's attitude: the co-simulation host computes b_body = C_bn *
    // b_ned from a reference field (WMM, or a fixed launch-site vector). The
    // names match the generic magnetometer FMU's, so the two are drop-in
    // alternatives on a plant's output.
    register_real("b_x_ut", &truth_x_ut_)
        .setCausality(causality_t::INPUT)
        .setDescription("True magnetic field, body X [uT]");
    register_real("b_y_ut", &truth_y_ut_)
        .setCausality(causality_t::INPUT)
        .setDescription("True magnetic field, body Y [uT]");
    register_real("b_z_ut", &truth_z_ut_)
        .setCausality(causality_t::INPUT)
        .setDescription("True magnetic field, body Z [uT]");

    register_real("temperature_c", &temperature_c_)
        .setCausality(causality_t::INPUT)
        .setDescription("True die temperature [degrees C]; the part reports it at 0.8 C resolution");
  }

  /// Brings the simulated part up on its bus. Deliberately not done in the
  /// constructor: the build-time modelDescription.xml generator instantiates
  /// the model purely to enumerate its variables, and that must not create
  /// shared-memory objects or spawn threads.
  void exit_initialisation_mode() override
  {
    if (!endpoint_.attach())
    {
      throw fmu4cpp::fatal_error("[hemerion_mmc5983ma_fmu] Unable to create the I2C bus '" + endpoint_.bus_name() +
                                 "' (already in use by another run?)");
    }
    debugLog(fmiOK, "[hemerion_mmc5983ma_fmu] I2C peripheral ready on bus '" + endpoint_.bus_name() + "'");
  }

  /// Powers the part down. The data registers hold at most one measurement,
  /// so unlike the FIFO-buffering IMU there is nothing to let a controller
  /// drain: whatever is unread at power-down is lost, as on the real board.
  void terminate() override { endpoint_.detach(); }

  /// fmi2Reset equivalent. The hard-iron and bridge offsets
  /// Mmc5983maMeasurementModel drew at construction are kept: they model this
  /// instance's physical part and its installation, which a reset does not
  /// swap out.
  void reset() override
  {
    truth_x_ut_ = 0.0;
    truth_y_ut_ = 0.0;
    truth_z_ut_ = 0.0;
    temperature_c_ = kDefaultTemperatureC;
    time_into_period_s_ = 0.0;
    slave_.reset();
    endpoint_.detach();
  }

protected:
  bool do_step(double dt) override
  {
    if (!endpoint_.attached())
    {
      throw fmu4cpp::fatal_error("[hemerion_mmc5983ma_fmu] Stepped before initialisation mode was exited");
    }

    // Triggered measurements first, so a TM_M raised before this step is
    // answered by it rather than a step later -- the driver's blocking
    // bring-up paths poll on exactly this.
    if (slave_.take_triggered_measurement())
    {
      latch_field();
    }
    if (slave_.take_triggered_temperature())
    {
      slave_.latch_temperature(measurement_model_.measure_temperature(temperature_c_));
    }

    // Continuous mode: the part free-runs at the period its Internal control
    // 2 register selects. Whole periods elapsed within this step each latch a
    // fresh measurement (truth zero-order-held, noise redrawn), and the
    // remainder carries into the next step so a 50 Hz part stepped at 10 Hz
    // really measures at 50 Hz. The data registers are not a FIFO -- a
    // controller polling slower than the rate observes only the newest.
    const std::uint64_t period_us = slave_.sampling_period_us();
    if (period_us == 0)
    {
      time_into_period_s_ = 0.0;  // one-shot: the measurement engine is idle
    }
    else
    {
      const double period_s = static_cast<double>(period_us) * 1e-6;
      double remaining_s = dt;
      while (time_into_period_s_ + remaining_s >= period_s)
      {
        remaining_s -= period_s - time_into_period_s_;
        time_into_period_s_ = 0.0;
        latch_field();
      }
      time_into_period_s_ += remaining_s;
    }

    // The INT line is a level; re-drive it so a clear-on-write inside the
    // step reaches the bus line here.
    endpoint_.publish_interrupt(slave_.interrupt_asserted());
    return true;
  }

private:
  // The sensing state is read from the device model at the moment of the
  // measurement, not cached: SET and RESET land as ordinary register writes
  // from the bus service thread, and a measurement taken after a RESET must
  // come back negated -- that is the behaviour the whole part turns on.
  void latch_field()
  {
    const Mmc5983maSensingState sensing = slave_.sensing_state();
    slave_.latch_measurement(measurement_model_.measure(truth_x_ut_, truth_y_ut_, truth_z_ut_, sensing));
  }

  Mmc5983maMeasurementModel measurement_model_;
  Mmc5983maI2cSlave slave_;
  sim::i2c_shm::I2cPeripheralEndpoint<Mmc5983maI2cSlave> endpoint_{ slave_, kI2cBus };
  double truth_x_ut_ = 0.0;
  double truth_y_ut_ = 0.0;
  double truth_z_ut_ = 0.0;
  double temperature_c_ = kDefaultTemperatureC;
  double time_into_period_s_ = 0.0;
};

}  // namespace hemerion::sensors::mag::mmc5983ma::fmu

fmu4cpp::model_info fmu4cpp::get_model_info()
{
  model_info info;
  info.modelName = "HemerionMmc5983maSimulator";
  info.author = "Onur Tuncer, Istanbul Technical University";
  info.description = "Register-accurate MEMSIC MMC5983MA magnetometer hardware simulator on a simulated I2C bus "
                     "for SWIL/HIL co-simulation";
  // Names such as b_x_ut carry no FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maSimulatorFmu);
