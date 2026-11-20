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
/// (200 x 1 ms as configured), or bring-up times out against a part that is
/// merely paused.
///
/// **When measurements start, in simulation time.** The controller is a
/// separate process on the wall clock, and its bring-up blocks on this part:
/// the SET/RESET pair is two triggered measurements it polls for. Served by
/// whichever steps happened to be running, those measurements -- and the step
/// continuous mode began in, and so the phase and the plant state of every
/// later measurement -- depended on wall-clock timing rather than on the
/// seed. `bring_up_timeout_s` holds the first step (PartBringUpGate,
/// part_bring_up.h) until continuous mode is on, answering the bring-up's
/// triggered measurements from the t = 0 inputs while it holds, so the whole
/// bring-up completes at t = 0. `ready_at_start` says whether that happened.
/// 0 (the default) does not hold, and the constraint above then still
/// applies.
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

#include "Hemerion/part_bring_up.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"

#include <hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h>

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <chrono>
#include <cstdint>
#include <thread>

namespace hemerion::sensors::mag::mmc5983ma::fmu
{

namespace
{

/// The error model's own defaults, so modelDescription.xml's start values and the model cannot
/// drift apart: both read this.
constexpr Mmc5983maMeasurementConfig kDefaultNoise{};

using fmu4cpp::causality_t;
using fmu4cpp::initial_t;
using fmu4cpp::variability_t;

/// Where this part sits: the bus it creates, and the environment variable a
/// launch script can retarget it with.
const sim::i2c_shm::I2cPeripheralConfig kI2cBus{ "hemerion_mmc5983ma_i2c", "HEMERION_MMC5983MA_FMU_I2C_BUS" };

/// Die temperature the part reports when the host does not drive it.
constexpr double kDefaultTemperatureC = 25.0;

/// How long terminate() keeps the part powered for a controller to read the
/// last measurement. A polling controller comes back within milliseconds.
constexpr auto kDrainTimeout = std::chrono::seconds(1);

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
    // The error model, as fixed parameters: a part's noise does not change in flight, so these are read
    // once in exit_initialisation_mode() and the model is rebuilt from them. Start values are the model's
    // own defaults, so a master that sets none of them gets the part this FMU has always been.
    register_integer("seed", &seed_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Error-model RNG seed; 0 draws a nondeterministic one, any other value makes the "
                        "run reproducible");
    register_real("noise_ut", &noise_ut_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("White noise, 1-sigma per axis [uT]");
    register_real("hard_iron_sigma_ut", &hard_iron_sigma_ut_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Hard-iron offset 1-sigma per axis, drawn once per run [uT]; a real field the installation "
                        "adds, so SET/RESET does not remove it");
    register_real("bridge_offset_sigma_ut", &bridge_offset_sigma_ut_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Bridge offset 1-sigma per axis, drawn once per run [uT]; this is what a SET/RESET pair "
                        "removes");
    register_real("soft_iron_sigma", &soft_iron_sigma_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Soft-iron distortion, 1-sigma per matrix element, drawn once per run; unlike hard iron it "
                        "turns with the vehicle, which is what makes a real calibration an ellipsoid fit");
    register_real("temperature_noise_c", &temperature_noise_c_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Die-temperature white noise, 1-sigma [degrees C]");

    // When simulation time may start: see the file comment. Registered after every earlier variable so their
    // value references keep their numbers.
    register_real("bring_up_timeout_s", &bring_up_timeout_s_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Wall-clock time the first step waits for a controller to put the part into continuous "
                        "mode, answering its triggered measurements meanwhile, so the bring-up completes at t = 0 "
                        "and the measurement schedule depends on simulation time and the seed only [s]; 0 does "
                        "not wait");
    register_integer("ready_at_start", &ready_at_start_)
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("1 if a controller had the part in continuous mode when simulation time started, else 0");
  }

  /// Brings the simulated part up on its bus. Deliberately not done in the
  /// constructor: the build-time modelDescription.xml generator instantiates
  /// the model purely to enumerate its variables, and that must not create
  /// shared-memory objects or spawn threads.
  void exit_initialisation_mode() override
  {
    apply_noise_config();
    gate_.arm(bring_up_timeout_s_);
    soft_resets_seen_ = slave_.soft_resets();
    schedule_.reset();
    if (!endpoint_.attach())
    {
      throw fmu4cpp::fatal_error("[hemerion_mmc5983ma_fmu] Unable to create the I2C bus '" + endpoint_.bus_name() +
                                 "' (already in use by another run?)");
    }
    debugLog(fmiOK, "[hemerion_mmc5983ma_fmu] I2C peripheral ready on bus '" + endpoint_.bus_name() + "'");
  }

  /// Powers the part down -- once a controller still on the bus has read the
  /// last measurement, so the sample stream ends where the simulation does
  /// rather than wherever the controller's last poll landed. Bounded, and
  /// abandoned immediately if nobody is left on the bus.
  void terminate() override
  {
    const auto deadline = std::chrono::steady_clock::now() + kDrainTimeout;
    while (slave_.measurement_unread() && endpoint_.controller_attached() &&
           std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    endpoint_.detach();
  }

  /// fmi2Reset equivalent. The hard-iron and bridge offsets
  /// Mmc5983maMeasurementModel drew at construction are kept: they model this
  /// instance's physical part and its installation, which a reset does not
  /// swap out.
  void reset() override
  {
    seed_ = 0;
    noise_ut_ = kDefaultNoise.noise_ut;
    hard_iron_sigma_ut_ = kDefaultNoise.hard_iron_sigma_ut;
    bridge_offset_sigma_ut_ = kDefaultNoise.bridge_offset_sigma_ut;
    soft_iron_sigma_ = kDefaultNoise.soft_iron_sigma;
    temperature_noise_c_ = kDefaultNoise.temperature_noise_c;
    bring_up_timeout_s_ = 0.0;
    ready_at_start_ = 0;
    truth_x_ut_ = 0.0;
    truth_y_ut_ = 0.0;
    truth_z_ut_ = 0.0;
    temperature_c_ = kDefaultTemperatureC;
    schedule_.reset();
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

    // The first step holds until the controller has the part in continuous
    // mode, answering its bring-up's triggered measurements meanwhile from the
    // t = 0 inputs, so the bring-up takes effect at t = 0 rather than at
    // whichever steps the wall clock delivered it in.
    if (gate_.outcome() == PartBringUpGate::Outcome::kPending)
    {
      gate_.hold([this] { return slave_.sampling_period_us() != 0; }, [this] { serve_bring_up(); });
      ready_at_start_ = gate_.ready_at_start() ? 1 : 0;
      if (gate_.outcome() == PartBringUpGate::Outcome::kTimedOut)
      {
        debugLog(fmiWarning,
                 "[hemerion_mmc5983ma_fmu] No controller put the part into continuous mode within "
                 "bring_up_timeout_s; simulation time starts anyway, and the measurement phase now depends on when "
                 "it does");
      }
    }

    // Triggered measurements first, so a TM_M raised before this step is
    // answered by it rather than a step later -- the driver's blocking
    // bring-up paths poll on exactly this.
    serve_triggered();

    // Continuous mode: the part free-runs at the period its Internal control
    // 2 register selects (none while it is off). Each measurement is a fresh
    // one (truth zero-order-held, noise redrawn). The data registers are not
    // a FIFO -- a controller polling slower than the rate observes only the
    // newest.
    schedule_.advance(dt, slave_.sampling_period_us(), [this](double /*offset_s*/) { latch_field(); });

    // The INT line is a level; re-drive it so a clear-on-write inside the
    // step reaches the bus line here.
    endpoint_.publish_interrupt(slave_.interrupt_asserted());
    return true;
  }

private:
  /// The bring-up hold's service: answers the bring-up's triggered
  /// measurements from the t = 0 inputs.
  ///
  /// It also restarts the seeded error model at each software reset the
  /// controller sends before simulation time starts. A controller whose
  /// first bring-up attempt timed out (the master had not stepped yet) leaves
  /// a TM_M behind; served by the hold before the controller's retry resets
  /// the part, that orphan would cost one noise draw, and how many orphans
  /// there were depends on the wall clock. Every attempt begins with SW_RST,
  /// so rebuilding the model there -- same seed, so the same biases and the
  /// same stream -- makes the successful attempt's draws the stream's first,
  /// however many attempts preceded it. An unseeded part is left alone: a
  /// rebuild would draw a different part.
  void serve_bring_up()
  {
    const std::uint32_t resets = slave_.soft_resets();
    if (resets != soft_resets_seen_)
    {
      soft_resets_seen_ = resets;
      if (seed_ != 0)
      {
        apply_noise_config();
      }
    }
    serve_triggered();
  }

  /// Answers a pending TM_M or TM_T trigger with one measurement.
  void serve_triggered()
  {
    if (slave_.take_triggered_measurement())
    {
      latch_field();
    }
    if (slave_.take_triggered_temperature())
    {
      slave_.latch_temperature(measurement_model_.measure_temperature(temperature_c_));
    }
  }

  // The sensing state is read from the device model at the moment of the
  // measurement, not cached: SET and RESET land as ordinary register writes
  // from the bus service thread, and a measurement taken after a RESET must
  // come back negated -- that is the behaviour the whole part turns on.
  void latch_field()
  {
    const Mmc5983maSensingState sensing = slave_.sensing_state();
    slave_.latch_measurement(measurement_model_.measure(truth_x_ut_, truth_y_ut_, truth_z_ut_, sensing));
  }

  /// Rebuilds the error model from the fixed parameters, once per initialisation, so a seeded run is
  /// reproducible from its first step and a reset-and-reinitialise repeats it.
  void apply_noise_config()
  {
    Mmc5983maMeasurementConfig config;
    config.noise_ut = static_cast<float>(noise_ut_);
    config.hard_iron_sigma_ut = static_cast<float>(hard_iron_sigma_ut_);
    config.bridge_offset_sigma_ut = static_cast<float>(bridge_offset_sigma_ut_);
    config.soft_iron_sigma = static_cast<float>(soft_iron_sigma_);
    config.temperature_noise_c = static_cast<float>(temperature_noise_c_);
    measurement_model_ = (seed_ == 0) ? Mmc5983maMeasurementModel(config) :
                                        Mmc5983maMeasurementModel(config, static_cast<std::uint64_t>(seed_));
  }

  // Error-model parameters, FMI-typed and narrowed once in apply_noise_config().
  int seed_ = 0;
  double noise_ut_ = kDefaultNoise.noise_ut;
  double hard_iron_sigma_ut_ = kDefaultNoise.hard_iron_sigma_ut;
  double bridge_offset_sigma_ut_ = kDefaultNoise.bridge_offset_sigma_ut;
  double soft_iron_sigma_ = kDefaultNoise.soft_iron_sigma;
  double temperature_noise_c_ = kDefaultNoise.temperature_noise_c;
  double bring_up_timeout_s_ = 0.0;
  int ready_at_start_ = 0;
  std::uint32_t soft_resets_seen_ = 0;
  PartBringUpGate gate_;
  FreeRunningSchedule schedule_;

  Mmc5983maMeasurementModel measurement_model_;
  Mmc5983maI2cSlave slave_;
  sim::i2c_shm::I2cPeripheralEndpoint<Mmc5983maI2cSlave> endpoint_{ slave_, kI2cBus };
  double truth_x_ut_ = 0.0;
  double truth_y_ut_ = 0.0;
  double truth_z_ut_ = 0.0;
  double temperature_c_ = kDefaultTemperatureC;
};

}  // namespace hemerion::sensors::mag::mmc5983ma::fmu

/// @cond FMI_ENTRY_POINTS
/// fmu4cpp's C entry points. Excluded from the API reference: they are
/// the FMI 2.0/3.0 ABI the packaging layer requires, not Hemerion API,
/// and fmu4cpp's own headers are not part of the Doxygen input.
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
/// @endcond
