// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief BMP390 barometer hardware simulator, exported as an FMI
/// Co-Simulation FMU.
///
/// This FMU carries no sensor data on FMI output variables -- its effect is
/// an **I2C side channel**. Each step maps the current truth altitude through
/// the ICAO Standard Atmosphere and the part's error model to raw 24-bit
/// conversion words (Bmp390MeasurementModel, which numerically inverts the
/// real Bosch compensation), and latches them into the simulated part's data
/// registers (Bmp390I2cSlave). A controller -- a host flight computer
/// process, or emulated firmware under Renode once an I2C bridge exists --
/// then reads the part the way real firmware does: probe CHIP_ID, soft-reset,
/// read the calibration NVM, program OSR/ODR/INT_CTRL/PWR_CTRL, poll STATUS,
/// burst the shadowed data block. The words it recovers compensate back to
/// the noisy truth through the unmodified on-target Bmp390Driver +
/// Bmp390Compensator, exactly as bytes from physical silicon would.
///
/// What the FMI outputs *do* carry is a diagnostic the silicon cannot offer:
/// `conversions` counts every conversion latched since initialisation, and
/// `conversions_out_of_rating` counts those produced with the ambient
/// pressure or die temperature outside the part's rated envelope
/// (300--1250 hPa, -40..+85 C). A real BMP390 taken out of rating keeps
/// converting plausible-looking words with no accuracy guarantee, and so
/// does this one -- nothing on the I2C side changes -- but the bench can now
/// say it happened. NASA check-case 12 (examples/f16_trim_ecos, Mach 2 at
/// 30 013 ft) motivated the pair: most of that flight sits below the rated
/// pressure floor with the die at -46 C, and every conversion still reads
/// plausibly. Each envelope crossing is also debug-logged once, with the
/// offending value -- edges, not levels, so a 200 s excursion is two log
/// lines rather than ten thousand.
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
/// **When conversions start, in simulation time.** A controller is a separate
/// process on the wall clock, so if it configured the part while the master
/// was already stepping, the step its PWR_CTRL write landed in -- and with it
/// the conversion phase and which plant state each conversion sampled --
/// would depend on wall-clock timing, not on the seed. `bring_up_timeout_s`
/// holds the first step (PartBringUpGate, part_bring_up.h) until the part is
/// measuring, so the bring-up completes at t = 0 and the conversions fall on
/// a grid from there. `ready_at_start` says whether that happened. 0 (the
/// default) does not hold: the part behaves as it always has.
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

#include "Hemerion/part_bring_up.h"
#include "Hemerion/sensor_clock.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"

#include <hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h>

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <chrono>
#include <cstdint>
#include <random>
#include <cstdio>
#include <thread>

namespace hemerion::sensors::baro::bmp390::fmu
{

namespace
{

/// The error model's own defaults, so modelDescription.xml's start values and the model cannot
/// drift apart: both read this.
constexpr Bmp390MeasurementConfig kDefaultNoise{};

using fmu4cpp::causality_t;
using fmu4cpp::initial_t;
using fmu4cpp::variability_t;

/// Where this part sits: the bus it creates, and the environment variable a
/// launch script can retarget it with.
const sim::i2c_shm::I2cPeripheralConfig kI2cBus{ "hemerion_bmp390_i2c", "HEMERION_BMP390_FMU_I2C_BUS" };

/// How long terminate() keeps the part powered for a controller to read the
/// last conversion. A polling controller comes back within milliseconds.
constexpr auto kDrainTimeout = std::chrono::seconds(1);

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
        .setDescription("True geometric altitude above mean sea level [m]; ignored once p_Pa is written");

    // The ambient the die is actually in, for a plant that integrates its own
    // atmosphere. Writing p_Pa switches the part off the ISA-from-altitude
    // path for the rest of the run: on a non-standard day the two disagree by
    // hundreds of feet of indicated altitude, and that disagreement is the
    // whole reason a barometer is fused with GNSS rather than trusted. A
    // master that connects neither gets the standard day, as before.
    //
    // T_degC is separate and optional: a master with pressure but no air
    // temperature still gets the ISA temperature for the altitude, which is
    // the better of the two available wrongs -- the pressure channel is what
    // the altimeter reads, and the temperature channel only trims the
    // compensation polynomial.
    const auto onAmbientWritten = [this] { ambient_pressure_set_ = true; };
    register_real("p_Pa", &ambient_pressure_pa_, onAmbientWritten)
        .setCausality(causality_t::INPUT)
        .setDescription("True static pressure at the part [Pa]; once written, supersedes h_m");
    register_real("T_degC", &ambient_temperature_c_, [this] { ambient_temperature_set_ = true; })
        .setCausality(causality_t::INPUT)
        .setDescription("True die temperature [degrees Celsius]; defaults to the ISA temperature for h_m");

    // Diagnostics, not sensor data -- the sample stream stays on the I2C
    // bus. DISCRETE because an FMI 2.0 Integer must not claim continuity;
    // CALCULATED because the counts exist only once stepping does.
    register_integer("conversions", &conversions_)
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("Conversions latched since initialisation");
    register_integer("conversions_out_of_rating", &conversions_out_of_rating_)
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("Conversions produced with ambient pressure or die temperature outside the part's rated "
                        "envelope (300-1250 hPa, -40..+85 degC); the words themselves stay plausible, as on the "
                        "real part");
    // The error model, as fixed parameters: a part's noise does not change in flight, so these are read
    // once in exit_initialisation_mode() and the model is rebuilt from them. Start values are the model's
    // own defaults, so a master that sets none of them gets the part this FMU has always been.
    register_integer("seed", &seed_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Error-model RNG seed; 0 draws a nondeterministic one, any other value makes the "
                        "run reproducible");
    register_real("clock_skew_sigma_ppm", &clock_skew_sigma_ppm_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Oscillator rate error, 1-sigma, drawn once per run [ppm]. It accumulates, so it slides this "
                        "part's stream against the other sensors' over a long flight, which is what a filter fusing "
                        "them has to be robust to");
    register_real("clock_jitter_sigma_s", &clock_jitter_sigma_s_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Per-sample timestamp jitter, 1-sigma [s]. No memory, so unlike skew it does not accumulate: "
                        "it is the gap between when a sample was taken and when the part says it was");
    register_real("pressure_noise_pa", &pressure_noise_pa_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Pressure white noise, 1-sigma [Pa]");
    register_real("temperature_noise_c", &temperature_noise_c_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Die-temperature white noise, 1-sigma [degrees C]");
    register_real("pressure_bias_sigma_pa", &pressure_bias_sigma_pa_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Turn-on pressure bias 1-sigma, drawn once per run [Pa]");
    register_real("temperature_bias_sigma_c", &temperature_bias_sigma_c_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Turn-on die-temperature bias 1-sigma, drawn once per run [degrees C]");

    // The biases this instance actually drew -- the realisations of the two
    // sigmas above -- so a filter's barometer-bias state can be scored against
    // what the part did. Diagnostics like the counters: nothing on the I2C
    // side changes, and reading them draws nothing. Fixed for the run, hence
    // DISCRETE; set when initialisation mode is exited. Registered last so
    // every earlier value reference keeps its number.
    register_real("pressure_bias_pa", [this] { return static_cast<double>(measurement_model_.pressure_bias_pa()); })
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("Turn-on pressure bias this part was drawn with [Pa], added to the ambient pressure before "
                        "conversion");
    register_real("temperature_bias_c", [this] { return static_cast<double>(measurement_model_.temperature_bias_c()); })
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("Turn-on die-temperature bias this part was drawn with [degrees C]");

    // When simulation time may start: see the file comment. Registered after every earlier variable so their
    // value references keep their numbers.
    register_real("bring_up_timeout_s", &bring_up_timeout_s_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Wall-clock time the first step waits for a controller to put the part into normal or "
                        "forced mode, so the bring-up completes at t = 0 and the conversion schedule depends on "
                        "simulation time and the seed only [s]; 0 does not wait");
    register_integer("ready_at_start", &ready_at_start_)
        .setCausality(causality_t::OUTPUT)
        .setVariability(variability_t::DISCRETE)
        .setInitial(initial_t::CALCULATED)
        .setDescription("1 if a controller had the part measuring when simulation time started, else 0");
  }

  /// Brings the simulated part up on its bus. Deliberately not done in the
  /// constructor: the build-time modelDescription.xml generator instantiates
  /// the model purely to enumerate its variables, and that must not create
  /// shared-memory objects or spawn threads.
  void exit_initialisation_mode() override
  {
    apply_noise_config();
    gate_.arm(bring_up_timeout_s_);
    schedule_.reset();
    if (!endpoint_.attach())
    {
      throw fmu4cpp::fatal_error("[hemerion_bmp390_fmu] Unable to create the I2C bus '" + endpoint_.bus_name() +
                                 "' (already in use by another run?)");
    }
    debugLog(fmiOK, "[hemerion_bmp390_fmu] I2C peripheral ready on bus '" + endpoint_.bus_name() + "'");
  }

  /// Powers the part down -- once a controller still on the bus has read the
  /// last conversion, so the sample stream ends where the simulation does
  /// rather than wherever the controller's last poll landed. Bounded, and
  /// abandoned immediately if nobody is left on the bus.
  void terminate() override
  {
    const auto deadline = std::chrono::steady_clock::now() + kDrainTimeout;
    while (slave_.conversion_unread() && endpoint_.controller_attached() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    endpoint_.detach();
  }

  /// fmi2Reset equivalent. The turn-on biases Bmp390MeasurementModel drew at
  /// construction are kept: they model this instance's physical part, which
  /// a reset does not swap out.
  void reset() override
  {
    seed_ = 0;
    clock_skew_sigma_ppm_ = 0.0;
    clock_jitter_sigma_s_ = 0.0;
    pressure_noise_pa_ = kDefaultNoise.pressure_noise_pa;
    temperature_noise_c_ = kDefaultNoise.temperature_noise_c;
    pressure_bias_sigma_pa_ = kDefaultNoise.pressure_bias_sigma_pa;
    temperature_bias_sigma_c_ = kDefaultNoise.temperature_bias_sigma_c;
    bring_up_timeout_s_ = 0.0;
    ready_at_start_ = 0;
    altitude_m_ = 0.0;
    ambient_pressure_pa_ = 0.0;
    ambient_temperature_c_ = 0.0;
    ambient_pressure_set_ = false;
    ambient_temperature_set_ = false;
    schedule_.reset();
    conversions_ = 0;
    conversions_out_of_rating_ = 0;
    pressure_was_out_ = false;
    temperature_was_out_ = false;
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

    // The first step holds until the controller has the part measuring, so
    // its PWR_CTRL write takes effect at t = 0 rather than at whichever step
    // the wall clock delivered it in.
    if (gate_.outcome() == PartBringUpGate::Outcome::kPending)
    {
      gate_.hold([this] { return slave_.measuring(); });
      ready_at_start_ = gate_.ready_at_start() ? 1 : 0;
      if (gate_.outcome() == PartBringUpGate::Outcome::kTimedOut)
      {
        debugLog(fmiWarning,
                 "[hemerion_bmp390_fmu] No controller put the part into a measuring mode within bring_up_timeout_s; "
                 "simulation time starts anyway, and the conversion phase now depends on when it does");
      }
    }

    // One conversion per forced-mode trigger, regardless of the ODR.
    if (slave_.take_forced_conversion())
    {
      latch_one(currentTime());
    }

    // Normal mode: the part free-runs at the period its ODR register
    // selects (none in sleep). Each conversion is a fresh one (truth
    // zero-order-held, noise redrawn) stamped with its own conversion time.
    // The data registers are not a FIFO -- a controller that polls slower
    // than the ODR observes only the newest conversion, and SENSORTIME is how
    // it still knows when that conversion happened.
    schedule_.advance(
        dt, slave_.sampling_period_us(), [this](double offset_s) { latch_one(currentTime() + offset_s); });

    // The INT line is a level; re-drive it so clear-on-read inside the step
    // reaches the bus line here.
    endpoint_.publish_interrupt(slave_.interrupt_asserted());
    return true;
  }

private:
  void latch_one(double sample_time_s)
  {
    // Driven by the plant's own air when it publishes it, by the ISA
    // otherwise -- see the p_Pa registration.
    const Bmp390MeasurementModel::Conversion conversion =
        ambient_pressure_set_ ?
            measurement_model_.measure_ambient(ambient_pressure_pa_,
                                               ambient_temperature_set_ ?
                                                   ambient_temperature_c_ :
                                                   baro::fmu::BaroNoiseModel::isa_temperature_c(altitude_m_)) :
            measurement_model_.measure(altitude_m_);
    // SENSORTIME is a counter on the die, so it is the part's clock that
    // drives it -- the same stamp the other parts skew, under the name the
    // datasheet gives it.
    slave_.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp, clock_.stamp(sample_time_s, clock_rng_));

    ++conversions_;
    if (!conversion.pressure_in_rating || !conversion.temperature_in_rating)
    {
      ++conversions_out_of_rating_;
    }
    log_rating_edges(conversion);
  }

  /// Logs each rated-envelope crossing once -- edges, not levels, so a
  /// 200 s excursion is two lines rather than ten thousand. The values
  /// quoted are the *compensated* readings of the words just latched (what
  /// the driver will recover), so the log agrees with the bus to
  /// quantization error.
  void log_rating_edges(const Bmp390MeasurementModel::Conversion& conversion)
  {
    char message[160];
    if (conversion.pressure_in_rating == pressure_was_out_)  // i.e. the state flipped
    {
      pressure_was_out_ = !conversion.pressure_in_rating;
      const double t_lin = measurement_model_.compensator().compensate_temperature(conversion.uncomp_temp);
      const double pressure_hpa =
          measurement_model_.compensator().compensate_pressure(conversion.uncomp_press, t_lin) / 100.0;
      std::snprintf(message,
                    sizeof(message),
                    "[hemerion_bmp390_fmu] ambient pressure %.1f hPa is %s the part's rated 300-1250 hPa envelope",
                    pressure_hpa,
                    pressure_was_out_ ? "outside" : "back inside");
      debugLog(pressure_was_out_ ? fmiWarning : fmiOK, message);
    }
    if (conversion.temperature_in_rating == temperature_was_out_)
    {
      temperature_was_out_ = !conversion.temperature_in_rating;
      const double temperature_c = measurement_model_.compensator().compensate_temperature(conversion.uncomp_temp);
      std::snprintf(message,
                    sizeof(message),
                    "[hemerion_bmp390_fmu] die temperature %.1f degC is %s the part's rated -40..+85 degC envelope",
                    temperature_c,
                    temperature_was_out_ ? "outside" : "back inside");
      debugLog(temperature_was_out_ ? fmiWarning : fmiOK, message);
    }
  }

  /// Rebuilds the error model from the fixed parameters, once per initialisation, so a seeded run is
  /// reproducible from its first step and a reset-and-reinitialise repeats it.
  void apply_noise_config()
  {
    Bmp390MeasurementConfig config;
    config.pressure_noise_pa = static_cast<float>(pressure_noise_pa_);
    config.temperature_noise_c = static_cast<float>(temperature_noise_c_);
    config.pressure_bias_sigma_pa = static_cast<float>(pressure_bias_sigma_pa_);
    config.temperature_bias_sigma_c = static_cast<float>(temperature_bias_sigma_c_);
    config.calibration = measurement_model_.calibration();
    measurement_model_ = (seed_ == 0) ? Bmp390MeasurementModel(config) :
                                        Bmp390MeasurementModel(config, static_cast<std::uint64_t>(seed_));
    // The part's own clock. Seeded from the same number so one seed
    // determines the whole instrument, but from its own stream, so turning
    // the clock on does not move the error draws.
    clock_rng_.seed((seed_ == 0) ? std::random_device{}() : static_cast<std::uint64_t>(seed_) ^ 0x5DEECE66DULL);
    clock_ = SensorClock(
        SensorClockConfig{ static_cast<float>(clock_skew_sigma_ppm_), static_cast<float>(clock_jitter_sigma_s_) },
        clock_rng_);
  }

  // Error-model parameters, FMI-typed and narrowed once in apply_noise_config().
  int seed_ = 0;
  double clock_skew_sigma_ppm_ = 0.0;
  double clock_jitter_sigma_s_ = 0.0;
  double pressure_noise_pa_ = kDefaultNoise.pressure_noise_pa;
  double temperature_noise_c_ = kDefaultNoise.temperature_noise_c;
  double pressure_bias_sigma_pa_ = kDefaultNoise.pressure_bias_sigma_pa;
  double temperature_bias_sigma_c_ = kDefaultNoise.temperature_bias_sigma_c;
  double bring_up_timeout_s_ = 0.0;
  int ready_at_start_ = 0;
  PartBringUpGate gate_;
  FreeRunningSchedule schedule_;

  std::mt19937_64 clock_rng_{ 1 };
  SensorClock clock_{ SensorClockConfig{}, clock_rng_ };
  Bmp390MeasurementModel measurement_model_;
  Bmp390I2cSlave slave_{ measurement_model_.calibration() };
  sim::i2c_shm::I2cPeripheralEndpoint<Bmp390I2cSlave> endpoint_{ slave_, kI2cBus };
  double altitude_m_ = 0.0;

  // The plant's own air, when it publishes it. The two "written" flags latch
  // on the first write and stay set: a connected input is a property of the
  // wiring, not of the value, and a plant that legitimately publishes 0 Pa
  // would otherwise fall back to the ISA on that step alone.
  double ambient_pressure_pa_ = 0.0;
  double ambient_temperature_c_ = 0.0;
  bool ambient_pressure_set_ = false;
  bool ambient_temperature_set_ = false;
  int conversions_ = 0;
  int conversions_out_of_rating_ = 0;
  bool pressure_was_out_ = false;
  bool temperature_was_out_ = false;
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
  // None of the variable names carries an FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::baro::bmp390::fmu::Bmp390SimulatorFmu);
/// @endcond
