// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_mmc5983ma.cpp
//
// The MMC5983MA's I2C path with both ends against each other: the simulator's
// register/measurement model (mag/mmc5983ma/fmu/) driven by the real,
// on-target Mmc5983maDriver, with no transport in the picture -- the test's
// bus adapter delivers the driver's transactions to the device model as the
// bus events the shm peripheral endpoint would. This is the test that proves
// the simulated part is register- and math-compatible with the firmware
// driver, rather than just internally consistent with itself.
//
// The part that earns the extra machinery here over test_bmp390.cpp: this
// device measures on command, so the test bus has to *step* the part while
// the driver blocks, exactly as the FMU's do_step() does. That is what
// SimulatedPart::step() is, and driving it from delay_ms() is what lets the
// real driver's polling bring-up run unmodified.
//
// Plain asserts + exit code, matching test_bmp390.cpp -- Unity is not yet
// vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#include <crtdbg.h>
#endif

#include "Hemerion/mag/mag_types.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_registers.h"

using hemerion::sensors::mag::MagSample;
using hemerion::sensors::mag::mmc5983ma::decode_mmc5983ma_field;
using hemerion::sensors::mag::mmc5983ma::encode_mmc5983ma_field;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maControl0Reset;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maI2cAddress;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maLsbPerMicrotesla;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maBandwidth;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maConfig;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maContinuousRate;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maDriver;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maError;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maFieldCounts;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maI2cBus;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maReadResult;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maRegister;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maI2cSlave;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementConfig;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// One quantization step of the part's fixed scale [uT], the floor on any
// round-trip comparison here.
constexpr double kQuantumUt = 1.0 / static_cast<double>(kMmc5983maLsbPerMicrotesla);

// A truth field roughly the size and shape of Istanbul's: ~48 uT total,
// steeply inclined.
constexpr double kTruthXUt = 22.0;
constexpr double kTruthYUt = -6.0;
constexpr double kTruthZUt = 41.0;

// The simulated part: device model, measurement model, and the truth state
// feeding them -- i.e. the FMU minus FMI and minus the shared-memory bus.
// step() is a transcription of the FMU's do_step() triggered-measurement
// half; latch_continuous() stands in for its free-running half, called by the
// test wherever the part's own clock would have fired.
class SimulatedPart
{
public:
  explicit SimulatedPart(const Mmc5983maMeasurementConfig& config = {},
                         std::uint64_t seed = 42,
                         std::uint8_t address = kMmc5983maI2cAddress)
    : model_(config, seed), slave_(address)
  {
  }

  void set_truth(double x_ut, double y_ut, double z_ut)
  {
    truth_x_ut_ = x_ut;
    truth_y_ut_ = y_ut;
    truth_z_ut_ = z_ut;
  }

  void set_temperature(double temperature_c) { temperature_c_ = temperature_c; }

  // Serve whatever the controller has commanded since the last step.
  void step()
  {
    if (slave_.take_triggered_measurement())
    {
      latch_continuous();
    }
    if (slave_.take_triggered_temperature())
    {
      slave_.latch_temperature(model_.measure_temperature(temperature_c_));
    }
  }

  // One free-running measurement, as continuous mode would produce.
  void latch_continuous()
  {
    slave_.latch_measurement(model_.measure(truth_x_ut_, truth_y_ut_, truth_z_ut_, slave_.sensing_state()));
  }

  [[nodiscard]] Mmc5983maI2cSlave& slave() { return slave_; }
  [[nodiscard]] const Mmc5983maMeasurementModel& model() const { return model_; }

private:
  Mmc5983maMeasurementModel model_;
  Mmc5983maI2cSlave slave_;
  double truth_x_ut_ = 0.0;
  double truth_y_ut_ = 0.0;
  double truth_z_ut_ = 0.0;
  double temperature_c_ = 25.0;
};

// Delivers the driver's transactions to the device model as the bus events
// the shm peripheral endpoint would: the write phase sets the pointer (and
// walks forward from it), the read phase runs under a repeated START.
// `probe_address` is what the *board* says the part is at -- pointing it
// somewhere else is how the address-NACK path is exercised.
//
// delay_ms() steps the part rather than sleeping: under co-simulation the
// FMI master's do_step() is what advances it, and the driver's blocking
// bring-up cannot complete unless something does.
class DirectBus final : public Mmc5983maI2cBus
{
public:
  explicit DirectBus(SimulatedPart& part, std::uint8_t probe_address = kMmc5983maI2cAddress)
    : part_(part), probe_address_(probe_address)
  {
  }

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    Mmc5983maI2cSlave& slave = part_.slave();
    const bool acked = slave.start(probe_address_, false) && slave.write(reg) && slave.write(value);
    slave.stop();
    return acked;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    Mmc5983maI2cSlave& slave = part_.slave();
    bool acked = slave.start(probe_address_, false) && slave.write(reg) && slave.start(probe_address_, true);
    if (acked)
    {
      for (std::size_t i = 0; i < count; ++i)
      {
        out[i] = slave.read(i + 1 == count);
      }
    }
    slave.stop();
    return acked;
  }

  void delay_ms(std::uint32_t milliseconds) override
  {
    now_us_ += static_cast<std::uint64_t>(milliseconds) * 1000ULL;
    part_.step();
  }

  [[nodiscard]] std::uint64_t now_us() const override { return now_us_; }

  [[nodiscard]] bool interrupt_line() const override { return part_.slave().interrupt_asserted(); }

private:
  SimulatedPart& part_;
  std::uint8_t probe_address_;
  std::uint64_t now_us_ = 0;
};

// The bit packing in mmc5983ma_registers.h is the one thing both ends depend
// on byte-for-byte, so check it standalone before anything built on it.
void test_field_encoding_round_trips()
{
  for (const Mmc5983maFieldCounts counts : { Mmc5983maFieldCounts{ 0, 0, 0 },
                                             Mmc5983maFieldCounts{ 262143, 262143, 262143 },
                                             Mmc5983maFieldCounts{ 131072, 131072, 131072 },
                                             Mmc5983maFieldCounts{ 1, 2, 3 },
                                             Mmc5983maFieldCounts{ 200001, 99999, 131073 } })
  {
    const auto burst = encode_mmc5983ma_field(counts);
    const Mmc5983maFieldCounts decoded = decode_mmc5983ma_field(burst);
    assert(decoded.x == counts.x);
    assert(decoded.y == counts.y);
    assert(decoded.z == counts.z);
    // XYZout2's low two bits read zero on the part.
    assert((burst[6] & 0x03U) == 0U);
  }
}

// probe() must identify the part, calibrate the bridge offset through the
// real register interface, and leave it free-running at the configured rate.
void test_probe_identifies_and_configures()
{
  SimulatedPart part;
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  assert(part.slave().sampling_period_us() == 0);  // power-on: one-shot
  assert(driver.probe() == Mmc5983maError::kNone);
  assert(part.slave().sampling_period_us() == 20000);  // the default 50 Hz
}

// A driver pointed at an address nothing answers must see the bus, not a
// bogus identity: the very first transaction fails.
void test_probe_wrong_address_fails()
{
  SimulatedPart part;
  DirectBus bus(part, static_cast<std::uint8_t>(kMmc5983maI2cAddress + 1));
  Mmc5983maDriver driver(bus);

  assert(driver.probe() == Mmc5983maError::kTransferFailed);
}

// The SET/RESET pair must recover the bridge offset the simulated part was
// born with -- the whole reason for choosing this part. Noiseless, so the
// only slack is the rounding of two 18-bit words and one integer halving.
void test_calibrate_offset_recovers_bridge_offset()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;
  // Leave bridge_offset_sigma_ut at its default: a real, large offset is
  // exactly what this must dig out.

  SimulatedPart part(config, /*seed=*/7);
  // A field standing during calibration must cancel in the pair; calibrating
  // in a shielded chamber is not a precondition, and the test says so.
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  assert(driver.probe() == Mmc5983maError::kNone);

  const auto& truth_offset = part.model().bridge_offset();
  const auto& measured = driver.bridge_offset();
  assert(std::abs(measured.x - truth_offset.x) <= 1);
  assert(std::abs(measured.y - truth_offset.y) <= 1);
  assert(std::abs(measured.z - truth_offset.z) <= 1);
}

// A noiseless part must round-trip: truth field through the error model, the
// register file, the real driver and convert_raw_to_si(), back to the truth
// -- to quantization plus the one-count slack in the calibrated offset.
void test_noiseless_sample_round_trips()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;

  SimulatedPart part(config, /*seed=*/11);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);

  for (const double scale : { 1.0, -1.0, 0.5 })
  {
    part.set_truth(kTruthXUt * scale, kTruthYUt * scale, kTruthZUt * scale);
    part.latch_continuous();

    MagSample sample;
    assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
    assert(near(sample.mag_x_ut, kTruthXUt * scale, 2.0 * kQuantumUt));
    assert(near(sample.mag_y_ut, kTruthYUt * scale, 2.0 * kQuantumUt));
    assert(near(sample.mag_z_ut, kTruthZUt * scale, 2.0 * kQuantumUt));
  }
}

// The headline failure this part and this simulator exist to catch: a driver
// that never runs the SET/RESET pair reads the bridge offset as if it were
// field, and the error is the size of Earth's field rather than a rounding
// error. Asserted against the model's truth offset so it is exact, not
// seed-lucky.
void test_uncalibrated_driver_reads_the_bridge_offset()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;

  SimulatedPart part(config, /*seed=*/23);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  Mmc5983maConfig driver_config;
  driver_config.calibrate_offset_on_probe = false;
  assert(driver.probe(driver_config) == Mmc5983maError::kNone);
  assert(driver.bridge_offset().x == 0);  // nothing calibrated

  part.latch_continuous();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);

  const auto& truth_offset = part.model().bridge_offset();
  const double offset_x_ut = static_cast<double>(truth_offset.x) / kMmc5983maLsbPerMicrotesla;
  const double offset_y_ut = static_cast<double>(truth_offset.y) / kMmc5983maLsbPerMicrotesla;
  const double offset_z_ut = static_cast<double>(truth_offset.z) / kMmc5983maLsbPerMicrotesla;

  // Exactly wrong by the uncancelled offset.
  assert(near(sample.mag_x_ut, kTruthXUt + offset_x_ut, 2.0 * kQuantumUt));
  assert(near(sample.mag_y_ut, kTruthYUt + offset_y_ut, 2.0 * kQuantumUt));
  assert(near(sample.mag_z_ut, kTruthZUt + offset_z_ut, 2.0 * kQuantumUt));

  // And wrong by an amount no heading algorithm survives: the default
  // sigma is 16.7 uT against a ~48 uT field.
  const double error =
      std::sqrt((offset_x_ut * offset_x_ut) + (offset_y_ut * offset_y_ut) + (offset_z_ut * offset_z_ut));
  assert(error > 1.0);
}

// Leaving the part RESET is the subtle version of the same bug: the offset
// is still cancelled and the field is still the right magnitude, but every
// axis comes back negated. The magnetization state has to be real for this
// to fail, which is why the device model carries it.
void test_reset_polarity_negates_the_field()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;

  SimulatedPart part(config, /*seed=*/29);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);

  // Behind the driver's back, as a stray RESET (or a bring-up that ended on
  // one) would leave it.
  assert(bus.write_register(static_cast<std::uint8_t>(Mmc5983maRegister::kInternalControl0), kMmc5983maControl0Reset));
  assert(part.slave().sensing_state().magnetization == -1);

  part.latch_continuous();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
  assert(near(sample.mag_x_ut, -kTruthXUt, 2.0 * kQuantumUt));
  assert(near(sample.mag_y_ut, -kTruthYUt, 2.0 * kQuantumUt));
  assert(near(sample.mag_z_ut, -kTruthZUt, 2.0 * kQuantumUt));
}

// With Auto_SR_en the part cancels the offset itself, so an uncalibrated
// driver is accurate -- and a manual calibration on top must be *refused*,
// not merely skipped. Under that bit the part runs its own pair inside every
// measurement and returns the positive-polarity difference, so a manual
// SET/RESET pair reads the same value twice: the field does not cancel, and
// the "offset" would come out equal to the standing field. Storing that
// would null the magnetometer out entirely. This test is why the driver
// refuses.
void test_automatic_set_reset_cancels_in_hardware()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;

  SimulatedPart part(config, /*seed=*/31);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  Mmc5983maConfig driver_config;
  driver_config.automatic_set_reset = true;
  driver_config.calibrate_offset_on_probe = false;
  assert(driver.probe(driver_config) == Mmc5983maError::kNone);

  part.latch_continuous();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
  assert(near(sample.mag_x_ut, kTruthXUt, 2.0 * kQuantumUt));
  assert(near(sample.mag_z_ut, kTruthZUt, 2.0 * kQuantumUt));

  // The refusal, and -- more importantly -- that the refusal left nothing
  // behind: no bogus offset, and the next sample is still accurate.
  assert(driver.calibrate_offset() == Mmc5983maError::kInvalidConfiguration);
  assert(driver.bridge_offset().x == 0);
  assert(driver.bridge_offset().y == 0);
  assert(driver.bridge_offset().z == 0);

  part.latch_continuous();
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
  assert(near(sample.mag_y_ut, kTruthYUt, 2.0 * kQuantumUt));
}

// probe() must not carry a stale software offset into a hardware-cancelling
// configuration: the readings arrive already corrected, so subtracting one
// more would double-count it.
void test_automatic_set_reset_clears_a_stale_offset()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;

  SimulatedPart part(config, /*seed=*/41);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  // As if restored from non-volatile storage by an application that then
  // reconfigured the part into hardware cancellation.
  driver.set_bridge_offset({ 5000, -5000, 5000 });

  Mmc5983maConfig driver_config;
  driver_config.automatic_set_reset = true;
  assert(driver.probe(driver_config) == Mmc5983maError::kNone);
  assert(driver.bridge_offset().x == 0);

  part.latch_continuous();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
  assert(near(sample.mag_x_ut, kTruthXUt, 2.0 * kQuantumUt));
}

// Reading the data block acknowledges the measurement: until the next one
// lands, the driver sees no new data, exactly as Meas_M_Done behaves.
void test_read_consumes_measurement_done()
{
  SimulatedPart part;
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);

  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kNoNewData);  // probe's own read was acknowledged

  part.latch_continuous();
  assert(driver.data_ready());  // INT follows the flag, and probe enabled it
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);
  assert(!driver.data_ready());
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kNoNewData);
}

// The one-shot path, and the part's temperature channel with it.
void test_one_shot_and_temperature()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;
  config.temperature_noise_c = 0.0F;

  SimulatedPart part(config, /*seed=*/37);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  part.set_temperature(41.0);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  Mmc5983maConfig driver_config;
  driver_config.continuous_rate = Mmc5983maContinuousRate::kOff;
  assert(driver.probe(driver_config) == Mmc5983maError::kNone);
  assert(part.slave().sampling_period_us() == 0);  // never free-runs

  MagSample sample;
  assert(driver.measure_once(sample) == Mmc5983maError::kNone);
  assert(near(sample.mag_x_ut, kTruthXUt, 2.0 * kQuantumUt));
  assert(near(sample.mag_z_ut, kTruthZUt, 2.0 * kQuantumUt));

  float temperature_c = 0.0F;
  assert(driver.read_temperature(temperature_c) == Mmc5983maError::kNone);
  // 0.8 C per count is the whole resolution of this channel.
  assert(near(temperature_c, 41.0, 0.8));
}

// Internal control 0..3 are write-only on the datasheet, so they read back
// zero. A driver that read-modify-writes one of them would clobber it -- on
// this model exactly as on the part, which is why Mmc5983maDriver shadows
// control 0 instead.
void test_control_registers_read_back_zero()
{
  SimulatedPart part;
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);

  for (const Mmc5983maRegister reg : { Mmc5983maRegister::kInternalControl0,
                                       Mmc5983maRegister::kInternalControl1,
                                       Mmc5983maRegister::kInternalControl2,
                                       Mmc5983maRegister::kInternalControl3 })
  {
    std::uint8_t value = 0xFF;
    assert(bus.read_registers(static_cast<std::uint8_t>(reg), &value, 1));
    assert(value == 0);
  }

  // The behaviour those writes drove is real even though the bits are
  // invisible: probe() left the part free-running.
  assert(part.slave().sampling_period_us() == 20000);
}

// A rate the datasheet conditions on a bandwidth must be refused rather than
// programmed into a part that cannot sustain it.
void test_unreachable_rate_is_refused()
{
  SimulatedPart part;
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);

  Mmc5983maConfig driver_config;
  driver_config.bandwidth = Mmc5983maBandwidth::k100Hz;
  driver_config.continuous_rate = Mmc5983maContinuousRate::k1000Hz;  // needs the 0.5 ms bandwidth
  assert(driver.probe(driver_config) == Mmc5983maError::kInvalidConfiguration);

  driver_config.bandwidth = Mmc5983maBandwidth::k800Hz;
  assert(driver.probe(driver_config) == Mmc5983maError::kNone);
  assert(part.slave().sampling_period_us() == 1000);
}

// A software reset must drop the configuration back to power-on (one-shot,
// no measurements), which probe() then proves by bringing the part all the
// way back up.
void test_soft_reset_then_reprobe()
{
  SimulatedPart part;
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);
  assert(part.slave().sampling_period_us() == 20000);

  using hemerion::sensors::mag::mmc5983ma::kMmc5983maControl1SoftReset;
  assert(
      bus.write_register(static_cast<std::uint8_t>(Mmc5983maRegister::kInternalControl1), kMmc5983maControl1SoftReset));
  assert(part.slave().sampling_period_us() == 0);

  assert(driver.probe() == Mmc5983maError::kNone);
  assert(part.slave().sampling_period_us() == 20000);
}

// A noisy part must still land within a generous bound of the truth once
// calibrated -- a statistical check with a fixed seed so it isn't flaky.
void test_noisy_sample_stays_bounded()
{
  Mmc5983maMeasurementConfig config;  // defaults: bridge offset, hard iron and noise all on
  SimulatedPart part(config, /*seed=*/19);
  part.set_truth(kTruthXUt, kTruthYUt, kTruthZUt);
  DirectBus bus(part);
  Mmc5983maDriver driver(bus);
  assert(driver.probe() == Mmc5983maError::kNone);

  part.latch_continuous();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kSample);

  // The bridge offset is gone, but the hard iron is not -- no amount of
  // SET/RESET removes a real field, only a magnetic calibration does. Bound
  // by 5 sigma of that plus 5 sigma of noise.
  const double bound = 5.0 * static_cast<double>(config.hard_iron_sigma_ut + config.noise_ut) + kQuantumUt;
  assert(near(sample.mag_x_ut, kTruthXUt, bound));
  assert(near(sample.mag_y_ut, kTruthYUt, bound));
  assert(near(sample.mag_z_ut, kTruthZUt, bound));
}

// A failed assert() must kill the process, not park it.
//
// On Windows the CRT answers abort() with a modal "terminate in an unusual
// way" dialog and _CrtDbgReport pops another; in a CI job with no desktop
// those block until the job's own timeout, so a one-line assertion failure
// reads as a hung runner instead of a failed test. This routes both to
// stderr and makes abort() return an exit code immediately. No effect
// anywhere else.
//
// The sibling tests in this directory have the same exposure and no guard;
// this is worth lifting into shared test scaffolding rather than copying.
void fail_fast_instead_of_blocking()
{
#ifdef _WIN32
  _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
  for (const int report : { _CRT_WARN, _CRT_ERROR, _CRT_ASSERT })
  {
    (void)_CrtSetReportMode(report, _CRTDBG_MODE_FILE);
    (void)_CrtSetReportFile(report, _CRTDBG_FILE_STDERR);
  }
#endif
}

}  // namespace

int main()
{
  fail_fast_instead_of_blocking();

  test_field_encoding_round_trips();
  test_probe_identifies_and_configures();
  test_probe_wrong_address_fails();
  test_calibrate_offset_recovers_bridge_offset();
  test_noiseless_sample_round_trips();
  test_uncalibrated_driver_reads_the_bridge_offset();
  test_reset_polarity_negates_the_field();
  test_automatic_set_reset_cancels_in_hardware();
  test_automatic_set_reset_clears_a_stale_offset();
  test_read_consumes_measurement_done();
  test_one_shot_and_temperature();
  test_control_registers_read_back_zero();
  test_unreachable_rate_is_refused();
  test_soft_reset_then_reprobe();
  test_noisy_sample_stays_bounded();

  std::puts("test_mmc5983ma: all checks passed");
  return 0;
}
