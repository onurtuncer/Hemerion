// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_bmp390.cpp
//
// The BMP390's I2C path with both ends against each other: the simulator's
// register/measurement model (baro/bmp390/fmu/) driven by the real, on-target
// Bmp390Driver + Bmp390Compensator, with no transport in the picture -- the
// test's bus adapter delivers the driver's transactions to the device model
// as the bus events the shm peripheral endpoint would. This is the test that
// proves the simulated part is register- and math-compatible with the
// firmware driver, rather than just internally consistent with itself.
// (sim/i2c_shm/test/test_i2c_shm_bmp390.cpp repeats the probe/sample cycle
// across the real shared-memory transport.)
//
// Plain asserts + exit code, matching test_imu_spi.cpp -- Unity is not yet
// vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>

#include "Hemerion/baro/baro_types.h"
#include "Hemerion/baro/bmp390/bmp390_driver.h"
#include "Hemerion/baro/bmp390/bmp390_registers.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"
#include "Hemerion/baro/fmu/baro_noise_model.h"

using hemerion::sensors::baro::BaroSample;
using hemerion::sensors::baro::bmp390::Bmp390Driver;
using hemerion::sensors::baro::bmp390::Bmp390Error;
using hemerion::sensors::baro::bmp390::Bmp390I2cBus;
using hemerion::sensors::baro::bmp390::Bmp390ReadResult;
using hemerion::sensors::baro::bmp390::kBmp390I2cAddressPrimary;
using hemerion::sensors::baro::bmp390::kBmp390I2cAddressSecondary;
using hemerion::sensors::baro::bmp390::fmu::Bmp390I2cSlave;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementConfig;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementModel;
using hemerion::sensors::baro::bmp390::fmu::kBmp390ReferenceCalibration;
using hemerion::sensors::baro::fmu::BaroNoiseModel;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Delivers the driver's transactions to the device model as the bus events
// the shm peripheral endpoint would: write phase sets the pointer (plus
// paired data), the read phase runs under a repeated START. `probe_address`
// is what the *board* says the part is strapped to -- pointing it away from
// the model's strap is how the address-NACK path is exercised.
class DirectBus final : public Bmp390I2cBus
{
public:
  explicit DirectBus(Bmp390I2cSlave& slave, std::uint8_t probe_address = kBmp390I2cAddressPrimary)
    : slave_(slave), probe_address_(probe_address)
  {
  }

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const bool acked = slave_.start(probe_address_, false) && slave_.write(reg) && slave_.write(value);
    slave_.stop();
    return acked;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    bool acked = slave_.start(probe_address_, false) && slave_.write(reg) && slave_.start(probe_address_, true);
    if (acked)
    {
      for (std::size_t i = 0; i < count; ++i)
      {
        out[i] = slave_.read(i + 1 == count);
      }
    }
    slave_.stop();
    return acked;
  }

  void delay_ms(std::uint32_t milliseconds) override { (void)milliseconds; }  // the model settles instantly

  [[nodiscard]] bool interrupt_line() const override { return slave_.interrupt_asserted(); }

private:
  Bmp390I2cSlave& slave_;
  std::uint8_t probe_address_;
};

// probe() must identify the part, pull the calibration NVM through the real
// register interface, and leave the part converting at the configured ODR.
void test_probe_reads_identity_and_calibration()
{
  Bmp390I2cSlave slave;
  DirectBus bus(slave);
  Bmp390Driver driver(bus);

  assert(slave.sampling_period_us() == 0);  // power-on: sleep mode
  assert(driver.probe() == Bmp390Error::kNone);

  // The calibration words the driver decoded are the burn the model serves.
  const auto& calib = driver.calibration();
  assert(calib.par_t1 == kBmp390ReferenceCalibration.par_t1);
  assert(calib.par_t2 == kBmp390ReferenceCalibration.par_t2);
  assert(calib.par_t3 == kBmp390ReferenceCalibration.par_t3);
  assert(calib.par_p1 == kBmp390ReferenceCalibration.par_p1);
  assert(calib.par_p5 == kBmp390ReferenceCalibration.par_p5);
  assert(calib.par_p9 == kBmp390ReferenceCalibration.par_p9);
  assert(calib.par_p11 == kBmp390ReferenceCalibration.par_p11);

  // Normal mode at the driver's default 50 Hz ODR.
  assert(slave.sampling_period_us() == 20000);
}

// A part strapped to the other address must look absent: the very first
// transaction fails and probe() reports the bus, not a bogus identity.
void test_probe_wrong_address_fails()
{
  Bmp390I2cSlave slave(kBmp390ReferenceCalibration, kBmp390I2cAddressSecondary);
  DirectBus bus(slave, kBmp390I2cAddressPrimary);
  Bmp390Driver driver(bus);

  assert(driver.probe() == Bmp390Error::kTransferFailed);
}

// A noiseless measurement model must round-trip: truth altitude through the
// ISA, the inverse compensation, the register file, the real driver and the
// real forward compensation, back to the ISA values -- to inversion
// granularity plus float truncation.
void test_noiseless_sample_round_trips()
{
  Bmp390MeasurementConfig config;
  config.pressure_noise_pa = 0.0F;
  config.temperature_noise_c = 0.0F;
  config.pressure_bias_sigma_pa = 0.0F;
  config.temperature_bias_sigma_c = 0.0F;

  Bmp390MeasurementModel model(config, /*seed=*/42);
  Bmp390I2cSlave slave(model.calibration());
  DirectBus bus(slave);
  Bmp390Driver driver(bus);
  assert(driver.probe() == Bmp390Error::kNone);

  for (const double altitude_m : { 0.0, 1500.0, 5000.0, 11000.0, 15000.0 })
  {
    const auto conversion = model.measure(altitude_m);
    slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp);
    assert(driver.data_ready());

    BaroSample sample;
    assert(driver.read_sample(sample) == Bmp390ReadResult::kSample);
    assert(near(sample.pressure_pa, BaroNoiseModel::isa_pressure_pa(altitude_m), 0.5));
    assert(near(sample.temperature_c, BaroNoiseModel::isa_temperature_c(altitude_m), 0.01));
  }
}

// Reading the data block consumes the conversion: until the next latch, the
// driver sees no new data, exactly as the drdy status bits behave on the
// part.
void test_read_consumes_data_ready()
{
  Bmp390MeasurementModel model({}, /*seed=*/7);
  Bmp390I2cSlave slave(model.calibration());
  DirectBus bus(slave);
  Bmp390Driver driver(bus);
  assert(driver.probe() == Bmp390Error::kNone);

  BaroSample sample;
  assert(driver.read_sample(sample) == Bmp390ReadResult::kNoNewData);  // nothing latched yet

  const auto conversion = model.measure(250.0);
  slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp);
  assert(driver.read_sample(sample) == Bmp390ReadResult::kSample);
  assert(!driver.data_ready());
  assert(driver.read_sample(sample) == Bmp390ReadResult::kNoNewData);
}

// A noisy sample must still land within a generous (5-sigma plus bias)
// bound of the ISA truth -- a statistical check with a fixed seed so it
// isn't flaky.
void test_noisy_sample_stays_bounded()
{
  Bmp390MeasurementConfig config;  // defaults: bias + white noise on
  Bmp390MeasurementModel model(config, /*seed=*/19);
  Bmp390I2cSlave slave(model.calibration());
  DirectBus bus(slave);
  Bmp390Driver driver(bus);
  assert(driver.probe() == Bmp390Error::kNone);

  const double altitude_m = 1500.0;
  const auto conversion = model.measure(altitude_m);
  slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp);

  BaroSample sample;
  assert(driver.read_sample(sample) == Bmp390ReadResult::kSample);

  const double pressure_bound =
      5.0 * static_cast<double>(config.pressure_noise_pa + config.pressure_bias_sigma_pa) + 0.5;
  const double temperature_bound =
      5.0 * static_cast<double>(config.temperature_noise_c + config.temperature_bias_sigma_c) + 0.01;
  assert(near(sample.pressure_pa, BaroNoiseModel::isa_pressure_pa(altitude_m), pressure_bound));
  assert(near(sample.temperature_c, BaroNoiseModel::isa_temperature_c(altitude_m), temperature_bound));
}

// Soft reset must drop the configuration back to power-on (sleep, no
// conversions) while the calibration NVM survives -- which probe() then
// proves by bringing the part all the way back up.
void test_soft_reset_then_reprobe()
{
  using hemerion::sensors::baro::bmp390::Bmp390Register;
  using hemerion::sensors::baro::bmp390::kBmp390CmdSoftReset;

  Bmp390I2cSlave slave;
  DirectBus bus(slave);
  Bmp390Driver driver(bus);
  assert(driver.probe() == Bmp390Error::kNone);
  assert(slave.sampling_period_us() == 20000);

  assert(bus.write_register(static_cast<std::uint8_t>(Bmp390Register::kCmd), kBmp390CmdSoftReset));
  assert(slave.sampling_period_us() == 0);

  assert(driver.probe() == Bmp390Error::kNone);
  assert(slave.sampling_period_us() == 20000);
}

// Forced mode owes exactly one conversion, then the part falls back to
// sleep.
void test_forced_mode_single_conversion()
{
  using hemerion::sensors::baro::bmp390::Bmp390Register;
  using hemerion::sensors::baro::bmp390::kBmp390PwrCtrlModeForced;
  using hemerion::sensors::baro::bmp390::kBmp390PwrCtrlPressureEnable;
  using hemerion::sensors::baro::bmp390::kBmp390PwrCtrlTemperatureEnable;

  Bmp390I2cSlave slave;
  DirectBus bus(slave);

  const std::uint8_t forced = kBmp390PwrCtrlPressureEnable | kBmp390PwrCtrlTemperatureEnable | kBmp390PwrCtrlModeForced;
  assert(bus.write_register(static_cast<std::uint8_t>(Bmp390Register::kPwrCtrl), forced));

  assert(slave.sampling_period_us() == 0);  // forced is not free-running
  assert(slave.take_forced_conversion());
  assert(!slave.take_forced_conversion());  // consumed; the part is asleep again
}

}  // namespace

int main()
{
  test_probe_reads_identity_and_calibration();
  test_probe_wrong_address_fails();
  test_noiseless_sample_round_trips();
  test_read_consumes_data_ready();
  test_noisy_sample_stays_bounded();
  test_soft_reset_then_reprobe();
  test_forced_mode_single_conversion();

  std::puts("test_bmp390: all checks passed");
  return 0;
}
