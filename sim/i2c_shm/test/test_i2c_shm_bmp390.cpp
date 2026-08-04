// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_i2c_shm_bmp390.cpp
//
// The full SWIL chain for the BMP390, minus only the FMI wrapper: the real
// on-target Bmp390Driver issues its transactions through I2cShmController,
// across the shared-memory bus, into the I2cPeripheralEndpoint +
// Bmp390I2cSlave exactly as the hemerion_bmp390_fmu packages them -- while
// this thread plays the co-simulation master, latching conversions from the
// measurement model between polls. What test_bmp390.cpp proves about
// register/math compatibility, this test proves survives the real transport
// and its service thread.
//
// Single-process like test_i2c_shm_link.cpp: both bus ends map the same
// named segment.
// ------------------------------------------------------------------------------
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <thread>

#include "Hemerion/baro/baro_types.h"
#include "Hemerion/baro/bmp390/bmp390_driver.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"
#include "Hemerion/baro/fmu/baro_noise_model.h"
#include "hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h"

using hemerion::sensors::baro::BaroSample;
using hemerion::sensors::baro::bmp390::Bmp390Driver;
using hemerion::sensors::baro::bmp390::Bmp390Error;
using hemerion::sensors::baro::bmp390::Bmp390I2cBus;
using hemerion::sensors::baro::bmp390::Bmp390ReadResult;
using hemerion::sensors::baro::bmp390::kBmp390I2cAddressPrimary;
using hemerion::sensors::baro::bmp390::fmu::Bmp390I2cSlave;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementConfig;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementModel;
using hemerion::sensors::baro::fmu::BaroNoiseModel;
using hemerion::sim::i2c_shm::I2cPeripheralConfig;
using hemerion::sim::i2c_shm::I2cPeripheralEndpoint;
using hemerion::sim::i2c_shm::I2cShmController;
using namespace std::chrono_literals;

namespace
{

constexpr auto kTimeout = 2000ms;

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// What the firmware's board layer is on hardware, this is for the host
// co-simulation: Bmp390I2cBus over the shared-memory controller endpoint --
// the same adapter shape a flight-computer example process would use.
class ShmBus final : public Bmp390I2cBus
{
public:
  explicit ShmBus(I2cShmController& controller) : controller_(controller) {}

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const std::uint8_t frame[2] = { reg, value };
    return controller_.transaction(kBmp390I2cAddressPrimary, frame, 2, nullptr, 0, kTimeout) ==
           I2cShmController::Result::kOk;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    return controller_.transaction(kBmp390I2cAddressPrimary, &reg, 1, out, count, kTimeout) ==
           I2cShmController::Result::kOk;
  }

  void delay_ms(std::uint32_t milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  [[nodiscard]] bool interrupt_line() const override { return controller_.interrupt_line(); }

private:
  I2cShmController& controller_;
};

void test_probe_and_sample_across_the_bus()
{
  // Peripheral side, wired exactly as fmu_main.cpp wires it (no environment
  // override, so the test cannot collide with a configured run).
  Bmp390MeasurementConfig config;
  config.pressure_noise_pa = 0.0F;
  config.temperature_noise_c = 0.0F;
  config.pressure_bias_sigma_pa = 0.0F;
  config.temperature_bias_sigma_c = 0.0F;
  Bmp390MeasurementModel model(config, /*seed=*/42);
  Bmp390I2cSlave slave(model.calibration());
  I2cPeripheralEndpoint<Bmp390I2cSlave> endpoint(slave, I2cPeripheralConfig{ "hemerion_test_i2c_bmp390", "" });
  assert(endpoint.attach());

  // Controller side: the on-target driver over the shm transport.
  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_bmp390", kTimeout);
  assert(controller.has_value());
  ShmBus bus(*controller);
  Bmp390Driver driver(bus);

  assert(driver.probe() == Bmp390Error::kNone);
  assert(endpoint.controller_attached());
  assert(slave.sampling_period_us() == 20000);  // the driver's 50 Hz ODR arrived through the bus

  // A few "communication steps": latch a conversion, let the driver poll it
  // off the bus, check the compensated values against the ISA truth and the
  // SENSORTIME stamp against the latched clock.
  std::uint64_t timestamp_us = 0;
  for (const double altitude_m : { 0.0, 3000.0, 9000.0 })
  {
    timestamp_us += 20000;
    const auto conversion = model.measure(altitude_m);
    slave.latch_conversion(conversion.uncomp_press, conversion.uncomp_temp, timestamp_us);
    endpoint.publish_interrupt(slave.interrupt_asserted());
    assert(driver.data_ready());

    BaroSample sample;
    assert(driver.read_sample(sample) == Bmp390ReadResult::kSample);
    assert(near(sample.pressure_pa, BaroNoiseModel::isa_pressure_pa(altitude_m), 0.5));
    assert(near(sample.temperature_c, BaroNoiseModel::isa_temperature_c(altitude_m), 0.01));
    assert(sample.timestamp_us <= timestamp_us && timestamp_us - sample.timestamp_us <= 31);

    endpoint.publish_interrupt(slave.interrupt_asserted());
    assert(!driver.data_ready());
    assert(driver.read_sample(sample) == Bmp390ReadResult::kNoNewData);
  }

  assert(endpoint.transactions_serviced() > 0);

  // Power-down: the next poll fails as a transfer, not a hang -- the part is
  // gone and the driver's caller finds out immediately.
  endpoint.detach();
  BaroSample sample;
  assert(driver.read_sample(sample) == Bmp390ReadResult::kTransferFailed);
}

}  // namespace

int main()
{
  test_probe_and_sample_across_the_bus();

  std::puts("test_i2c_shm_bmp390: all checks passed");
  return 0;
}
