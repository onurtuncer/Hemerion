// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_i2c_shm_mmc5983ma.cpp
//
// The full SWIL chain for the MMC5983MA, minus only the FMI wrapper: the real
// on-target Mmc5983maDriver issues its transactions through I2cShmController,
// across the shared-memory bus, into the I2cPeripheralEndpoint +
// Mmc5983maI2cSlave exactly as the hemerion_mmc5983ma_fmu packages them --
// while a pump thread plays the co-simulation master. What test_mmc5983ma.cpp
// proves about register/math compatibility, this test proves survives the
// real transport and its service thread.
//
// **Why this one needs a thread and test_i2c_shm_bmp390.cpp does not.** The
// BMP390 free-runs: its master can latch a conversion, then call the driver,
// then latch again, all on one thread. This part measures *on command* and
// the driver's bring-up blocks on the result -- calibrate_offset() writes
// SET, triggers TM_M, and then polls Status until the measurement lands. A
// single-threaded master would never reach the latch, so probe() would time
// out against a part that is merely paused. The pump below is the same loop
// the FMU's do_step() and sim/i2c_shm/tools/mmc5983ma_shm_peripheral.cpp run,
// which is exactly the point: the blocking, multi-transaction, stateful
// SET/RESET handshake is the thing worth pushing through a real transport.
//
// Single-process like test_i2c_shm_link.cpp: both bus ends map the same
// named segment.
// ------------------------------------------------------------------------------
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <thread>

#include "Hemerion/mag/mag_types.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"
#include "hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h"

using hemerion::sensors::mag::MagSample;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maI2cAddress;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maLsbPerMicrotesla;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maDriver;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maError;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maI2cBus;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maReadResult;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maI2cSlave;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementConfig;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;
using hemerion::sim::i2c_shm::I2cPeripheralConfig;
using hemerion::sim::i2c_shm::I2cPeripheralEndpoint;
using hemerion::sim::i2c_shm::I2cShmController;
using namespace std::chrono_literals;

namespace
{

constexpr auto kTimeout = 2000ms;

// A truth field roughly the size and shape of Istanbul's.
constexpr double kTruthXUt = 22.0;
constexpr double kTruthYUt = -6.0;
constexpr double kTruthZUt = 41.0;

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// What the firmware's board layer is on hardware, this is for the host
// co-simulation: Mmc5983maI2cBus over the shared-memory controller endpoint --
// the same adapter shape a flight-computer example process would use.
class ShmBus final : public Mmc5983maI2cBus
{
public:
  explicit ShmBus(I2cShmController& controller) : controller_(controller) {}

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const std::uint8_t frame[2] = { reg, value };
    return controller_.transaction(kMmc5983maI2cAddress, frame, 2, nullptr, 0, kTimeout) ==
           I2cShmController::Result::kOk;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    return controller_.transaction(kMmc5983maI2cAddress, &reg, 1, out, count, kTimeout) ==
           I2cShmController::Result::kOk;
  }

  void delay_ms(std::uint32_t milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  [[nodiscard]] std::uint64_t now_us() const override
  {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

  [[nodiscard]] bool interrupt_line() const override { return controller_.interrupt_line(); }

private:
  I2cShmController& controller_;
};

// The co-simulation master, on its own thread: serve triggered measurements,
// then free-run at whatever period the controller has programmed. A
// transcription of the FMU's do_step() with wall time standing in for
// simulation time -- see this file's header comment for why it cannot be
// folded into the test thread.
class PartPump
{
public:
  PartPump(Mmc5983maI2cSlave& slave,
           Mmc5983maMeasurementModel& model,
           I2cPeripheralEndpoint<Mmc5983maI2cSlave>& endpoint,
           const std::atomic<double>* truth)
    : thread_([this, &slave, &model, &endpoint, truth] {
      auto previous = std::chrono::steady_clock::now();
      double time_into_period_s = 0.0;
      while (!stop_.load(std::memory_order_relaxed))
      {
        std::this_thread::sleep_for(1ms);
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - previous).count();
        previous = now;

        auto measure = [&] {
          slave.latch_measurement(model.measure(truth[0].load(std::memory_order_relaxed),
                                                truth[1].load(std::memory_order_relaxed),
                                                truth[2].load(std::memory_order_relaxed),
                                                slave.sensing_state()));
        };

        if (slave.take_triggered_measurement())
        {
          measure();
        }
        if (slave.take_triggered_temperature())
        {
          slave.latch_temperature(model.measure_temperature(25.0));
        }

        const std::uint64_t period_us = slave.sampling_period_us();
        if (period_us == 0)
        {
          time_into_period_s = 0.0;
        }
        else
        {
          const double period_s = static_cast<double>(period_us) * 1e-6;
          time_into_period_s += dt;
          if (time_into_period_s >= period_s)
          {
            measure();
            time_into_period_s = std::fmod(time_into_period_s, period_s);
          }
        }
        endpoint.publish_interrupt(slave.interrupt_asserted());
      }
    })
  {
  }

  PartPump(const PartPump&) = delete;
  PartPump& operator=(const PartPump&) = delete;
  PartPump(PartPump&&) = delete;
  PartPump& operator=(PartPump&&) = delete;

  ~PartPump()
  {
    stop_.store(true, std::memory_order_relaxed);
    thread_.join();
  }

private:
  std::atomic<bool> stop_{ false };
  std::thread thread_;
};

void test_probe_and_sample_across_the_bus()
{
  // Peripheral side, wired exactly as fmu_main.cpp wires it (no environment
  // override, so the test cannot collide with a configured run). Noiseless so
  // the round trip is exact, but the bridge offset is left at its full
  // default: recovering *that* across the transport is what this proves.
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;
  config.temperature_noise_c = 0.0F;
  Mmc5983maMeasurementModel model(config, /*seed=*/42);
  Mmc5983maI2cSlave slave;
  I2cPeripheralEndpoint<Mmc5983maI2cSlave> endpoint(slave, I2cPeripheralConfig{ "hemerion_test_i2c_mmc5983ma", "" });
  assert(endpoint.attach());

  std::atomic<double> truth[3] = { kTruthXUt, kTruthYUt, kTruthZUt };
  PartPump pump(slave, model, endpoint, truth);

  // Controller side: the on-target driver over the shm transport.
  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_mmc5983ma", kTimeout);
  assert(controller.has_value());
  ShmBus bus(*controller);
  Mmc5983maDriver driver(bus);

  // probe() here is not the one-shot register write the BMP390's is: it
  // identifies, software-resets, then runs a whole SET / measure / RESET /
  // measure / SET handshake across the bus before programming continuous
  // mode. Every one of those steps is a separate transaction with a poll loop
  // between them.
  assert(driver.probe() == Mmc5983maError::kNone);
  assert(endpoint.controller_attached());
  assert(slave.sampling_period_us() == 20000);  // the driver's 50 Hz arrived through the bus

  // The bridge offset the driver dug out matches the one the simulated part
  // was born with -- the SET/RESET pair survived the transport intact.
  const auto& truth_offset = model.bridge_offset();
  assert(std::abs(driver.bridge_offset().x - truth_offset.x) <= 1);
  assert(std::abs(driver.bridge_offset().y - truth_offset.y) <= 1);
  assert(std::abs(driver.bridge_offset().z - truth_offset.z) <= 1);

  // And the part is left SET, not RESET: a sign error here would still pass
  // every offset check above.
  assert(slave.sensing_state().magnetization == +1);

  // A few samples off the free-running part, at fields the pump picks up
  // between iterations.
  const double quantum_ut = 1.0 / static_cast<double>(kMmc5983maLsbPerMicrotesla);
  for (const double scale : { 1.0, -1.0, 0.5 })
  {
    truth[0].store(kTruthXUt * scale, std::memory_order_relaxed);
    truth[1].store(kTruthYUt * scale, std::memory_order_relaxed);
    truth[2].store(kTruthZUt * scale, std::memory_order_relaxed);

    // Wait for a measurement taken *after* the new truth was published: the
    // pump may already have been mid-iteration with the old value.
    std::this_thread::sleep_for(60ms);

    MagSample sample;
    Mmc5983maReadResult result = Mmc5983maReadResult::kNoNewData;
    for (int attempt = 0; attempt < 100 && result != Mmc5983maReadResult::kSample; ++attempt)
    {
      result = driver.read_sample(sample);
      if (result == Mmc5983maReadResult::kNoNewData)
      {
        std::this_thread::sleep_for(5ms);
      }
    }
    assert(result == Mmc5983maReadResult::kSample);
    assert(near(sample.mag_x_ut, kTruthXUt * scale, 2.0 * quantum_ut));
    assert(near(sample.mag_y_ut, kTruthYUt * scale, 2.0 * quantum_ut));
    assert(near(sample.mag_z_ut, kTruthZUt * scale, 2.0 * quantum_ut));
    assert(sample.timestamp_us > 0);  // stamped from the controller's clock
  }

  // The temperature channel takes the same triggered path as the calibration
  // measurements, so it exercises the poll loop once more.
  float temperature_c = 0.0F;
  assert(driver.read_temperature(temperature_c) == Mmc5983maError::kNone);
  assert(near(temperature_c, 25.0, 0.8));

  assert(endpoint.transactions_serviced() > 0);
}

// Power-down: the next transaction fails as a transfer, not a hang -- the
// part is gone and the driver's caller finds out immediately. Split from the
// test above so the pump is joined before the endpoint detaches.
void test_detach_fails_the_next_read()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;
  Mmc5983maMeasurementModel model(config, /*seed=*/7);
  Mmc5983maI2cSlave slave;
  I2cPeripheralEndpoint<Mmc5983maI2cSlave> endpoint(slave,
                                                    I2cPeripheralConfig{ "hemerion_test_i2c_mmc5983ma_off", "" });
  assert(endpoint.attach());

  std::optional<I2cShmController> controller =
      I2cShmController::attach_within("hemerion_test_i2c_mmc5983ma_off", kTimeout);
  assert(controller.has_value());
  ShmBus bus(*controller);
  Mmc5983maDriver driver(bus);

  {
    std::atomic<double> truth[3] = { kTruthXUt, kTruthYUt, kTruthZUt };
    PartPump pump(slave, model, endpoint, truth);
    assert(driver.probe() == Mmc5983maError::kNone);
  }

  endpoint.detach();
  MagSample sample;
  assert(driver.read_sample(sample) == Mmc5983maReadResult::kTransferFailed);
}

}  // namespace

int main()
{
  test_probe_and_sample_across_the_bus();
  test_detach_fails_the_next_read();

  std::puts("test_i2c_shm_mmc5983ma: all checks passed");
  return 0;
}
