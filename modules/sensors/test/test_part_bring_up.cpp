// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_part_bring_up.cpp
//
// Whether a bus part's sample stream depends on when its controller ran.
//
// In the co-simulations the controller (the flight computer) is a separate
// process on the wall clock, and the part's FMU steps on the master's
// schedule. Before part_bring_up.h, the step a bring-up command landed in was
// whichever one the wall clock delivered it to, and so the BMP390's
// conversion phase, the MMC5983MA's measurement phase and the IMU's first
// surviving sample varied run to run under one seed (Linux CI, 2026-11-20).
//
// Each bench below is that situation in one process: a paced stepping loop
// that does what the FMU's do_step() does -- PartBringUpGate::hold() on the
// first step, then FreeRunningSchedule (or the IMU's per-step push) -- and a
// controller thread running the unmodified on-target driver's bring-up after
// a wall-clock delay. The streams must be identical for an early and a late
// controller once the gate holds; without it (timeout 0, the FMUs' default
// and the behaviour before the gate) the late controller moves the stream,
// which is checked too so the comparison cannot pass vacuously.
//
// Plain asserts + exit code, matching test_bmp390.cpp.
// ------------------------------------------------------------------------------

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "Hemerion/baro/bmp390/bmp390_driver.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"
#include "Hemerion/imu/fmu/imu_spi_slave.h"
#include "Hemerion/imu/imu_spi_driver.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"
#include "Hemerion/part_bring_up.h"

using hemerion::sensors::FreeRunningSchedule;
using hemerion::sensors::PartBringUpGate;
using hemerion::sensors::baro::bmp390::Bmp390Driver;
using hemerion::sensors::baro::bmp390::Bmp390Error;
using hemerion::sensors::baro::bmp390::Bmp390I2cBus;
using hemerion::sensors::baro::bmp390::kBmp390I2cAddressPrimary;
using hemerion::sensors::baro::bmp390::fmu::Bmp390I2cSlave;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSpiBus;
using hemerion::sensors::imu::ImuSpiDriver;
using hemerion::sensors::imu::ImuSpiError;
using hemerion::sensors::imu::fmu::ImuPacketEmitter;
using hemerion::sensors::imu::fmu::ImuSpiSlave;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maI2cAddress;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maBridgeOffset;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maConfig;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maDriver;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maError;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maFieldCounts;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maI2cBus;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maI2cSlave;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;

namespace
{

// The bench master: 0.01 s steps, the step-7 baseline, paced at 2 ms of wall
// time each so a controller delayed by kLateController arrives tens of steps
// in when nothing holds the first one.
constexpr double kStepS = 0.01;
constexpr int kSteps = 150;
constexpr auto kStepWall = std::chrono::milliseconds(2);
constexpr auto kEarlyController = std::chrono::milliseconds(0);
constexpr auto kLateController = std::chrono::milliseconds(60);
constexpr double kHoldTimeoutS = 10.0;

bool same_time(double a, double b) { return std::fabs(a - b) < 1e-12; }

// One step's wall time. Spun rather than slept: a 2 ms sleep is a 15.6 ms
// scheduler tick on Windows, which would only make the bench slow.
void pace_step()
{
  const auto until = std::chrono::steady_clock::now() + kStepWall;
  while (std::chrono::steady_clock::now() < until)
  {
    std::this_thread::yield();
  }
}

// ---------------------------------------------------------------- gate, schedule

void test_gate_outcomes()
{
  {
    PartBringUpGate gate;
    gate.arm(0.0);
    int checks = 0;
    gate.hold([&] {
      ++checks;
      return false;
    });
    assert(gate.outcome() == PartBringUpGate::Outcome::kNotReady);
    assert(!gate.ready_at_start());
    assert(checks == 1);
    gate.hold([&] {
      ++checks;
      return true;
    });  // only the first step holds
    assert(checks == 1);
  }
  {
    PartBringUpGate gate;
    gate.arm(kHoldTimeoutS);
    gate.hold([] { return true; });
    assert(gate.outcome() == PartBringUpGate::Outcome::kReady);
    assert(gate.ready_at_start());
  }
  {
    PartBringUpGate gate;
    gate.arm(0.05);
    const auto start = std::chrono::steady_clock::now();
    int services = 0;
    gate.hold([] { return false; }, [&] { ++services; });
    assert(gate.outcome() == PartBringUpGate::Outcome::kTimedOut);
    assert(std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(50));
    assert(services > 1);  // served while holding, not just once
    gate.arm(0.05);        // a reset re-arms it
    assert(gate.outcome() == PartBringUpGate::Outcome::kPending);
  }
}

void test_schedule_grid()
{
  // 50 Hz at 0.01 s steps: one conversion at the end of every second step.
  FreeRunningSchedule schedule;
  std::vector<double> times;
  for (int k = 0; k < 6; ++k)
  {
    schedule.advance(kStepS, 20000, [&](double offset_s) { times.push_back(k * kStepS + offset_s); });
  }
  assert(times.size() == 3);
  assert(same_time(times[0], 0.02) && same_time(times[1], 0.04) && same_time(times[2], 0.06));

  // 200 Hz: two per step, at their own instants within it.
  times.clear();
  schedule.reset();
  schedule.advance(kStepS, 5000, [&](double offset_s) { times.push_back(offset_s); });
  assert(times.size() == 2);
  assert(same_time(times[0], 0.005) && same_time(times[1], 0.01));

  // Idle holds the phase at zero: a part started mid-period converts one
  // whole period after the step that saw the command.
  times.clear();
  schedule.advance(kStepS * 0.5, 20000, [&](double offset_s) { times.push_back(offset_s); });
  schedule.advance(kStepS, 0, [&](double offset_s) { times.push_back(offset_s); });
  schedule.advance(kStepS, 20000, [&](double offset_s) { times.push_back(offset_s); });
  schedule.advance(kStepS, 20000, [&](double offset_s) { times.push_back(offset_s); });
  assert(times.size() == 1);
  assert(same_time(times[0], kStepS));
}

// ---------------------------------------------------------------- BMP390

class Bmp390Bus final : public Bmp390I2cBus
{
public:
  explicit Bmp390Bus(Bmp390I2cSlave& slave) : slave_(slave) {}

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const bool acked = slave_.start(kBmp390I2cAddressPrimary, false) && slave_.write(reg) && slave_.write(value);
    slave_.stop();
    return acked;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    const bool acked = slave_.start(kBmp390I2cAddressPrimary, false) && slave_.write(reg) &&
                       slave_.start(kBmp390I2cAddressPrimary, true);
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

  void delay_ms(std::uint32_t milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  [[nodiscard]] bool interrupt_line() const override { return slave_.interrupt_asserted(); }

private:
  Bmp390I2cSlave& slave_;
};

struct BaroRun
{
  std::vector<double> conversion_times_s;
  bool ready_at_start = false;
};

// The BMP390 FMU's do_step() conversion path, with a controller arriving
// `controller_delay` into the run.
BaroRun run_baro(std::chrono::milliseconds controller_delay, double hold_timeout_s)
{
  Bmp390I2cSlave slave;
  std::atomic<bool> probed{ false };
  std::thread controller([&] {
    std::this_thread::sleep_for(controller_delay);
    Bmp390Bus bus(slave);
    Bmp390Driver driver(bus);
    probed = driver.probe() == Bmp390Error::kNone;
  });

  BaroRun run;
  PartBringUpGate gate;
  gate.arm(hold_timeout_s);
  FreeRunningSchedule schedule;
  for (int k = 0; k < kSteps; ++k)
  {
    gate.hold([&] { return slave.measuring(); });
    const double now_s = k * kStepS;
    schedule.advance(kStepS, slave.sampling_period_us(), [&](double offset_s) {
      run.conversion_times_s.push_back(now_s + offset_s);
    });
    pace_step();
  }
  controller.join();
  assert(probed);
  run.ready_at_start = gate.ready_at_start();
  return run;
}

void test_baro_conversions_do_not_depend_on_when_the_controller_ran()
{
  const BaroRun early = run_baro(kEarlyController, kHoldTimeoutS);
  const BaroRun late = run_baro(kLateController, kHoldTimeoutS);
  assert(early.ready_at_start && late.ready_at_start);
  // Normal mode at the driver's 50 Hz from t = 0: 0.02, 0.04, ...
  assert(early.conversion_times_s.size() == kSteps / 2);
  assert(same_time(early.conversion_times_s.front(), 0.02));
  assert(late.conversion_times_s == early.conversion_times_s);

  // The defect the gate removes: without it the late controller's PWR_CTRL
  // write lands steps into the run and the whole schedule moves with it.
  const BaroRun unheld = run_baro(kLateController, 0.0);
  assert(!unheld.ready_at_start);
  assert(!unheld.conversion_times_s.empty());
  assert(unheld.conversion_times_s.front() > early.conversion_times_s.front() + kStepS / 2);
}

// ---------------------------------------------------------------- IMU

class ImuBus final : public ImuSpiBus
{
public:
  explicit ImuBus(ImuSpiSlave& slave) : slave_(slave) {}

  bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t length) override
  {
    slave_.chip_select(true);
    for (std::size_t i = 0; i < length; ++i)
    {
      rx[i] = slave_.shift(tx[i]);
    }
    slave_.chip_select(false);
    return true;
  }

  [[nodiscard]] bool data_ready_line() const override { return slave_.data_ready(); }

private:
  ImuSpiSlave& slave_;
};

struct ImuRun
{
  std::vector<std::uint64_t> stamps_us;
  bool ready_at_start = false;
};

// The IMU FMU's do_step() at 100 Hz (one frame per 0.01 s step, stamped with
// its own sample time), with a controller arriving `controller_delay` in.
// The controller's probe writes CONTROL with the FIFO-reset bit.
ImuRun run_imu(std::chrono::milliseconds controller_delay, double hold_timeout_s)
{
  ImuSpiSlave slave;
  std::atomic<bool> probed{ false };
  std::thread controller([&] {
    std::this_thread::sleep_for(controller_delay);
    ImuBus bus(slave);
    ImuSpiDriver driver(bus);
    probed = driver.probe() == ImuSpiError::kNone;
  });

  ImuRun run;
  PartBringUpGate gate;
  gate.arm(hold_timeout_s);
  for (int k = 0; k < kSteps; ++k)
  {
    gate.hold([&] { return slave.configured(); });
    ImuRawSample raw;
    raw.timestamp_us = static_cast<std::uint64_t>(k + 1) * 10000ULL;
    const auto frame = ImuPacketEmitter::encode_raw_sample(raw);
    assert(slave.push_frame(frame.data(), frame.size()));
    pace_step();
  }
  controller.join();
  assert(probed);
  run.ready_at_start = gate.ready_at_start();

  // Drain what the controller would read after its bring-up.
  ImuBus bus(slave);
  ImuSpiDriver driver(bus);
  std::vector<ImuRawSample> batch(kSteps);
  while (driver.data_ready())
  {
    const ImuSpiDriver::PollResult result = driver.poll(batch.data(), batch.size());
    assert(result.error == ImuSpiError::kNone);
    if (result.samples == 0)
    {
      break;
    }
    for (std::size_t i = 0; i < result.samples; ++i)
    {
      run.stamps_us.push_back(batch[i].timestamp_us);
    }
  }
  return run;
}

void test_imu_stream_does_not_depend_on_when_the_controller_ran()
{
  const ImuRun early = run_imu(kEarlyController, kHoldTimeoutS);
  const ImuRun late = run_imu(kLateController, kHoldTimeoutS);
  assert(early.ready_at_start && late.ready_at_start);
  // Every sample from the first step on survives the bring-up's FIFO reset.
  assert(early.stamps_us.size() == static_cast<std::size_t>(kSteps));
  assert(early.stamps_us.front() == 10000ULL);
  assert(late.stamps_us == early.stamps_us);

  // Without the hold the reset empties whatever the late controller found.
  const ImuRun unheld = run_imu(kLateController, 0.0);
  assert(!unheld.ready_at_start);
  assert(unheld.stamps_us.size() < early.stamps_us.size());
  assert(unheld.stamps_us.front() > 10000ULL);
}

// ---------------------------------------------------------------- MMC5983MA

class MagBus final : public Mmc5983maI2cBus
{
public:
  explicit MagBus(Mmc5983maI2cSlave& slave) : slave_(slave), start_(std::chrono::steady_clock::now()) {}

  bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    const bool acked = slave_.start(kMmc5983maI2cAddress, false) && slave_.write(reg) && slave_.write(value);
    slave_.stop();
    return acked;
  }

  bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    const bool acked =
        slave_.start(kMmc5983maI2cAddress, false) && slave_.write(reg) && slave_.start(kMmc5983maI2cAddress, true);
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

  // Real sleeps: the part is answered by the stepping thread, as in the FMU.
  void delay_ms(std::uint32_t milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  [[nodiscard]] std::uint64_t now_us() const override
  {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_).count());
  }

  [[nodiscard]] bool interrupt_line() const override { return slave_.interrupt_asserted(); }

private:
  Mmc5983maI2cSlave& slave_;
  std::chrono::steady_clock::time_point start_;
};

struct MagMeasurement
{
  double time_s = 0.0;
  Mmc5983maFieldCounts counts;

  bool operator==(const MagMeasurement& other) const
  {
    return same_time(time_s, other.time_s) && counts.x == other.counts.x && counts.y == other.counts.y &&
           counts.z == other.counts.z;
  }
};

struct MagRun
{
  std::vector<MagMeasurement> continuous;
  Mmc5983maBridgeOffset offset;
  bool ready_at_start = false;
};

// The MMC5983MA FMU's do_step(): the hold answering the bring-up's triggered
// measurements, then triggered measurements, then continuous mode, from a
// seeded model and a fixed field. The controller's probe blocks on the
// triggered SET/RESET pair, so it only completes because the hold serves it.
MagRun run_mag(std::chrono::milliseconds controller_delay, double hold_timeout_s)
{
  Mmc5983maI2cSlave slave;
  Mmc5983maMeasurementModel model({}, 1234);
  const auto measure = [&] { return model.measure(22.0, -6.0, 41.0, slave.sensing_state()); };
  const auto serve_triggered = [&] {
    if (slave.take_triggered_measurement())
    {
      slave.latch_measurement(measure());
    }
    if (slave.take_triggered_temperature())
    {
      slave.latch_temperature(model.measure_temperature(25.0));
    }
  };

  MagRun run;
  std::atomic<bool> probed{ false };
  std::thread controller([&] {
    std::this_thread::sleep_for(controller_delay);
    MagBus bus(slave);
    Mmc5983maDriver driver(bus);
    probed = driver.probe(Mmc5983maConfig{}) == Mmc5983maError::kNone;
    run.offset = driver.bridge_offset();
  });

  PartBringUpGate gate;
  gate.arm(hold_timeout_s);
  FreeRunningSchedule schedule;
  for (int k = 0; k < kSteps; ++k)
  {
    gate.hold([&] { return slave.sampling_period_us() != 0; }, serve_triggered);
    serve_triggered();
    const double now_s = k * kStepS;
    schedule.advance(kStepS, slave.sampling_period_us(), [&](double offset_s) {
      const Mmc5983maFieldCounts counts = measure();
      slave.latch_measurement(counts);
      run.continuous.push_back(MagMeasurement{ now_s + offset_s, counts });
    });
    pace_step();
  }
  controller.join();
  assert(probed);
  run.ready_at_start = gate.ready_at_start();
  return run;
}

void test_mag_measurements_do_not_depend_on_when_the_controller_ran()
{
  const MagRun early = run_mag(kEarlyController, kHoldTimeoutS);
  const MagRun late = run_mag(kLateController, kHoldTimeoutS);
  assert(early.ready_at_start && late.ready_at_start);
  // Continuous mode at the driver's default 50 Hz from t = 0.
  assert(early.continuous.size() == kSteps / 2);
  assert(same_time(early.continuous.front().time_s, 0.02));
  assert(late.continuous == early.continuous);
  assert(late.offset.x == early.offset.x && late.offset.y == early.offset.y && late.offset.z == early.offset.z);

  // Without the hold, the late controller's bring-up is served by whichever
  // steps it lands in and continuous mode starts there.
  const MagRun unheld = run_mag(kLateController, 0.0);
  assert(!unheld.ready_at_start);
  assert(!unheld.continuous.empty());
  assert(unheld.continuous.front().time_s > early.continuous.front().time_s + kStepS / 2);
}

}  // namespace

int main()
{
  test_gate_outcomes();
  test_schedule_grid();
  test_baro_conversions_do_not_depend_on_when_the_controller_ran();
  test_imu_stream_does_not_depend_on_when_the_controller_ran();
  test_mag_measurements_do_not_depend_on_when_the_controller_ran();
  std::puts("test_part_bring_up: all tests passed");
  return 0;
}
