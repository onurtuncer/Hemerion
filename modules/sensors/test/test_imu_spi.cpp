// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_imu_spi.cpp
//
// The IMU's SPI path, both ends against each other: the hardware simulator's
// register/FIFO model (imu/fmu/imu_spi_slave.h) answering the real, on-target
// ImuSpiDriver (imu/imu_spi_driver.h) across a bus stub that does nothing but
// shift bytes.
//
// This is the test that has to hold for the transport change to be a
// non-event for firmware: the same ImuPacketEmitter frames the FMU used to put
// on a UDP socket now go into a FIFO and come back out through burst reads,
// and the bytes -- and therefore the samples ImuPacketParser +
// convert_raw_to_si() recover -- must be identical either way.
//
// Plain asserts + exit code, matching test_imu_packet.cpp -- Unity is not yet
// vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"
#include "Hemerion/imu/fmu/imu_spi_slave.h"
#include "Hemerion/imu/imu_conversion.h"
#include "Hemerion/imu/imu_spi_driver.h"

using hemerion::sensors::imu::convert_raw_to_si;
using hemerion::sensors::imu::ImuConversionError;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSample;
using hemerion::sensors::imu::ImuSpiBus;
using hemerion::sensors::imu::ImuSpiDriver;
using hemerion::sensors::imu::ImuSpiError;
using hemerion::sensors::imu::ImuSpiRegister;
using hemerion::sensors::imu::kImuSpiFifoCapacityBytes;
using hemerion::sensors::imu::kImuSpiWhoAmI;
using hemerion::sensors::imu::fmu::ImuNoiseConfig;
using hemerion::sensors::imu::fmu::ImuNoiseModel;
using hemerion::sensors::imu::fmu::ImuPacketEmitter;
using hemerion::sensors::imu::fmu::ImuSpiSlave;
using hemerion::sensors::imu::fmu::ImuTruthSample;

namespace
{

// Stands in for the board: asserts chip select, shifts every byte through the
// part, releases chip select. Exactly what sim/spi_shm's service loop does for
// the real co-simulation, minus the shared memory.
class DirectSpiBus final : public ImuSpiBus
{
public:
  explicit DirectSpiBus(ImuSpiSlave& slave) : slave_(slave) {}

  bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t length) override
  {
    if (fail_next_)
    {
      fail_next_ = false;
      return false;
    }
    slave_.chip_select(true);
    for (std::size_t i = 0; i < length; ++i)
    {
      rx[i] = slave_.shift(tx[i]);
    }
    slave_.chip_select(false);
    ++transfers_;
    return true;
  }

  bool data_ready_line() const override { return slave_.data_ready(); }

  void fail_next_transfer() { fail_next_ = true; }
  [[nodiscard]] std::size_t transfers() const { return transfers_; }

private:
  ImuSpiSlave& slave_;
  std::size_t transfers_ = 0;
  bool fail_next_ = false;
};

ImuRawSample make_sample(std::int32_t seed)
{
  ImuRawSample raw;
  raw.accel_x = seed;
  raw.accel_y = -seed;
  raw.accel_z = static_cast<std::int32_t>(seed / 2);
  raw.gyro_x = static_cast<std::int32_t>(seed * 3);
  raw.gyro_y = 1;
  raw.gyro_z = -seed;
  raw.timestamp_us = static_cast<std::uint64_t>(seed) * 10000ULL;
  return raw;
}

void push(ImuSpiSlave& slave, const ImuRawSample& raw)
{
  const auto frame = ImuPacketEmitter::encode_raw_sample(raw);
  assert(slave.push_frame(frame.data(), frame.size()));
}

// A driver has to be able to tell it is talking to the right part before it
// trusts anything else it reads.
void test_probe_identifies_the_part()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);

  assert(driver.probe() == ImuSpiError::kNone);
  assert(bus.transfers() == 2);  // WHO_AM_I read, CONTROL write

  // A bus that cannot complete the identity read is reported as such, not as
  // a wrong part.
  bus.fail_next_transfer();
  assert(driver.probe() == ImuSpiError::kTransferFailed);
}

// The command byte is shifted in while the part shifts out a don't-care, and
// the map auto-increments -- which is what lets one transfer carry STATUS and
// both FIFO_COUNT bytes.
void test_register_reads_auto_increment()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  push(slave, make_sample(1));

  const std::array<std::uint8_t, 5> tx{
    hemerion::sensors::imu::imu_spi_command(ImuSpiRegister::kWhoAmI, /*read=*/true), 0, 0, 0, 0
  };
  std::array<std::uint8_t, 5> rx{};
  assert(bus.transfer(tx.data(), rx.data(), tx.size()));

  assert(rx[0] == hemerion::sensors::imu::kImuSpiDummyByte);  // address phase
  assert(rx[1] == kImuSpiWhoAmI);
  assert((rx[2] & hemerion::sensors::imu::kImuSpiStatusDataReady) != 0);  // STATUS
  const auto fifo_bytes = static_cast<std::size_t>(rx[3]) | (static_cast<std::size_t>(rx[4]) << 8);
  assert(fifo_bytes == ImuPacketEmitter::kFrameLength);
  assert(fifo_bytes == slave.fifo_used());
}

// The whole point of the exercise: frames buffered by the simulator come back
// out of the FIFO port byte-identical, and decode to the same raw counts.
void test_frames_survive_the_fifo_byte_for_byte()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);

  constexpr std::size_t kSamples = 40;  // several bursts' worth
  std::vector<ImuRawSample> sent;
  for (std::size_t i = 0; i < kSamples; ++i)
  {
    const ImuRawSample raw = make_sample(static_cast<std::int32_t>(i + 1));
    sent.push_back(raw);
    push(slave, raw);
  }
  assert(driver.data_ready());

  std::vector<ImuRawSample> received;
  std::array<ImuRawSample, 8> batch{};
  while (received.size() < kSamples)
  {
    const ImuSpiDriver::PollResult result = driver.poll(batch.data(), batch.size());
    assert(result.error == ImuSpiError::kNone);
    assert(result.checksum_errors == 0);
    assert(result.dropped_samples == 0);
    assert(!result.fifo_overflow);
    if (result.samples == 0)
    {
      break;
    }
    received.insert(received.end(), batch.begin(), batch.begin() + result.samples);
  }

  assert(received.size() == kSamples);
  for (std::size_t i = 0; i < kSamples; ++i)
  {
    assert(received[i].accel_x == sent[i].accel_x);
    assert(received[i].accel_y == sent[i].accel_y);
    assert(received[i].accel_z == sent[i].accel_z);
    assert(received[i].gyro_x == sent[i].gyro_x);
    assert(received[i].gyro_y == sent[i].gyro_y);
    assert(received[i].gyro_z == sent[i].gyro_z);
    assert(received[i].timestamp_us == sent[i].timestamp_us);
  }

  // Drained: no data-ready, and a further poll finds nothing.
  assert(!driver.data_ready());
  assert(driver.poll(batch.data(), batch.size()).samples == 0);
}

// A burst that ends mid-frame must leave the remainder in the FIFO and the
// partial frame in the driver's parser -- the controller has no way to align
// its reads to frame boundaries, so this is the normal case, not an edge one.
void test_frames_split_across_bursts_reassemble()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);

  // 8 frames = 312 bytes, so the driver's 128-byte bursts land mid-frame
  // twice over.
  constexpr std::size_t kSamples = 8;
  for (std::size_t i = 0; i < kSamples; ++i)
  {
    push(slave, make_sample(static_cast<std::int32_t>(100 + i)));
  }

  // One sample of capacity at a time forces the poll budget to cut the FIFO
  // read at a single frame's worth of bytes, out of step with the bursts.
  std::size_t decoded = 0;
  for (std::size_t attempt = 0; attempt < kSamples * 4 && decoded < kSamples; ++attempt)
  {
    ImuRawSample sample;
    const ImuSpiDriver::PollResult result = driver.poll(&sample, 1);
    assert(result.error == ImuSpiError::kNone);
    assert(result.checksum_errors == 0);
    if (result.samples == 1)
    {
      assert(sample.accel_x == static_cast<std::int32_t>(100 + decoded));
      ++decoded;
    }
  }
  assert(decoded == kSamples);
  assert(slave.fifo_used() == 0);
}

// A controller that falls behind loses whole samples, never half a frame, and
// finds out about it through the sticky STATUS bit.
void test_fifo_overflow_drops_whole_frames_and_is_reported()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);

  const auto frame = ImuPacketEmitter::encode_raw_sample(make_sample(7));
  const std::size_t capacity_frames = kImuSpiFifoCapacityBytes / frame.size();
  for (std::size_t i = 0; i < capacity_frames; ++i)
  {
    assert(slave.push_frame(frame.data(), frame.size()));
  }
  // The FIFO now has fewer than one frame of room left.
  assert(!slave.push_frame(frame.data(), frame.size()));
  assert(slave.frames_dropped() == 1);
  assert(slave.fifo_used() == capacity_frames * frame.size());

  std::array<ImuRawSample, 4> batch{};
  const ImuSpiDriver::PollResult result = driver.poll(batch.data(), batch.size());
  assert(result.fifo_overflow);
  assert(result.samples == batch.size());
  assert(result.checksum_errors == 0);

  // Sticky until read: the next poll no longer reports it.
  assert(!driver.poll(batch.data(), batch.size()).fifo_overflow);
}

// CONTROL's reset bit empties the FIFO and reads back clear (it is
// self-clearing), and a disabled FIFO buffers nothing.
void test_control_register_semantics()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  push(slave, make_sample(3));
  assert(slave.fifo_used() > 0);

  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);  // probe writes FIFO_ENABLE | FIFO_RESET
  assert(slave.fifo_used() == 0);

  std::array<std::uint8_t, 2> tx{ hemerion::sensors::imu::imu_spi_command(ImuSpiRegister::kControl, /*read=*/true), 0 };
  std::array<std::uint8_t, 2> rx{};
  assert(bus.transfer(tx.data(), rx.data(), tx.size()));
  assert(rx[1] == hemerion::sensors::imu::kImuSpiControlFifoEnable);  // reset bit did not stick

  // Buffering off: samples are discarded at the source, as on a part whose
  // FIFO was never enabled.
  tx[0] = hemerion::sensors::imu::imu_spi_command(ImuSpiRegister::kControl, /*read=*/false);
  tx[1] = 0;
  assert(bus.transfer(tx.data(), rx.data(), tx.size()));
  const auto frame = ImuPacketEmitter::encode_raw_sample(make_sample(4));
  assert(!slave.push_frame(frame.data(), frame.size()));
  assert(slave.fifo_used() == 0);
  assert(slave.frames_dropped() == 0);  // discarded, not overflowed
}

// End to end with the physics in place: truth -> noise model -> emitter ->
// FIFO -> SPI bursts -> parser -> SI units, which is the path the flight
// software actually runs.
void test_truth_to_si_across_the_bus()
{
  ImuNoiseConfig config;
  config.accel_noise_mps2 = 0.0F;
  config.gyro_noise_rad_s = 0.0F;
  config.accel_bias_sigma_mps2 = 0.0F;
  config.gyro_bias_sigma_rad_s = 0.0F;
  ImuNoiseModel model(config, /*seed=*/11);

  ImuTruthSample truth;
  truth.specific_force_x_mps2 = 62.5;  // boost-phase thrust acceleration
  truth.specific_force_z_mps2 = -1.25;
  truth.angular_rate_y_rad_s = -0.031;
  truth.timestamp_us = 1'234'000;

  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);
  push(slave, model.apply(truth));

  ImuRawSample decoded;
  const ImuSpiDriver::PollResult result = driver.poll(&decoded, 1);
  assert(result.error == ImuSpiError::kNone);
  assert(result.samples == 1);

  ImuSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == ImuConversionError::kNone);

  const double accel_lsb = 9.80665 / static_cast<double>(model.scale().accel_lsb_per_g);
  const double gyro_lsb = (std::numbers::pi / 180.0) / static_cast<double>(model.scale().gyro_lsb_per_dps);
  assert(std::fabs(si.accel_x - truth.specific_force_x_mps2) <= accel_lsb);
  assert(std::fabs(si.accel_z - truth.specific_force_z_mps2) <= accel_lsb);
  assert(std::fabs(si.gyro_y - truth.angular_rate_y_rad_s) <= gyro_lsb);
  assert(si.timestamp_us == truth.timestamp_us);
}

// A bus fault mid-poll is reported rather than silently truncating the stream.
void test_bus_failure_is_reported()
{
  ImuSpiSlave slave;
  DirectSpiBus bus(slave);
  ImuSpiDriver driver(bus);
  assert(driver.probe() == ImuSpiError::kNone);
  push(slave, make_sample(9));

  bus.fail_next_transfer();
  ImuRawSample sample;
  const ImuSpiDriver::PollResult result = driver.poll(&sample, 1);
  assert(result.error == ImuSpiError::kTransferFailed);
  assert(result.samples == 0);
}

}  // namespace

int main()
{
  test_probe_identifies_the_part();
  test_register_reads_auto_increment();
  test_frames_survive_the_fifo_byte_for_byte();
  test_frames_split_across_bursts_reassemble();
  test_fifo_overflow_drops_whole_frames_and_is_reported();
  test_control_register_semantics();
  test_truth_to_si_across_the_bus();
  test_bus_failure_is_reported();

  std::puts("test_imu_spi: all checks passed");
  return 0;
}
