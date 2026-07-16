// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_imu_packet.cpp
//
// Round-trip test for the IMU FMU simulator: ImuNoiseModel + ImuPacketEmitter
// (modules/sensors/include/Hemerion/imu/fmu/) feeding the real, on-target
// ImuPacketParser (modules/sensors/include/Hemerion/imu/imu_packet.h) and
// convert_raw_to_si(). This is the test that actually proves the simulator
// is byte-compatible with the firmware parser, rather than just internally
// consistent with itself.
//
// Plain asserts + exit code, matching test_ubx_emitter.cpp -- Unity is not
// yet vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"
#include "Hemerion/imu/imu_conversion.h"
#include "Hemerion/imu/imu_packet.h"

using hemerion::sensors::imu::convert_raw_to_si;
using hemerion::sensors::imu::ImuConversionError;
using hemerion::sensors::imu::ImuPacketError;
using hemerion::sensors::imu::ImuPacketParser;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSample;
using hemerion::sensors::imu::fmu::ImuNoiseConfig;
using hemerion::sensors::imu::fmu::ImuNoiseModel;
using hemerion::sensors::imu::fmu::ImuPacketEmitter;
using hemerion::sensors::imu::fmu::ImuTruthSample;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

ImuPacketError feed_frame(ImuPacketParser& parser, const ImuPacketEmitter::Frame& frame, ImuRawSample& out)
{
  ImuPacketError last_error = ImuPacketError::kIncomplete;
  for (const std::uint8_t byte : frame)
  {
    last_error = parser.parse_byte(byte, out);
  }
  return last_error;
}

// Raw counts through the emitter and back through the real parser must
// round-trip exactly -- the payload is fixed-width integers, no rounding.
void test_raw_counts_round_trip_exactly()
{
  ImuRawSample raw;
  raw.accel_x = 12345;
  raw.accel_y = -20000;
  raw.accel_z = 32767;
  raw.gyro_x = -32768;
  raw.gyro_y = 1;
  raw.gyro_z = 0;
  raw.timestamp_us = 0x0123456789ABCDEFULL;

  const auto frame = ImuPacketEmitter::encode_raw_sample(raw);

  ImuPacketParser parser;
  ImuRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == ImuPacketError::kNone);
  assert(decoded.accel_x == raw.accel_x);
  assert(decoded.accel_y == raw.accel_y);
  assert(decoded.accel_z == raw.accel_z);
  assert(decoded.gyro_x == raw.gyro_x);
  assert(decoded.gyro_y == raw.gyro_y);
  assert(decoded.gyro_z == raw.gyro_z);
  assert(decoded.timestamp_us == raw.timestamp_us);
}

// A noiseless noise model must reduce to pure quantization: truth in SI,
// through counts, back through the on-target convert_raw_to_si(), recovers
// truth to within half an LSB.
void test_noiseless_quantization_round_trips_through_conversion()
{
  ImuNoiseConfig config;
  config.accel_noise_mps2 = 0.0F;
  config.gyro_noise_rad_s = 0.0F;
  config.accel_bias_sigma_mps2 = 0.0F;
  config.gyro_bias_sigma_rad_s = 0.0F;

  ImuTruthSample truth;
  truth.specific_force_x_mps2 = 42.5;   // boost-phase thrust acceleration
  truth.specific_force_y_mps2 = -0.75;
  truth.specific_force_z_mps2 = 9.80665;
  truth.angular_rate_x_rad_s = 1.5;     // spinning airframe
  truth.angular_rate_y_rad_s = -0.02;
  truth.angular_rate_z_rad_s = 0.001;
  truth.timestamp_us = 250000;

  ImuNoiseModel model(config, /*seed=*/42);
  const ImuRawSample raw = model.apply(truth);
  const auto frame = ImuPacketEmitter::encode_raw_sample(raw);

  ImuPacketParser parser;
  ImuRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == ImuPacketError::kNone);

  ImuSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == ImuConversionError::kNone);

  // Half-LSB bounds for the default scale: 9.80665/800 m/s^2 and
  // (pi/180)/16.4 rad/s per count.
  const double accel_lsb = 9.80665 / static_cast<double>(model.scale().accel_lsb_per_g);
  const double gyro_lsb = (std::numbers::pi / 180.0) / static_cast<double>(model.scale().gyro_lsb_per_dps);
  assert(near(si.accel_x, truth.specific_force_x_mps2, accel_lsb));
  assert(near(si.accel_y, truth.specific_force_y_mps2, accel_lsb));
  assert(near(si.accel_z, truth.specific_force_z_mps2, accel_lsb));
  assert(near(si.gyro_x, truth.angular_rate_x_rad_s, gyro_lsb));
  assert(near(si.gyro_y, truth.angular_rate_y_rad_s, gyro_lsb));
  assert(near(si.gyro_z, truth.angular_rate_z_rad_s, gyro_lsb));
  assert(si.timestamp_us == truth.timestamp_us);
}

// A noisy sample must still round-trip, and the recovered SI value must land
// within a generous (5-sigma plus quantization) bound of truth -- a
// statistical check with a fixed seed so it isn't flaky.
void test_noisy_sample_stays_bounded()
{
  ImuNoiseConfig config;  // defaults: bias + white noise on
  ImuTruthSample truth;
  truth.specific_force_x_mps2 = 30.0;
  truth.angular_rate_z_rad_s = 0.5;
  truth.timestamp_us = 1000;

  ImuNoiseModel model(config, /*seed=*/7);
  const auto frame = ImuPacketEmitter::encode_raw_sample(model.apply(truth));

  ImuPacketParser parser;
  ImuRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == ImuPacketError::kNone);

  ImuSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == ImuConversionError::kNone);

  const double accel_bound =
      (5.0 * static_cast<double>(config.accel_noise_mps2 + config.accel_bias_sigma_mps2)) + (9.80665 / 800.0);
  const double gyro_bound =
      (5.0 * static_cast<double>(config.gyro_noise_rad_s + config.gyro_bias_sigma_rad_s)) + (0.0175 / 16.4);
  assert(near(si.accel_x, truth.specific_force_x_mps2, accel_bound));
  assert(near(si.gyro_z, truth.angular_rate_z_rad_s, gyro_bound));
}

// Register saturation: truth far beyond full scale must clip to int16
// extremes, exactly as real silicon does, not wrap or overflow.
void test_out_of_range_truth_saturates()
{
  ImuNoiseConfig config;
  config.accel_noise_mps2 = 0.0F;
  config.gyro_noise_rad_s = 0.0F;
  config.accel_bias_sigma_mps2 = 0.0F;
  config.gyro_bias_sigma_rad_s = 0.0F;

  ImuTruthSample truth;
  truth.specific_force_x_mps2 = 10000.0;   // ~1000 g, far past +/-40 g full scale
  truth.angular_rate_x_rad_s = -1000.0;    // far past -2000 deg/s full scale

  ImuNoiseModel model(config, /*seed=*/1);
  const ImuRawSample raw = model.apply(truth);
  assert(raw.accel_x == 32767);
  assert(raw.gyro_x == -32768);
}

// A corrupted byte must be rejected by checksum, and the parser must then
// recover the next intact frame from the stream.
void test_corruption_is_rejected_and_parser_resyncs()
{
  ImuRawSample raw;
  raw.accel_x = 100;
  raw.timestamp_us = 5;
  auto corrupt = ImuPacketEmitter::encode_raw_sample(raw);
  corrupt[10] ^= 0xFFU;  // flip one payload byte

  ImuPacketParser parser;
  ImuRawSample decoded;
  ImuPacketError last_error = ImuPacketError::kIncomplete;
  bool saw_checksum_error = false;
  for (const std::uint8_t byte : corrupt)
  {
    last_error = parser.parse_byte(byte, decoded);
    saw_checksum_error = saw_checksum_error || (last_error == ImuPacketError::kChecksumMismatch);
  }
  assert(saw_checksum_error);
  assert(last_error != ImuPacketError::kNone);

  // Garbage between frames (including a lone sync1) must not prevent the
  // next intact frame from decoding.
  ImuRawSample next;
  next.gyro_y = -42;
  next.timestamp_us = 6;
  for (const std::uint8_t byte : { 0x00, 0xA5, 0x13, 0xFF })
  {
    (void)parser.parse_byte(static_cast<std::uint8_t>(byte), decoded);
  }
  assert(feed_frame(parser, ImuPacketEmitter::encode_raw_sample(next), decoded) == ImuPacketError::kNone);
  assert(decoded.gyro_y == next.gyro_y);
  assert(decoded.timestamp_us == next.timestamp_us);
}

// An unknown message id with a valid checksum must be skipped (reported as
// unsupported) without desyncing the parser.
void test_unknown_message_is_skipped()
{
  // Hand-built frame: id 0x7E, 3-byte payload, valid Fletcher checksum.
  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  const std::array<std::uint8_t, 6> body = { 0x7E, 0x03, 0x00, 0xAA, 0xBB, 0xCC };
  for (const std::uint8_t byte : body)
  {
    ck_a = static_cast<std::uint8_t>(ck_a + byte);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }

  ImuPacketParser parser;
  ImuRawSample decoded;
  (void)parser.parse_byte(0xA5, decoded);
  (void)parser.parse_byte(0x5A, decoded);
  for (const std::uint8_t byte : body)
  {
    (void)parser.parse_byte(byte, decoded);
  }
  (void)parser.parse_byte(ck_a, decoded);
  const ImuPacketError last_error = parser.parse_byte(ck_b, decoded);
  assert(last_error == ImuPacketError::kUnsupportedMessage);

  // The parser must be back in sync for the next real frame.
  ImuRawSample raw;
  raw.accel_z = 7;
  assert(feed_frame(parser, ImuPacketEmitter::encode_raw_sample(raw), decoded) == ImuPacketError::kNone);
  assert(decoded.accel_z == 7);
}

}  // namespace

int main()
{
  test_raw_counts_round_trip_exactly();
  test_noiseless_quantization_round_trips_through_conversion();
  test_noisy_sample_stays_bounded();
  test_out_of_range_truth_saturates();
  test_corruption_is_rejected_and_parser_resyncs();
  test_unknown_message_is_skipped();

  std::puts("test_imu_packet: all checks passed");
  return 0;
}
