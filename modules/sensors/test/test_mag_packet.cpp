// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_mag_packet.cpp
//
// Round-trip test for the magnetometer FMU simulator: MagNoiseModel +
// MagPacketEmitter (modules/sensors/include/Hemerion/mag/fmu/) feeding the
// real, on-target MagPacketParser
// (modules/sensors/include/Hemerion/mag/mag_packet.h) and
// convert_raw_to_si(). This is the test that actually proves the simulator
// is byte-compatible with the firmware parser, rather than just internally
// consistent with itself.
//
// Plain asserts + exit code, matching test_imu_packet.cpp -- Unity is not
// yet vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>

#include "Hemerion/mag/fmu/mag_noise_model.h"
#include "Hemerion/mag/fmu/mag_packet_emitter.h"
#include "Hemerion/mag/mag_conversion.h"
#include "Hemerion/mag/mag_packet.h"

using hemerion::sensors::mag::convert_raw_to_si;
using hemerion::sensors::mag::MagConversionError;
using hemerion::sensors::mag::MagPacketError;
using hemerion::sensors::mag::MagPacketParser;
using hemerion::sensors::mag::MagRawSample;
using hemerion::sensors::mag::MagSample;
using hemerion::sensors::mag::fmu::MagNoiseConfig;
using hemerion::sensors::mag::fmu::MagNoiseModel;
using hemerion::sensors::mag::fmu::MagPacketEmitter;
using hemerion::sensors::mag::fmu::MagTruthSample;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

MagPacketError feed_frame(MagPacketParser& parser, const MagPacketEmitter::Frame& frame, MagRawSample& out)
{
  MagPacketError last_error = MagPacketError::kIncomplete;
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
  MagRawSample raw;
  raw.mag_x = 12345;
  raw.mag_y = -20000;
  raw.mag_z = 32767;
  raw.timestamp_us = 0x0123456789ABCDEFULL;

  const auto frame = MagPacketEmitter::encode_raw_sample(raw);

  MagPacketParser parser;
  MagRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == MagPacketError::kNone);
  assert(decoded.mag_x == raw.mag_x);
  assert(decoded.mag_y == raw.mag_y);
  assert(decoded.mag_z == raw.mag_z);
  assert(decoded.timestamp_us == raw.timestamp_us);
}

// A noiseless noise model must reduce to pure quantization: truth field,
// through counts, back through the on-target convert_raw_to_si(), recovers
// truth to within half an LSB.
void test_noiseless_quantization_round_trips_through_conversion()
{
  MagNoiseConfig config;
  config.mag_noise_ut = 0.0F;
  config.mag_bias_sigma_ut = 0.0F;

  MagTruthSample truth;
  truth.mag_x_ut = 21.3;  // mid-latitude Earth field components
  truth.mag_y_ut = -4.75;
  truth.mag_z_ut = 43.9;
  truth.timestamp_us = 250000;

  MagNoiseModel model(config, /*seed=*/42);
  const MagRawSample raw = model.apply(truth);
  const auto frame = MagPacketEmitter::encode_raw_sample(raw);

  MagPacketParser parser;
  MagRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == MagPacketError::kNone);

  MagSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == MagConversionError::kNone);

  const double mag_lsb = 1.0 / static_cast<double>(model.scale().mag_lsb_per_ut);
  assert(near(si.mag_x_ut, truth.mag_x_ut, mag_lsb));
  assert(near(si.mag_y_ut, truth.mag_y_ut, mag_lsb));
  assert(near(si.mag_z_ut, truth.mag_z_ut, mag_lsb));
  assert(si.timestamp_us == truth.timestamp_us);
}

// A noisy sample must still round-trip, and the recovered field must land
// within a generous (5-sigma plus quantization) bound of truth -- a
// statistical check with a fixed seed so it isn't flaky.
void test_noisy_sample_stays_bounded()
{
  MagNoiseConfig config;  // defaults: bias + white noise on
  MagTruthSample truth;
  truth.mag_x_ut = 30.0;
  truth.mag_z_ut = -45.0;
  truth.timestamp_us = 1000;

  MagNoiseModel model(config, /*seed=*/7);
  const auto frame = MagPacketEmitter::encode_raw_sample(model.apply(truth));

  MagPacketParser parser;
  MagRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == MagPacketError::kNone);

  MagSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == MagConversionError::kNone);

  const double mag_bound = 5.0 * static_cast<double>(config.mag_noise_ut + config.mag_bias_sigma_ut) + 0.01;
  assert(near(si.mag_x_ut, truth.mag_x_ut, mag_bound));
  assert(near(si.mag_z_ut, truth.mag_z_ut, mag_bound));
}

// Register saturation: truth far beyond full scale must clip to int16
// extremes, exactly as real silicon does, not wrap or overflow.
void test_out_of_range_truth_saturates()
{
  MagNoiseConfig config;
  config.mag_noise_ut = 0.0F;
  config.mag_bias_sigma_ut = 0.0F;

  MagTruthSample truth;
  truth.mag_x_ut = 100000.0;  // far past the +/-327 uT full scale
  truth.mag_y_ut = -100000.0;

  MagNoiseModel model(config, /*seed=*/1);
  const MagRawSample raw = model.apply(truth);
  assert(raw.mag_x == 32767);
  assert(raw.mag_y == -32768);
}

// A corrupted byte must be rejected by checksum, and the parser must then
// recover the next intact frame from the stream.
void test_corruption_is_rejected_and_parser_resyncs()
{
  MagRawSample raw;
  raw.mag_x = 100;
  raw.timestamp_us = 5;
  auto corrupt = MagPacketEmitter::encode_raw_sample(raw);
  corrupt[10] ^= 0xFFU;  // flip one payload byte

  MagPacketParser parser;
  MagRawSample decoded;
  MagPacketError last_error = MagPacketError::kIncomplete;
  bool saw_checksum_error = false;
  for (const std::uint8_t byte : corrupt)
  {
    last_error = parser.parse_byte(byte, decoded);
    saw_checksum_error = saw_checksum_error || (last_error == MagPacketError::kChecksumMismatch);
  }
  assert(saw_checksum_error);
  assert(last_error != MagPacketError::kNone);

  // Garbage between frames (including a lone sync1) must not prevent the
  // next intact frame from decoding.
  MagRawSample next;
  next.mag_y = -42;
  next.timestamp_us = 6;
  for (const std::uint8_t byte : { 0x00, 0xD1, 0x13, 0xFF })
  {
    (void)parser.parse_byte(static_cast<std::uint8_t>(byte), decoded);
  }
  assert(feed_frame(parser, MagPacketEmitter::encode_raw_sample(next), decoded) == MagPacketError::kNone);
  assert(decoded.mag_y == next.mag_y);
  assert(decoded.timestamp_us == next.timestamp_us);
}

// An unknown message id with a valid checksum must be skipped (reported as
// unsupported) without desyncing the parser.
void test_unknown_message_is_skipped()
{
  // Hand-built frame: id 0x7E, 3-byte payload, valid Fletcher checksum.
  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  std::uint8_t body[] = { 0x7E, 0x03, 0x00, 0xAA, 0xBB, 0xCC };
  for (const std::uint8_t byte : body)
  {
    ck_a = static_cast<std::uint8_t>(ck_a + byte);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }

  MagPacketParser parser;
  MagRawSample decoded;
  MagPacketError last_error = MagPacketError::kIncomplete;
  (void)parser.parse_byte(0xD1, decoded);
  (void)parser.parse_byte(0x1D, decoded);
  for (const std::uint8_t byte : body)
  {
    last_error = parser.parse_byte(byte, decoded);
  }
  (void)parser.parse_byte(ck_a, decoded);
  last_error = parser.parse_byte(ck_b, decoded);
  assert(last_error == MagPacketError::kUnsupportedMessage);

  // The parser must be back in sync for the next real frame.
  MagRawSample raw;
  raw.mag_z = 7;
  assert(feed_frame(parser, MagPacketEmitter::encode_raw_sample(raw), decoded) == MagPacketError::kNone);
  assert(decoded.mag_z == 7);
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

  std::puts("test_mag_packet: all checks passed");
  return 0;
}
