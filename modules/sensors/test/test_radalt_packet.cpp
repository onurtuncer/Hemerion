// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_radalt_packet.cpp
//
// Round-trip test for the radar altimeter FMU simulator: RadAltNoiseModel +
// RadAltPacketEmitter (modules/sensors/include/Hemerion/radalt/fmu/) feeding
// the real, on-target RadAltPacketParser
// (modules/sensors/include/Hemerion/radalt/radalt_packet.h) and
// convert_raw_to_si(). This is the test that actually proves the simulator
// is byte-compatible with the firmware parser, rather than just internally
// consistent with itself. Also pins the loss-of-track behaviour beyond max
// range.
//
// Plain asserts + exit code, matching test_imu_packet.cpp -- Unity is not
// yet vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>

#include "Hemerion/radalt/fmu/radalt_noise_model.h"
#include "Hemerion/radalt/fmu/radalt_packet_emitter.h"
#include "Hemerion/radalt/radalt_conversion.h"
#include "Hemerion/radalt/radalt_packet.h"

using hemerion::sensors::radalt::convert_raw_to_si;
using hemerion::sensors::radalt::kRadAltStatusNoReturn;
using hemerion::sensors::radalt::kRadAltStatusTrackValid;
using hemerion::sensors::radalt::RadAltConversionError;
using hemerion::sensors::radalt::RadAltPacketError;
using hemerion::sensors::radalt::RadAltPacketParser;
using hemerion::sensors::radalt::RadAltRawSample;
using hemerion::sensors::radalt::RadAltSample;
using hemerion::sensors::radalt::fmu::RadAltNoiseConfig;
using hemerion::sensors::radalt::fmu::RadAltNoiseModel;
using hemerion::sensors::radalt::fmu::RadAltPacketEmitter;
using hemerion::sensors::radalt::fmu::RadAltTruthSample;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

RadAltPacketError feed_frame(RadAltPacketParser& parser, const RadAltPacketEmitter::Frame& frame, RadAltRawSample& out)
{
  RadAltPacketError last_error = RadAltPacketError::kIncomplete;
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
  RadAltRawSample raw;
  raw.range = 600000;  // 6000 m at 100 LSB/m
  raw.status = kRadAltStatusTrackValid;
  raw.timestamp_us = 0x0123456789ABCDEFULL;

  const auto frame = RadAltPacketEmitter::encode_raw_sample(raw);

  RadAltPacketParser parser;
  RadAltRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == RadAltPacketError::kNone);
  assert(decoded.range == raw.range);
  assert(decoded.status == raw.status);
  assert(decoded.timestamp_us == raw.timestamp_us);
}

// A noiseless noise model must reduce to pure quantization: truth height,
// through counts, back through the on-target convert_raw_to_si(), recovers
// truth to within half an LSB.
void test_noiseless_quantization_round_trips_through_conversion()
{
  RadAltNoiseConfig config;
  config.range_noise_m = 0.0F;
  config.range_bias_sigma_m = 0.0F;

  RadAltTruthSample truth;
  truth.height_agl_m = 1234.567;
  truth.timestamp_us = 250000;

  RadAltNoiseModel model(config, /*seed=*/42);
  const RadAltRawSample raw = model.apply(truth);
  const auto frame = RadAltPacketEmitter::encode_raw_sample(raw);

  RadAltPacketParser parser;
  RadAltRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == RadAltPacketError::kNone);

  RadAltSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == RadAltConversionError::kNone);
  assert(si.valid);

  const double range_lsb = 1.0 / static_cast<double>(model.scale().range_lsb_per_m);
  assert(near(si.range_m, truth.height_agl_m, range_lsb));
  assert(si.timestamp_us == truth.timestamp_us);
}

// A noisy sample must still round-trip, and the recovered range must land
// within a generous (5-sigma plus quantization) bound of truth -- a
// statistical check with a fixed seed so it isn't flaky.
void test_noisy_sample_stays_bounded()
{
  RadAltNoiseConfig config;  // defaults: bias + white noise on
  RadAltTruthSample truth;
  truth.height_agl_m = 300.0;  // final descent
  truth.timestamp_us = 1000;

  RadAltNoiseModel model(config, /*seed=*/7);
  const auto frame = RadAltPacketEmitter::encode_raw_sample(model.apply(truth));

  RadAltPacketParser parser;
  RadAltRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == RadAltPacketError::kNone);

  RadAltSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == RadAltConversionError::kNone);
  assert(si.valid);

  const double range_bound = 5.0 * static_cast<double>(config.range_noise_m + config.range_bias_sigma_m) + 0.01;
  assert(near(si.range_m, truth.height_agl_m, range_bound));
}

// Beyond max tracking range (and below zero height) the model must report a
// no-return sample, and the conversion must decode it as invalid -- not an
// error, and not a bogus range.
void test_out_of_range_reports_no_return()
{
  RadAltNoiseConfig config;
  RadAltNoiseModel model(config, /*seed=*/1);

  RadAltTruthSample truth;
  truth.height_agl_m = static_cast<double>(config.max_range_m) + 1000.0;  // above max range
  truth.timestamp_us = 42;

  RadAltRawSample raw = model.apply(truth);
  assert(raw.status == kRadAltStatusNoReturn);
  assert(raw.range == 0);

  RadAltSample si;
  assert(convert_raw_to_si(raw, model.scale(), si) == RadAltConversionError::kNone);
  assert(!si.valid);
  assert(si.range_m == 0.0F);
  assert(si.timestamp_us == truth.timestamp_us);

  truth.height_agl_m = -5.0;  // below the ground plane
  raw = model.apply(truth);
  assert(raw.status == kRadAltStatusNoReturn);
}

// A corrupted byte must be rejected by checksum, and the parser must then
// recover the next intact frame from the stream.
void test_corruption_is_rejected_and_parser_resyncs()
{
  RadAltRawSample raw;
  raw.range = 100;
  raw.status = kRadAltStatusTrackValid;
  raw.timestamp_us = 5;
  auto corrupt = RadAltPacketEmitter::encode_raw_sample(raw);
  corrupt[10] ^= 0xFFU;  // flip one payload byte

  RadAltPacketParser parser;
  RadAltRawSample decoded;
  RadAltPacketError last_error = RadAltPacketError::kIncomplete;
  bool saw_checksum_error = false;
  for (const std::uint8_t byte : corrupt)
  {
    last_error = parser.parse_byte(byte, decoded);
    saw_checksum_error = saw_checksum_error || (last_error == RadAltPacketError::kChecksumMismatch);
  }
  assert(saw_checksum_error);
  assert(last_error != RadAltPacketError::kNone);

  // Garbage between frames (including a lone sync1) must not prevent the
  // next intact frame from decoding.
  RadAltRawSample next;
  next.range = 4242;
  next.status = kRadAltStatusTrackValid;
  next.timestamp_us = 6;
  for (const std::uint8_t byte : { 0x00, 0xC9, 0x13, 0xFF })
  {
    (void)parser.parse_byte(static_cast<std::uint8_t>(byte), decoded);
  }
  assert(feed_frame(parser, RadAltPacketEmitter::encode_raw_sample(next), decoded) == RadAltPacketError::kNone);
  assert(decoded.range == next.range);
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

  RadAltPacketParser parser;
  RadAltRawSample decoded;
  RadAltPacketError last_error = RadAltPacketError::kIncomplete;
  (void)parser.parse_byte(0xC9, decoded);
  (void)parser.parse_byte(0x9C, decoded);
  for (const std::uint8_t byte : body)
  {
    last_error = parser.parse_byte(byte, decoded);
  }
  (void)parser.parse_byte(ck_a, decoded);
  last_error = parser.parse_byte(ck_b, decoded);
  assert(last_error == RadAltPacketError::kUnsupportedMessage);

  // The parser must be back in sync for the next real frame.
  RadAltRawSample raw;
  raw.range = 7;
  raw.status = kRadAltStatusTrackValid;
  assert(feed_frame(parser, RadAltPacketEmitter::encode_raw_sample(raw), decoded) == RadAltPacketError::kNone);
  assert(decoded.range == 7);
}

}  // namespace

int main()
{
  test_raw_counts_round_trip_exactly();
  test_noiseless_quantization_round_trips_through_conversion();
  test_noisy_sample_stays_bounded();
  test_out_of_range_reports_no_return();
  test_corruption_is_rejected_and_parser_resyncs();
  test_unknown_message_is_skipped();

  std::puts("test_radalt_packet: all checks passed");
  return 0;
}
