// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_baro_packet.cpp
//
// Round-trip test for the barometer FMU simulator: BaroNoiseModel +
// BaroPacketEmitter (modules/sensors/include/Hemerion/baro/fmu/) feeding the
// real, on-target BaroPacketParser
// (modules/sensors/include/Hemerion/baro/baro_packet.h) and
// convert_raw_to_si(). This is the test that actually proves the simulator
// is byte-compatible with the firmware parser, rather than just internally
// consistent with itself. Also pins the ISA layers of the noise model to
// their textbook anchor points.
//
// Plain asserts + exit code, matching test_imu_packet.cpp -- Unity is not
// yet vendored (see test_ubx_parser.cpp's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>

#include "Hemerion/baro/baro_conversion.h"
#include "Hemerion/baro/baro_packet.h"
#include "Hemerion/baro/fmu/baro_noise_model.h"
#include "Hemerion/baro/fmu/baro_packet_emitter.h"

using hemerion::sensors::baro::BaroConversionError;
using hemerion::sensors::baro::BaroPacketError;
using hemerion::sensors::baro::BaroPacketParser;
using hemerion::sensors::baro::BaroRawSample;
using hemerion::sensors::baro::BaroSample;
using hemerion::sensors::baro::convert_raw_to_si;
using hemerion::sensors::baro::fmu::BaroNoiseConfig;
using hemerion::sensors::baro::fmu::BaroNoiseModel;
using hemerion::sensors::baro::fmu::BaroPacketEmitter;
using hemerion::sensors::baro::fmu::BaroTruthSample;

namespace
{

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

BaroPacketError feed_frame(BaroPacketParser& parser, const BaroPacketEmitter::Frame& frame, BaroRawSample& out)
{
  BaroPacketError last_error = BaroPacketError::kIncomplete;
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
  BaroRawSample raw;
  raw.pressure = 101325;
  raw.temperature = -4000;  // -40 C at 100 LSB/C
  raw.timestamp_us = 0x0123456789ABCDEFULL;

  const auto frame = BaroPacketEmitter::encode_raw_sample(raw);

  BaroPacketParser parser;
  BaroRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == BaroPacketError::kNone);
  assert(decoded.pressure == raw.pressure);
  assert(decoded.temperature == raw.temperature);
  assert(decoded.timestamp_us == raw.timestamp_us);
}

// The ISA layers must hit their textbook anchor points: 101325 Pa / 15 C at
// sea level, ~22632 Pa / -56.5 C at the 11 km tropopause, and continuity
// plus monotonic decay across and above it.
void test_isa_anchor_points()
{
  assert(near(BaroNoiseModel::isa_pressure_pa(0.0), 101325.0, 0.5));
  assert(near(BaroNoiseModel::isa_temperature_c(0.0), 15.0, 0.01));

  assert(near(BaroNoiseModel::isa_pressure_pa(11000.0), 22632.0, 5.0));
  assert(near(BaroNoiseModel::isa_temperature_c(11000.0), -56.5, 0.01));

  // Continuity across the tropopause (the physical gradient there is
  // ~3.6 Pa/m, so keep the straddle tight) and monotonic decrease with
  // altitude.
  assert(near(BaroNoiseModel::isa_pressure_pa(10999.9), BaroNoiseModel::isa_pressure_pa(11000.1), 2.0));
  double previous = BaroNoiseModel::isa_pressure_pa(0.0);
  for (double h = 1000.0; h <= 30000.0; h += 1000.0)
  {
    const double p = BaroNoiseModel::isa_pressure_pa(h);
    assert(p < previous);
    assert(p > 0.0);
    previous = p;
  }
}

// A noiseless noise model must reduce to ISA + pure quantization: truth
// altitude, through counts, back through the on-target convert_raw_to_si(),
// recovers the ISA pressure/temperature to within half an LSB.
void test_noiseless_quantization_round_trips_through_conversion()
{
  BaroNoiseConfig config;
  config.pressure_noise_pa = 0.0F;
  config.temperature_noise_c = 0.0F;
  config.pressure_bias_sigma_pa = 0.0F;
  config.temperature_bias_sigma_c = 0.0F;

  BaroTruthSample truth;
  truth.altitude_m = 5000.0;  // mid-troposphere, boost phase
  truth.timestamp_us = 250000;

  BaroNoiseModel model(config, /*seed=*/42);
  const BaroRawSample raw = model.apply(truth);
  const auto frame = BaroPacketEmitter::encode_raw_sample(raw);

  BaroPacketParser parser;
  BaroRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == BaroPacketError::kNone);

  BaroSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == BaroConversionError::kNone);

  const double pressure_lsb = 1.0 / static_cast<double>(model.scale().pressure_lsb_per_pa);
  const double temperature_lsb = 1.0 / static_cast<double>(model.scale().temperature_lsb_per_c);
  assert(near(si.pressure_pa, BaroNoiseModel::isa_pressure_pa(truth.altitude_m), pressure_lsb));
  assert(near(si.temperature_c, BaroNoiseModel::isa_temperature_c(truth.altitude_m), temperature_lsb));
  assert(si.timestamp_us == truth.timestamp_us);
}

// A noisy sample must still round-trip, and the recovered value must land
// within a generous (5-sigma plus quantization) bound of the ISA truth -- a
// statistical check with a fixed seed so it isn't flaky.
void test_noisy_sample_stays_bounded()
{
  BaroNoiseConfig config;  // defaults: bias + white noise on
  BaroTruthSample truth;
  truth.altitude_m = 1500.0;
  truth.timestamp_us = 1000;

  BaroNoiseModel model(config, /*seed=*/7);
  const auto frame = BaroPacketEmitter::encode_raw_sample(model.apply(truth));

  BaroPacketParser parser;
  BaroRawSample decoded;
  assert(feed_frame(parser, frame, decoded) == BaroPacketError::kNone);

  BaroSample si;
  assert(convert_raw_to_si(decoded, model.scale(), si) == BaroConversionError::kNone);

  const double pressure_bound =
      5.0 * static_cast<double>(config.pressure_noise_pa + config.pressure_bias_sigma_pa) + 1.0;
  const double temperature_bound =
      5.0 * static_cast<double>(config.temperature_noise_c + config.temperature_bias_sigma_c) + 0.01;
  assert(near(si.pressure_pa, BaroNoiseModel::isa_pressure_pa(truth.altitude_m), pressure_bound));
  assert(near(si.temperature_c, BaroNoiseModel::isa_temperature_c(truth.altitude_m), temperature_bound));
}

// A corrupted byte must be rejected by checksum, and the parser must then
// recover the next intact frame from the stream.
void test_corruption_is_rejected_and_parser_resyncs()
{
  BaroRawSample raw;
  raw.pressure = 100;
  raw.timestamp_us = 5;
  auto corrupt = BaroPacketEmitter::encode_raw_sample(raw);
  corrupt[10] ^= 0xFFU;  // flip one payload byte

  BaroPacketParser parser;
  BaroRawSample decoded;
  BaroPacketError last_error = BaroPacketError::kIncomplete;
  bool saw_checksum_error = false;
  for (const std::uint8_t byte : corrupt)
  {
    last_error = parser.parse_byte(byte, decoded);
    saw_checksum_error = saw_checksum_error || (last_error == BaroPacketError::kChecksumMismatch);
  }
  assert(saw_checksum_error);
  assert(last_error != BaroPacketError::kNone);

  // Garbage between frames (including a lone sync1) must not prevent the
  // next intact frame from decoding.
  BaroRawSample next;
  next.temperature = -42;
  next.timestamp_us = 6;
  for (const std::uint8_t byte : { 0x00, 0xB7, 0x13, 0xFF })
  {
    (void)parser.parse_byte(static_cast<std::uint8_t>(byte), decoded);
  }
  assert(feed_frame(parser, BaroPacketEmitter::encode_raw_sample(next), decoded) == BaroPacketError::kNone);
  assert(decoded.temperature == next.temperature);
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

  BaroPacketParser parser;
  BaroRawSample decoded;
  BaroPacketError last_error = BaroPacketError::kIncomplete;
  (void)parser.parse_byte(0xB7, decoded);
  (void)parser.parse_byte(0x7B, decoded);
  for (const std::uint8_t byte : body)
  {
    last_error = parser.parse_byte(byte, decoded);
  }
  (void)parser.parse_byte(ck_a, decoded);
  last_error = parser.parse_byte(ck_b, decoded);
  assert(last_error == BaroPacketError::kUnsupportedMessage);

  // The parser must be back in sync for the next real frame.
  BaroRawSample raw;
  raw.pressure = 7;
  assert(feed_frame(parser, BaroPacketEmitter::encode_raw_sample(raw), decoded) == BaroPacketError::kNone);
  assert(decoded.pressure == 7);
}

}  // namespace

int main()
{
  test_raw_counts_round_trip_exactly();
  test_isa_anchor_points();
  test_noiseless_quantization_round_trips_through_conversion();
  test_noisy_sample_stays_bounded();
  test_corruption_is_rejected_and_parser_resyncs();
  test_unknown_message_is_skipped();

  std::puts("test_baro_packet: all checks passed");
  return 0;
}
