// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_ubx_emitter.cpp
//
// Round-trip test for the GPS FMU simulator: GpsNoiseModel + UbxEmitter
// (modules/sensors/include/Hemerion/gps/fmu/) feeding the real, on-target
// UbxParser (modules/sensors/include/Hemerion/gps/ubxParser.hpp). This is
// the test that actually proves the simulator is byte-compatible with the
// firmware parser, rather than just internally consistent with itself.
//
// Plain asserts + exit code, matching test_ubx_parser.cpp -- Unity is not
// yet vendored (see that file's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>

#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"
#include "Hemerion/gps/fmu/ubxEmitter.hpp"
#include "Hemerion/gps/ubxParser.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsFixType;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::UbxParser;
using hemerion::sensors::gps::fmu::GpsNoiseConfig;
using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;
using hemerion::sensors::gps::fmu::UbxEmitter;

namespace {

bool near(double a, double b, double tol) {
  return std::fabs(a - b) <= tol;
}

GpsParseError feed_frame(UbxParser& parser, const UbxEmitter::Frame& frame, std::uint64_t timestamp_us,
                          GpsFix& out) {
  GpsParseError last_error = GpsParseError::kIncomplete;
  for (const std::uint8_t byte : frame) {
    last_error = parser.parse_byte(byte, timestamp_us, out);
  }
  return last_error;
}

// Noiseless fix straight through the emitter and back through the real
// parser must round-trip to within fixed-point rounding error only.
void test_noiseless_round_trip() {
  GpsFix truth;
  truth.latitude_deg = 41.0082;
  truth.longitude_deg = 28.9784;
  truth.altitude_m = 50.0F;
  truth.ground_speed_mps = 12.5F;
  truth.course_deg = 270.0F;
  truth.horizontal_accuracy_m = 1.5F;
  truth.vertical_accuracy_m = 3.0F;
  truth.num_satellites = 11;
  truth.fix_type = GpsFixType::kFix3D;
  truth.timestamp_us = 123456;

  const auto frame = UbxEmitter::encode_nav_pvt(truth);

  UbxParser parser;
  GpsFix decoded;
  const GpsParseError error = feed_frame(parser, frame, truth.timestamp_us, decoded);

  assert(error == GpsParseError::kNone);
  assert(near(decoded.latitude_deg, truth.latitude_deg, 1e-6));
  assert(near(decoded.longitude_deg, truth.longitude_deg, 1e-6));
  assert(near(decoded.altitude_m, truth.altitude_m, 1e-3));
  assert(near(decoded.ground_speed_mps, truth.ground_speed_mps, 1e-2));
  assert(near(decoded.course_deg, truth.course_deg, 1e-3));
  assert(near(decoded.horizontal_accuracy_m, truth.horizontal_accuracy_m, 1e-3));
  assert(near(decoded.vertical_accuracy_m, truth.vertical_accuracy_m, 1e-3));
  assert(decoded.num_satellites == truth.num_satellites);
  assert(decoded.fix_type == truth.fix_type);
  assert(decoded.timestamp_us == truth.timestamp_us);
}

// A noisy fix from GpsNoiseModel must still round-trip through the emitter
// and parser, and the decoded position must land within a generous
// (5-sigma) bound of truth -- a statistical check with a fixed seed so it
// isn't flaky.
void test_noisy_round_trip_stays_bounded() {
  GpsTruthSample truth;
  truth.latitude_deg = 41.0082;
  truth.longitude_deg = 28.9784;
  truth.altitude_m = 100.0F;
  truth.ground_speed_mps = 5.0F;
  truth.course_deg = 45.0F;
  truth.timestamp_us = 7000;

  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 1.5F;
  config.vertical_pos_noise_m = 3.0F;

  GpsNoiseModel noise_model(config, /*seed=*/42);
  const GpsFix noisy_fix = noise_model.apply(truth);
  const auto frame = UbxEmitter::encode_nav_pvt(noisy_fix);

  UbxParser parser;
  GpsFix decoded;
  const GpsParseError error = feed_frame(parser, frame, truth.timestamp_us, decoded);

  assert(error == GpsParseError::kNone);
  // 1 deg of latitude is ~111.32 km; 5 sigma at 1.5 m noise is well under 1 m.
  constexpr double kMetersPerDegLat = 111320.0;
  const double lat_error_m = std::fabs(decoded.latitude_deg - truth.latitude_deg) * kMetersPerDegLat;
  assert(lat_error_m < 5.0 * static_cast<double>(config.horizontal_pos_noise_m));
  assert(near(decoded.horizontal_accuracy_m, config.horizontal_pos_noise_m, 1e-3));
  assert(near(decoded.vertical_accuracy_m, config.vertical_pos_noise_m, 1e-3));
  assert(decoded.fix_type == GpsFixType::kFix3D);
  assert(decoded.timestamp_us == truth.timestamp_us);
}

}  // namespace

int main() {
  test_noiseless_round_trip();
  test_noisy_round_trip_stays_bounded();

  std::puts("test_ubx_emitter: all checks passed");
  return 0;
}
