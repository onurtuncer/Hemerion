// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_nmea_parser.cpp
//
// Native unit test for the NMEA 0183 streaming parser. Run by CTest under
// the test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string_view>

#include "Hemerion/gps/nmeaParser.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsFixType;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::NmeaParser;

namespace {

bool near(double a, double b, double tol = 1e-4) {
  return std::fabs(a - b) <= tol;
}

// Feeds every byte of `sentence` plus a trailing "\r\n" into `parser` and
// returns the result of decoding the sentence -- i.e. parse_byte()'s return
// for the '\r' that terminates it. The following '\n' is fed too (receivers
// send CRLF), but by then in_sentence_ is already false, so it's always a
// no-op kIncomplete and isn't what the caller wants back.
GpsParseError feed_sentence(NmeaParser& parser, std::string_view sentence, std::uint64_t timestamp_us, GpsFix& out) {
  for (const char c : sentence) {
    static_cast<void>(parser.parse_byte(static_cast<std::uint8_t>(c), timestamp_us, out));
  }
  const GpsParseError result = parser.parse_byte(static_cast<std::uint8_t>('\r'), timestamp_us, out);
  static_cast<void>(parser.parse_byte(static_cast<std::uint8_t>('\n'), timestamp_us, out));
  return result;
}

void test_gga_decodes_position_altitude_and_satellites() {
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error =
      feed_sentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", 1000, fix);

  assert(error == GpsParseError::kNone);
  assert(near(fix.latitude_deg, 48.1173));
  assert(near(fix.longitude_deg, 11.516667, 1e-5));
  assert(near(fix.altitude_m, 545.4));
  assert(fix.num_satellites == 8);
  assert(fix.fix_type == GpsFixType::kFix3D);
  assert(fix.timestamp_us == 1000);
}

void test_gga_zero_quality_is_no_fix() {
  // Same sentence with quality forced to 0; checksum recomputed for the body
  // "GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,".
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error =
      feed_sentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*46", 0, fix);

  assert(error == GpsParseError::kNone);
  assert(fix.fix_type == GpsFixType::kNoFix);
}

void test_rmc_decodes_speed_and_course() {
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error =
      feed_sentence(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", 2000, fix);

  assert(error == GpsParseError::kNone);
  assert(near(fix.latitude_deg, 48.1173));
  assert(near(fix.longitude_deg, 11.516667, 1e-5));
  assert(near(fix.ground_speed_mps, 22.4 * 0.514444, 1e-3));
  assert(near(fix.course_deg, 84.4, 1e-3));
  assert(fix.fix_type == GpsFixType::kFix3D);
  assert(fix.timestamp_us == 2000);
}

void test_rmc_void_status_is_no_fix() {
  // Status field flipped from 'A' to 'V'; checksum recomputed for the body
  // "GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W".
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error =
      feed_sentence(parser, "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*7D", 0, fix);

  assert(error == GpsParseError::kNone);
  assert(fix.fix_type == GpsFixType::kNoFix);
}

void test_corrupted_checksum_is_rejected() {
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error =
      feed_sentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48", 0, fix);

  assert(error == GpsParseError::kChecksumMismatch);
}

void test_unsupported_sentence_type_is_reported() {
  NmeaParser parser;
  GpsFix fix;
  const GpsParseError error = feed_sentence(parser, "$GPXXX,1,2,3*53", 0, fix);

  assert(error == GpsParseError::kUnsupportedMessage);
}

void test_bytes_outside_a_sentence_are_ignored() {
  NmeaParser parser;
  GpsFix fix;
  assert(parser.parse_byte(static_cast<std::uint8_t>('x'), 0, fix) == GpsParseError::kIncomplete);
  assert(parser.parse_byte(static_cast<std::uint8_t>('\n'), 0, fix) == GpsParseError::kIncomplete);
}

}  // namespace

int main() {
  test_gga_decodes_position_altitude_and_satellites();
  test_gga_zero_quality_is_no_fix();
  test_rmc_decodes_speed_and_course();
  test_rmc_void_status_is_no_fix();
  test_corrupted_checksum_is_rejected();
  test_unsupported_sentence_type_is_reported();
  test_bytes_outside_a_sentence_are_ignored();

  std::puts("test_nmea_parser: all checks passed");
  return 0;
}
