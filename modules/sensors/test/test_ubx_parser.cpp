// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_ubx_parser.cpp
//
// Native unit test for the UBX-NAV-PVT streaming parser. Run by CTest under
// the test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
//
// The UBX-NAV-PVT message below was synthesized (not captured from a real
// receiver): fixType=3 (3D), numSV=9, lat=41.0082000 deg, lon=28.9784000 deg
// (Istanbul), height=50.000 m, hAcc=2.5 m, vAcc=4.0 m, gSpeed=1.5 m/s,
// headMot=90.00000 deg; all other fields zero. CK_A/CK_B were computed for
// this exact payload.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

#include "Hemerion/gps/ubxParser.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsFixType;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::UbxParser;

namespace {

bool near(float a, float b, float tol = 1e-4F) {
  return std::fabs(a - b) <= tol;
}

const std::vector<std::uint8_t>& nav_pvt_payload() {
  static const std::vector<std::uint8_t> payload = {
      0x00, 0x00, 0x00, 0x00, 0xEA, 0x07, 0x06, 0x1A, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x09, 0xC0, 0xC0, 0x45, 0x11, 0xD0, 0x5A, 0x71, 0x18, 0x50, 0xC3,
      0x00, 0x00, 0xC8, 0xAF, 0x00, 0x00, 0xC4, 0x09, 0x00, 0x00, 0xA0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x05, 0x00, 0x00, 0x40, 0x54, 0x89, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };
  return payload;
}

std::vector<std::uint8_t> build_message(std::uint8_t msg_class, std::uint8_t msg_id,
                                         const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> message = {0xB5, 0x62, msg_class, msg_id,
                                        static_cast<std::uint8_t>(payload.size() & 0xFFU),
                                        static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU)};
  message.insert(message.end(), payload.begin(), payload.end());

  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  for (std::size_t i = 2; i < message.size(); ++i) {
    ck_a = static_cast<std::uint8_t>(ck_a + message[i]);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }
  message.push_back(ck_a);
  message.push_back(ck_b);
  return message;
}

GpsParseError feed_message(UbxParser& parser, const std::vector<std::uint8_t>& message, std::uint64_t timestamp_us,
                            GpsFix& out) {
  GpsParseError last_error = GpsParseError::kIncomplete;
  for (const std::uint8_t byte : message) {
    last_error = parser.parse_byte(byte, timestamp_us, out);
  }
  return last_error;
}

void test_nav_pvt_decodes_full_fix() {
  UbxParser parser;
  GpsFix fix;
  const auto message = build_message(0x01, 0x07, nav_pvt_payload());
  const GpsParseError error = feed_message(parser, message, 5000, fix);

  assert(error == GpsParseError::kNone);
  assert(near(static_cast<float>(fix.latitude_deg), 41.0082F, 1e-3F));
  assert(near(static_cast<float>(fix.longitude_deg), 28.9784F, 1e-3F));
  assert(near(fix.altitude_m, 50.0F));
  assert(near(fix.ground_speed_mps, 1.5F));
  assert(near(fix.course_deg, 90.0F));
  assert(near(fix.horizontal_accuracy_m, 2.5F));
  assert(near(fix.vertical_accuracy_m, 4.0F));
  assert(fix.num_satellites == 9);
  assert(fix.fix_type == GpsFixType::kFix3D);
  assert(fix.timestamp_us == 5000);
}

void test_corrupted_checksum_is_rejected() {
  UbxParser parser;
  GpsFix fix;
  auto message = build_message(0x01, 0x07, nav_pvt_payload());
  message.back() = static_cast<std::uint8_t>(message.back() + 1);  // corrupt CK_B
  const GpsParseError error = feed_message(parser, message, 0, fix);

  assert(error == GpsParseError::kChecksumMismatch);
}

void test_other_message_class_is_unsupported() {
  UbxParser parser;
  GpsFix fix;
  const auto message = build_message(0x01, 0x02, {});  // UBX-NAV-POSLLH, empty payload for this test
  const GpsParseError error = feed_message(parser, message, 0, fix);

  assert(error == GpsParseError::kUnsupportedMessage);
}

void test_partial_message_is_incomplete() {
  UbxParser parser;
  GpsFix fix;
  const std::vector<std::uint8_t> partial = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00};
  const GpsParseError error = feed_message(parser, partial, 0, fix);

  assert(error == GpsParseError::kIncomplete);
}

}  // namespace

int main() {
  test_nav_pvt_decodes_full_fix();
  test_corrupted_checksum_is_rejected();
  test_other_message_class_is_unsupported();
  test_partial_message_is_incomplete();

  std::puts("test_ubx_parser: all checks passed");
  return 0;
}
