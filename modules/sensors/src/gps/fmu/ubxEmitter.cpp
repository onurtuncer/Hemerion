// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file ubxEmitter.cpp
/// @brief Implements the UBX-NAV-PVT frame encoder declared in
/// ubxEmitter.hpp.

#include "Hemerion/gps/fmu/ubxEmitter.hpp"

#include <cmath>
#include <numbers>

namespace hemerion::sensors::gps::fmu
{

namespace
{

constexpr std::uint8_t kSync1Byte = 0xB5;
constexpr std::uint8_t kSync2Byte = 0x62;
constexpr std::uint8_t kNavClass = 0x01;
constexpr std::uint8_t kNavPvtId = 0x07;
constexpr double kDegToRad = std::numbers::pi / 180.0;

// Fixed placeholder accuracy/quality fields GpsFix has no source data for.
// UbxParser never reads any of these; they exist only so a hex dump of the
// frame looks like a real receiver's, not because anything downstream
// consumes them.
constexpr std::uint32_t kSpeedAccuracyMmPerS = 200;       // 0.2 m/s
constexpr std::uint32_t kHeadingAccuracy1e5Deg = 200000;  // 2.0 deg
constexpr std::uint16_t kPositionDop = 150;               // 1.50, scale 0.01

void write_u16(UbxEmitter::Frame::value_type* payload, std::size_t offset, std::uint16_t value)
{
  payload[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  payload[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void write_u32(UbxEmitter::Frame::value_type* payload, std::size_t offset, std::uint32_t value)
{
  payload[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  payload[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  payload[offset + 2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  payload[offset + 3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void write_i32(UbxEmitter::Frame::value_type* payload, std::size_t offset, std::int32_t value)
{
  write_u32(payload, offset, static_cast<std::uint32_t>(value));
}

std::uint8_t to_ubx_fix_type(GpsFixType fix_type)
{
  switch (fix_type)
  {
    case GpsFixType::kFix2D:
      return 2;
    case GpsFixType::kFix3D:
      return 3;
    case GpsFixType::kNoFix:
    default:
      return 0;
  }
}

}  // namespace

UbxEmitter::Frame UbxEmitter::encode_nav_pvt(const GpsFix& fix)
{
  Frame frame{};
  frame[0] = kSync1Byte;
  frame[1] = kSync2Byte;
  frame[2] = kNavClass;
  frame[3] = kNavPvtId;
  write_u16(frame.data(), 4, static_cast<std::uint16_t>(kPayloadLength));

  // Payload starts at frame[6]; offsets below match UbxParser::decode_nav_pvt()'s
  // payload-relative offsets exactly.
  std::uint8_t* payload = frame.data() + 6;

  const std::uint8_t fix_type_raw = to_ubx_fix_type(fix.fix_type);
  payload[20] = fix_type_raw;
  payload[21] = (fix.fix_type != GpsFixType::kNoFix) ? 0x01 : 0x00;  // flags: bit0 gnssFixOK
  payload[23] = fix.num_satellites;

  write_i32(payload, 24, static_cast<std::int32_t>(std::lround(fix.longitude_deg * 1e7)));
  write_i32(payload, 28, static_cast<std::int32_t>(std::lround(fix.latitude_deg * 1e7)));

  const auto height_mm = static_cast<std::int32_t>(std::lround(static_cast<double>(fix.altitude_m) * 1000.0));
  write_i32(payload, 32, height_mm);  // height (ellipsoid)
  write_i32(payload, 36, height_mm);  // hMSL -- no geoid model here, reuse ellipsoid height

  write_u32(
      payload, 40, static_cast<std::uint32_t>(std::lround(static_cast<double>(fix.horizontal_accuracy_m) * 1000.0)));
  write_u32(
      payload, 44, static_cast<std::uint32_t>(std::lround(static_cast<double>(fix.vertical_accuracy_m) * 1000.0)));

  const double heading_rad = static_cast<double>(fix.course_deg) * kDegToRad;
  const double ground_speed_mm_s = static_cast<double>(fix.ground_speed_mps) * 1000.0;
  write_i32(payload, 48, static_cast<std::int32_t>(std::lround(ground_speed_mm_s * std::cos(heading_rad))));  // velN
  write_i32(payload, 52, static_cast<std::int32_t>(std::lround(ground_speed_mm_s * std::sin(heading_rad))));  // velE
  write_i32(payload, 56, 0);                                                                                  // velD

  write_i32(payload, 60, static_cast<std::int32_t>(std::lround(ground_speed_mm_s)));  // gSpeed
  write_i32(payload, 64,
            static_cast<std::int32_t>(std::lround(static_cast<double>(fix.course_deg) * 1e5)));  // headMot

  write_u32(payload, 68, kSpeedAccuracyMmPerS);
  write_u32(payload, 72, kHeadingAccuracy1e5Deg);
  write_u16(payload, 76, kPositionDop);
  // payload[78..83] reserved1, payload[88..91] magDec/magAcc: left zero.

  write_i32(payload,
            84,
            static_cast<std::int32_t>(std::lround(static_cast<double>(fix.course_deg) * 1e5)));  // headVeh == headMot

  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  for (std::size_t i = 2; i < frame.size() - 2; ++i)
  {
    ck_a = static_cast<std::uint8_t>(ck_a + frame[i]);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }
  frame[frame.size() - 2] = ck_a;
  frame[frame.size() - 1] = ck_b;

  return frame;
}

}  // namespace hemerion::sensors::gps::fmu
