// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#pragma once

#include <cstdint>

namespace hemerion::sensors::gps {

enum class GpsFixType : std::uint8_t {
  kNoFix,
  kFix2D,
  kFix3D,
};

// Decoded GPS fix, independent of wire format (NMEA 0183 or u-blox UBX).
// `timestamp_us` is stamped by the caller from a local clock when the last
// byte of the source message was received -- it is not derived from the
// receiver's own UTC time / iTOW fields, which don't share a clock with the
// rest of the system.
struct GpsFix {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  float altitude_m = 0.0F;
  float ground_speed_mps = 0.0F;
  float course_deg = 0.0F;
  float horizontal_accuracy_m = 0.0F;
  float vertical_accuracy_m = 0.0F;
  std::uint8_t num_satellites = 0;
  GpsFixType fix_type = GpsFixType::kNoFix;
  std::uint64_t timestamp_us = 0;
};

enum class GpsParseError : std::uint8_t {
  kNone,
  kIncomplete,
  kChecksumMismatch,
  kUnsupportedMessage,
  kMalformed,
};

}  // namespace hemerion::sensors::gps
