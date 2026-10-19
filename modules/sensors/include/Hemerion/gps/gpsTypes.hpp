// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsTypes.hpp
/// @brief Wire-format-independent GPS fix and parse-error types.

#pragma once

#include <cstdint>

/// @namespace hemerion::sensors::gps
/// @brief GPS wire-protocol parsers (NMEA 0183, u-blox UBX) and the driver
/// that feeds them.
namespace hemerion::sensors::gps
{

/// Kind of position solution a receiver reports.
enum class GpsFixType : std::uint8_t
{
  kNoFix,  ///< No usable position solution.
  kFix2D,  ///< Horizontal position only.
  kFix3D,  ///< Full 3D position solution.
};

/// @brief Decoded GPS fix, independent of wire format (NMEA 0183 or u-blox
/// UBX).
///
/// `timestamp_us` is stamped by the caller from a local clock when the last
/// byte of the source message was received -- it is not derived from the
/// receiver's own UTC time / iTOW fields, which don't share a clock with the
/// rest of the system.
///
/// The NED velocity is a measurement in its own right, not a restatement of
/// speed and course: it has a vertical component they lack, and it is not
/// singular at low speed where course is. Only UBX-NAV-PVT carries it, so it
/// is guarded by `ned_velocity_valid`; an NMEA fix leaves the flag false.
struct GpsFix
{
  double latitude_deg = 0.0;   ///< Geodetic latitude [degrees], north positive.
  double longitude_deg = 0.0;  ///< Geodetic longitude [degrees], east positive.
  /// Height [m]. **Datum depends on the wire format**: from UBX-NAV-PVT it is
  /// `height`, above the WGS-84 ellipsoid (not `hMSL`); from NMEA GGA it is the
  /// sentence's altitude field, above mean sea level, because the parser does
  /// not add the geoid separation back. The two differ by the geoid
  /// undulation, tens of metres. Every flight computer in this repository
  /// speaks UBX, so their logs are ellipsoidal.
  float altitude_m = 0.0F;
  float ground_speed_mps = 0.0F;             ///< Speed over ground [m/s].
  float course_deg = 0.0F;                   ///< Course over ground [degrees], 0..360.
  float horizontal_accuracy_m = 0.0F;        ///< Receiver-reported horizontal accuracy estimate [m].
  float vertical_accuracy_m = 0.0F;          ///< Receiver-reported vertical accuracy estimate [m].
  float vel_north_mps = 0.0F;                ///< NED velocity, north [m/s]; valid only if ned_velocity_valid.
  float vel_east_mps = 0.0F;                 ///< NED velocity, east [m/s]; valid only if ned_velocity_valid.
  float vel_down_mps = 0.0F;                 ///< NED velocity, down, positive downwards [m/s]; ditto.
  float speed_accuracy_mps = 0.0F;           ///< Receiver-reported velocity accuracy, 1-sigma per axis [m/s] (`sAcc`).
  bool ned_velocity_valid = false;           ///< The four fields above were decoded from the message.
  std::uint8_t num_satellites = 0;           ///< Satellites used in the solution.
  GpsFixType fix_type = GpsFixType::kNoFix;  ///< Kind of solution this fix represents.
  std::uint64_t timestamp_us = 0;            ///< Caller's local clock at message completion [microseconds].
};

/// Result of feeding one byte to a GPS wire-protocol parser.
enum class GpsParseError : std::uint8_t
{
  kNone,                ///< A supported message completed and `out` was updated.
  kIncomplete,          ///< Message still being assembled; feed more bytes.
  kChecksumMismatch,    ///< A completed message failed checksum verification.
  kUnsupportedMessage,  ///< A completed message is of a type this parser does not decode.
  kMalformed,           ///< A completed message violated the wire format.
};

}  // namespace hemerion::sensors::gps
