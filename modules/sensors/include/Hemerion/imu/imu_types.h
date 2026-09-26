// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_types.h
/// @brief IMU sample and sensitivity types shared by drivers and conversion.

#pragma once

#include <cstdint>
#include <string_view>

/// @namespace hemerion::sensors::imu
/// @brief IMU sample types and raw-count to SI-unit conversion.
namespace hemerion::sensors::imu
{

/// Raw accelerometer/gyroscope counts as read off a sensor's data registers,
/// in whatever LSB units the sensor's configured full-scale range implies.
struct ImuRawSample
{
  std::int32_t accel_x = 0;        ///< Accelerometer X axis [LSB].
  std::int32_t accel_y = 0;        ///< Accelerometer Y axis [LSB].
  std::int32_t accel_z = 0;        ///< Accelerometer Z axis [LSB].
  std::int32_t gyro_x = 0;         ///< Gyroscope X axis [LSB].
  std::int32_t gyro_y = 0;         ///< Gyroscope Y axis [LSB].
  std::int32_t gyro_z = 0;         ///< Gyroscope Z axis [LSB].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// IMU sample in SI units: accelerometer in m/s^2, gyroscope in rad/s.
struct ImuSample
{
  float accel_x = 0.0F;            ///< Accelerometer X axis [m/s^2].
  float accel_y = 0.0F;            ///< Accelerometer Y axis [m/s^2].
  float accel_z = 0.0F;            ///< Accelerometer Z axis [m/s^2].
  float gyro_x = 0.0F;             ///< Gyroscope X axis [rad/s].
  float gyro_y = 0.0F;             ///< Gyroscope Y axis [rad/s].
  float gyro_z = 0.0F;             ///< Gyroscope Z axis [rad/s].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Sensitivity of a specific sensor + full-scale-range configuration.
struct ImuScale
{
  float accel_lsb_per_g = 0.0F;   ///< Accelerometer sensitivity [LSB per g].
  float gyro_lsb_per_dps = 0.0F;  ///< Gyroscope sensitivity [LSB per degree/s].
};

/// A named full-scale range, and the sensitivity that goes with it.
///
/// A 16-bit part trades range against resolution: the same register spans
/// +/-2000 deg/s or +/-250, and the second resolves eight times finer. The
/// widest setting is right for a launch vehicle and wrong for an aircraft --
/// on the F-16 trim flyout every body rate falls inside one count at
/// +/-2000 deg/s, and the decoded stream is a picture of the quantiser rather
/// than of the aircraft.
///
/// **Both ends must agree.** Sensitivity is not part of the wire format: the
/// part quantises with it and the driver converts with it, and nothing on the
/// bus says which was used. On real silicon the driver programs a range
/// register and then converts with the matching number; here the simulated
/// part takes its sensitivity as an FMI parameter, so the host that configures
/// the part and the flight computer that decodes it must be told the same
/// range. Naming the pairs in one table is what keeps that from being two
/// magic numbers in two processes.
enum class ImuRange : std::uint8_t
{
  k250DpsPm16G,   ///< +/-250 deg/s, +/-16 g -- an aircraft in normal flight.
  k500DpsPm16G,   ///< +/-500 deg/s, +/-16 g -- aerobatics, or a small UAV.
  k2000DpsPm40G,  ///< +/-2000 deg/s, +/-40 g -- a launch vehicle; the default.
};

/// @brief The sensitivity a given range implies, for a 16-bit signed register.
///
/// counts = value * lsb, so lsb = 32768 / full_scale. The accelerometer is
/// quoted in LSB per g and the gyroscope in LSB per degree/s, matching the
/// units convert_raw_to_si() takes.
[[nodiscard]] constexpr ImuScale imu_scale_for(ImuRange range)
{
  switch (range)
  {
    case ImuRange::k250DpsPm16G:
      return ImuScale{ 32768.0F / 16.0F, 32768.0F / 250.0F };
    case ImuRange::k500DpsPm16G:
      return ImuScale{ 32768.0F / 16.0F, 32768.0F / 500.0F };
    case ImuRange::k2000DpsPm40G:
      break;
  }
  // The historical default: an ADIS16470-class part at its widest settings.
  // Not derived from 32768 because these are the datasheet's own numbers,
  // which is what the examples and their published figures were built on.
  return ImuScale{ 800.0F, 16.4F };
}

/// @brief Parses a range name ("250", "500", "2000"); false if unknown.
[[nodiscard]] constexpr bool imu_range_from_name(std::string_view name, ImuRange& out)
{
  if (name == "250")
  {
    out = ImuRange::k250DpsPm16G;
    return true;
  }
  if (name == "500")
  {
    out = ImuRange::k500DpsPm16G;
    return true;
  }
  if (name == "2000")
  {
    out = ImuRange::k2000DpsPm40G;
    return true;
  }
  return false;
}

/// Result of convert_raw_to_si().
enum class ImuConversionError : std::uint8_t
{
  kNone,          ///< Conversion succeeded.
  kInvalidScale,  ///< A sensitivity in ImuScale was not strictly positive.
};

}  // namespace hemerion::sensors::imu
