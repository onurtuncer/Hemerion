// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_types.h
/// @brief IMU sample and sensitivity types shared by drivers and conversion.

#pragma once

#include <cstdint>

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

/// Result of convert_raw_to_si().
enum class ImuConversionError : std::uint8_t
{
  kNone,          ///< Conversion succeeded.
  kInvalidScale,  ///< A sensitivity in ImuScale was not strictly positive.
};

}  // namespace hemerion::sensors::imu
