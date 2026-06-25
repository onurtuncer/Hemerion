// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#pragma once

#include <cstdint>

namespace hemerion::sensors::imu {

// Raw accelerometer/gyroscope counts as read off a sensor's data registers,
// in whatever LSB units the sensor's configured full-scale range implies.
struct ImuRawSample {
  std::int32_t accel_x = 0;
  std::int32_t accel_y = 0;
  std::int32_t accel_z = 0;
  std::int32_t gyro_x = 0;
  std::int32_t gyro_y = 0;
  std::int32_t gyro_z = 0;
  std::uint64_t timestamp_us = 0;
};

// Accelerometer in m/s^2, gyroscope in rad/s.
struct ImuSample {
  float accel_x = 0.0F;
  float accel_y = 0.0F;
  float accel_z = 0.0F;
  float gyro_x = 0.0F;
  float gyro_y = 0.0F;
  float gyro_z = 0.0F;
  std::uint64_t timestamp_us = 0;
};

// Sensitivity of a specific sensor + full-scale-range configuration.
struct ImuScale {
  float accel_lsb_per_g = 0.0F;
  float gyro_lsb_per_dps = 0.0F;
};

enum class ImuConversionError : std::uint8_t {
  kNone,
  kInvalidScale,
};

}  // namespace hemerion::sensors::imu
