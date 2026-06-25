// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include <numbers>

#include "Hemerion/imu/imu_conversion.h"

namespace hemerion::sensors::imu {

namespace {
// TODO: source from Aetherion's environment model instead of this local
// placeholder once modules/sensors links against Aetherion::Aetherion.
constexpr float kStandardGravityMps2 = 9.80665F;
constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0F;
}  // namespace

ImuConversionError convert_raw_to_si(const ImuRawSample& raw, const ImuScale& scale, ImuSample& out) {
  if (scale.accel_lsb_per_g <= 0.0F || scale.gyro_lsb_per_dps <= 0.0F) {
    return ImuConversionError::kInvalidScale;
  }

  const float accel_scale = kStandardGravityMps2 / scale.accel_lsb_per_g;
  const float gyro_scale = kDegToRad / scale.gyro_lsb_per_dps;

  out.accel_x = static_cast<float>(raw.accel_x) * accel_scale;
  out.accel_y = static_cast<float>(raw.accel_y) * accel_scale;
  out.accel_z = static_cast<float>(raw.accel_z) * accel_scale;
  out.gyro_x = static_cast<float>(raw.gyro_x) * gyro_scale;
  out.gyro_y = static_cast<float>(raw.gyro_y) * gyro_scale;
  out.gyro_z = static_cast<float>(raw.gyro_z) * gyro_scale;
  out.timestamp_us = raw.timestamp_us;

  return ImuConversionError::kNone;
}

}  // namespace hemerion::sensors::imu
