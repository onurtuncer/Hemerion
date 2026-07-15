// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_conversion.h
/// @brief Raw IMU count to SI-unit conversion.

#pragma once

#include "Hemerion/imu/imu_types.h"

namespace hemerion::sensors::imu {

/// @brief Converts raw accel/gyro counts to SI units using the given
/// sensitivity.
///
/// @param raw   Raw counts as read from the sensor.
/// @param scale Sensitivity of the sensor's configured full-scale range.
/// @param out   Receives the converted sample (accel in m/s^2, gyro in
///              rad/s, timestamp copied through) on success.
/// @return ImuConversionError::kInvalidScale (leaving `out` unmodified) if
///         either sensitivity is not strictly positive, else kNone.
ImuConversionError convert_raw_to_si(const ImuRawSample& raw, const ImuScale& scale, ImuSample& out);

}  // namespace hemerion::sensors::imu
