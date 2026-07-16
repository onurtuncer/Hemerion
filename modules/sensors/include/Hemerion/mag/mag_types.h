// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_types.h
/// @brief Magnetometer sample and sensitivity types shared by drivers and
/// conversion.

#pragma once

#include <cstdint>

/// @namespace hemerion::sensors::mag
/// @brief Magnetometer sample types and raw-count to SI-unit conversion.
namespace hemerion::sensors::mag
{

/// Raw magnetic field counts as read off a magnetometer's data registers, in
/// whatever LSB units the sensor's configured full-scale range implies.
struct MagRawSample
{
  std::int32_t mag_x = 0;          ///< Magnetic field X axis [LSB].
  std::int32_t mag_y = 0;          ///< Magnetic field Y axis [LSB].
  std::int32_t mag_z = 0;          ///< Magnetic field Z axis [LSB].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Magnetometer sample in microtesla (Earth's field is ~25-65 uT, so uT is
/// the working unit throughout Hemerion rather than the unwieldy tesla).
struct MagSample
{
  float mag_x_ut = 0.0F;           ///< Magnetic field X axis [uT].
  float mag_y_ut = 0.0F;           ///< Magnetic field Y axis [uT].
  float mag_z_ut = 0.0F;           ///< Magnetic field Z axis [uT].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Sensitivity of a specific sensor + full-scale-range configuration.
struct MagScale
{
  float mag_lsb_per_ut = 0.0F;  ///< Magnetometer sensitivity [LSB per uT].
};

/// Result of convert_raw_to_si().
enum class MagConversionError : std::uint8_t
{
  kNone,          ///< Conversion succeeded.
  kInvalidScale,  ///< The sensitivity in MagScale was not strictly positive.
};

}  // namespace hemerion::sensors::mag
