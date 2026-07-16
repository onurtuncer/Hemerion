// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_conversion.cpp
/// @brief Implements the raw-count to microtesla magnetometer conversion
/// declared in mag_conversion.h.

#include "Hemerion/mag/mag_conversion.h"

namespace hemerion::sensors::mag
{

MagConversionError convert_raw_to_si(const MagRawSample& raw, const MagScale& scale, MagSample& out)
{
  if (scale.mag_lsb_per_ut <= 0.0F)
  {
    return MagConversionError::kInvalidScale;
  }

  const float mag_scale = 1.0F / scale.mag_lsb_per_ut;

  out.mag_x_ut = static_cast<float>(raw.mag_x) * mag_scale;
  out.mag_y_ut = static_cast<float>(raw.mag_y) * mag_scale;
  out.mag_z_ut = static_cast<float>(raw.mag_z) * mag_scale;
  out.timestamp_us = raw.timestamp_us;

  return MagConversionError::kNone;
}

}  // namespace hemerion::sensors::mag
