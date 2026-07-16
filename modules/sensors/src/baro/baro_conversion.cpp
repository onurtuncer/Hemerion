// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_conversion.cpp
/// @brief Implements the raw-count to SI-unit barometer conversion declared
/// in baro_conversion.h.

#include "Hemerion/baro/baro_conversion.h"

namespace hemerion::sensors::baro
{

BaroConversionError convert_raw_to_si(const BaroRawSample& raw, const BaroScale& scale, BaroSample& out)
{
  if (scale.pressure_lsb_per_pa <= 0.0F || scale.temperature_lsb_per_c <= 0.0F)
  {
    return BaroConversionError::kInvalidScale;
  }

  out.pressure_pa = static_cast<float>(raw.pressure) / scale.pressure_lsb_per_pa;
  out.temperature_c = static_cast<float>(raw.temperature) / scale.temperature_lsb_per_c;
  out.timestamp_us = raw.timestamp_us;

  return BaroConversionError::kNone;
}

}  // namespace hemerion::sensors::baro
