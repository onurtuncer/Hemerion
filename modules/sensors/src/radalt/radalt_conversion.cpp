// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_conversion.cpp
/// @brief Implements the raw-count to SI-unit radar altimeter conversion
/// declared in radalt_conversion.h.

#include "Hemerion/radalt/radalt_conversion.h"

namespace hemerion::sensors::radalt
{

RadAltConversionError convert_raw_to_si(const RadAltRawSample& raw, const RadAltScale& scale, RadAltSample& out)
{
  if (scale.range_lsb_per_m <= 0.0F)
  {
    return RadAltConversionError::kInvalidScale;
  }

  out.valid = (raw.status == kRadAltStatusTrackValid);
  out.range_m = out.valid ? static_cast<float>(raw.range) / scale.range_lsb_per_m : 0.0F;
  out.timestamp_us = raw.timestamp_us;

  return RadAltConversionError::kNone;
}

}  // namespace hemerion::sensors::radalt
