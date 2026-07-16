// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_conversion.h
/// @brief Raw radar altimeter count to SI-unit conversion.

#pragma once

#include "Hemerion/radalt/radalt_types.h"

namespace hemerion::sensors::radalt
{

/// @brief Converts raw range counts to SI units using the given sensitivity.
///
/// @param raw   Raw counts + status as read from the sensor.
/// @param scale Sensitivity of the sensor's range register.
/// @param out   Receives the converted sample (range in m, `valid` decoded
///              from the status word, timestamp copied through) on success.
///              A no-return sample converts successfully with `valid` false
///              and range 0 -- loss of track is data, not an error.
/// @return RadAltConversionError::kInvalidScale (leaving `out` unmodified)
///         if the sensitivity is not strictly positive, else kNone.
RadAltConversionError convert_raw_to_si(const RadAltRawSample& raw, const RadAltScale& scale, RadAltSample& out);

}  // namespace hemerion::sensors::radalt
