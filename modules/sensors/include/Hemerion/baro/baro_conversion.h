// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_conversion.h
/// @brief Raw barometer count to SI-unit conversion.

#pragma once

#include "Hemerion/baro/baro_types.h"

namespace hemerion::sensors::baro
{

/// @brief Converts raw compensated pressure/temperature counts to SI units
/// using the given sensitivity.
///
/// @param raw   Raw counts as read from the sensor.
/// @param scale Sensitivity of the sensor's compensated output words.
/// @param out   Receives the converted sample (pressure in Pa, temperature
///              in degrees Celsius, timestamp copied through) on success.
/// @return BaroConversionError::kInvalidScale (leaving `out` unmodified) if
///         either sensitivity is not strictly positive, else kNone.
BaroConversionError convert_raw_to_si(const BaroRawSample& raw, const BaroScale& scale, BaroSample& out);

}  // namespace hemerion::sensors::baro
