// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_conversion.h
/// @brief Raw magnetometer count to microtesla conversion.

#pragma once

#include "Hemerion/mag/mag_types.h"

namespace hemerion::sensors::mag
{

/// @brief Converts raw magnetic field counts to microtesla using the given
/// sensitivity.
///
/// @param raw   Raw counts as read from the sensor.
/// @param scale Sensitivity of the sensor's configured full-scale range.
/// @param out   Receives the converted sample (field in uT, timestamp copied
///              through) on success.
/// @return MagConversionError::kInvalidScale (leaving `out` unmodified) if
///         the sensitivity is not strictly positive, else kNone.
MagConversionError convert_raw_to_si(const MagRawSample& raw, const MagScale& scale, MagSample& out);

}  // namespace hemerion::sensors::mag
