// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#pragma once

#include "Hemerion/imu/imu_types.h"

namespace hemerion::sensors::imu {

// Converts raw accel/gyro counts to SI units using the given sensitivity.
// Returns kInvalidScale (leaving out unmodified) if either sensitivity is
// not strictly positive.
ImuConversionError convert_raw_to_si(const ImuRawSample& raw, const ImuScale& scale, ImuSample& out);

}  // namespace hemerion::sensors::imu
