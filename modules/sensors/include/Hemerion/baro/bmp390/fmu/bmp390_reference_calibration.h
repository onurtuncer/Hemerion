// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_reference_calibration.h
/// @brief The simulated BMP390's calibration NVM burn.
///
/// Every physical BMP390 leaves the factory with an individual set of
/// calibration words; this is the simulated part's set, shared by the device
/// model (which serves it from registers 0x31..0x45) and the measurement
/// model (which inverts the compensation it parameterizes). The values are
/// representative rather than copied from one physical specimen: chosen so
/// the compensated output is monotonic in each raw word and spans roughly
/// 100 hPa to 1130 hPa and -115 °C to +160 °C across the 24-bit ADC range,
/// bracketing the part's specified 300-1250 hPa / -40-+85 °C operating
/// envelope the way a real burn does.
///
/// Host-only, like the rest of the fmu/ subtree.

#pragma once

#include "Hemerion/baro/bmp390/bmp390_registers.h"

namespace hemerion::sensors::baro::bmp390::fmu
{

/// Calibration words of the simulated part (see file comment).
inline constexpr Bmp390CalibData kBmp390ReferenceCalibration{
  .par_t1 = 27000,
  .par_t2 = 18000,
  .par_t3 = -10,
  .par_p1 = 21700,
  .par_p2 = 17000,
  .par_p3 = 30,
  .par_p4 = 5,
  .par_p5 = 1250,
  .par_p6 = 400,
  .par_p7 = 20,
  .par_p8 = -100,
  .par_p9 = 15000,
  .par_p10 = 10,
  .par_p11 = 20,
};

}  // namespace hemerion::sensors::baro::bmp390::fmu
