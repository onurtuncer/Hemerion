// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_compensation.h
/// @brief BMP390 output compensation: raw 24-bit conversion words to Pa / °C.
///
/// The floating-point compensation of datasheet BST-BMP390-DS002 section 8.4
/// (the same polynomial Bosch's reference BMP3 API ships): the raw words mean
/// nothing until corrected with the per-part calibration coefficients read
/// from NVM at probe time. Temperature comes first -- its compensated value
/// (`t_lin` in the datasheet) is an input to the pressure polynomial.
///
/// On-target code, cross-compiled into firmware: double arithmetic only, no
/// allocation, no exceptions. The Cortex-M7's double-precision FPU makes the
/// double math the datasheet specifies the straightforward choice over a
/// fixed-point reimplementation that would need its own error analysis.
///
/// The hardware-simulator FMU runs this exact code in reverse (numerically,
/// see `baro/bmp390/fmu/bmp390_measurement_model.h`), which is what makes the
/// driver-side compensation testable to quantization error instead of against
/// a reimplementation of itself.

#pragma once

#include <cstdint>

#include "Hemerion/baro/bmp390/bmp390_registers.h"

namespace hemerion::sensors::baro::bmp390
{

/// @brief Calibration coefficients scaled per datasheet table 22, plus the
/// compensation polynomial they parameterize.
class Bmp390Compensator
{
public:
  Bmp390Compensator() = default;

  /// @brief Scales raw NVM words into the floating-point coefficients
  /// (datasheet table 22: each word divided by a fixed power of two).
  [[nodiscard]] static Bmp390Compensator from_calibration(const Bmp390CalibData& calib);

  /// @brief Compensates a raw 24-bit temperature word.
  /// @return Temperature [°C] -- the datasheet's `t_lin`.
  [[nodiscard]] double compensate_temperature(std::uint32_t uncomp_temp) const;

  /// @brief Compensates a raw 24-bit pressure word.
  /// @param uncomp_press  Raw pressure conversion word.
  /// @param temperature_c Compensated temperature of the same conversion
  ///                      (`t_lin`), from compensate_temperature().
  /// @return Pressure [Pa].
  [[nodiscard]] double compensate_pressure(std::uint32_t uncomp_press, double temperature_c) const;

private:
  double par_t1_ = 0.0;
  double par_t2_ = 0.0;
  double par_t3_ = 0.0;
  double par_p1_ = 0.0;
  double par_p2_ = 0.0;
  double par_p3_ = 0.0;
  double par_p4_ = 0.0;
  double par_p5_ = 0.0;
  double par_p6_ = 0.0;
  double par_p7_ = 0.0;
  double par_p8_ = 0.0;
  double par_p9_ = 0.0;
  double par_p10_ = 0.0;
  double par_p11_ = 0.0;
};

}  // namespace hemerion::sensors::baro::bmp390
