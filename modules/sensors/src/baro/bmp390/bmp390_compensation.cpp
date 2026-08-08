// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_compensation.cpp
/// @brief Implements the BMP390 floating-point compensation declared in
/// bmp390_compensation.h (datasheet BST-BMP390-DS002 section 8.4).

#include "Hemerion/baro/bmp390/bmp390_compensation.h"

namespace hemerion::sensors::baro::bmp390
{

namespace
{

// 2^n as a double, spelled the way the datasheet's scale table reads.
[[nodiscard]] constexpr double pow2(int exponent)
{
  double result = 1.0;
  for (int i = 0; i < (exponent < 0 ? -exponent : exponent); ++i)
  {
    result *= 2.0;
  }
  return (exponent < 0) ? 1.0 / result : result;
}

}  // namespace

Bmp390Compensator Bmp390Compensator::from_calibration(const Bmp390CalibData& calib)
{
  // Datasheet table 22: par_x = nvm_par_x / 2^n (a negative n is the
  // datasheet's way of writing a multiplication).
  Bmp390Compensator compensator;
  compensator.par_t1_ = static_cast<double>(calib.par_t1) / pow2(-8);
  compensator.par_t2_ = static_cast<double>(calib.par_t2) / pow2(30);
  compensator.par_t3_ = static_cast<double>(calib.par_t3) / pow2(48);
  compensator.par_p1_ = (static_cast<double>(calib.par_p1) - pow2(14)) / pow2(20);
  compensator.par_p2_ = (static_cast<double>(calib.par_p2) - pow2(14)) / pow2(29);
  compensator.par_p3_ = static_cast<double>(calib.par_p3) / pow2(32);
  compensator.par_p4_ = static_cast<double>(calib.par_p4) / pow2(37);
  compensator.par_p5_ = static_cast<double>(calib.par_p5) / pow2(-3);
  compensator.par_p6_ = static_cast<double>(calib.par_p6) / pow2(6);
  compensator.par_p7_ = static_cast<double>(calib.par_p7) / pow2(8);
  compensator.par_p8_ = static_cast<double>(calib.par_p8) / pow2(15);
  compensator.par_p9_ = static_cast<double>(calib.par_p9) / pow2(48);
  compensator.par_p10_ = static_cast<double>(calib.par_p10) / pow2(48);
  compensator.par_p11_ = static_cast<double>(calib.par_p11) / pow2(65);
  return compensator;
}

double Bmp390Compensator::compensate_temperature(std::uint32_t uncomp_temp) const
{
  const double partial_data1 = static_cast<double>(uncomp_temp) - par_t1_;
  const double partial_data2 = partial_data1 * par_t2_;
  return partial_data2 + partial_data1 * partial_data1 * par_t3_;
}

double Bmp390Compensator::compensate_pressure(std::uint32_t uncomp_press, double temperature_c) const
{
  const double t_lin = temperature_c;
  const auto uncomp = static_cast<double>(uncomp_press);

  double partial_data1 = par_p6_ * t_lin;
  double partial_data2 = par_p7_ * t_lin * t_lin;
  double partial_data3 = par_p8_ * t_lin * t_lin * t_lin;
  const double partial_out1 = par_p5_ + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = par_p2_ * t_lin;
  partial_data2 = par_p3_ * t_lin * t_lin;
  partial_data3 = par_p4_ * t_lin * t_lin * t_lin;
  const double partial_out2 = uncomp * (par_p1_ + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = uncomp * uncomp;
  partial_data2 = par_p9_ + par_p10_ * t_lin;
  partial_data3 = partial_data1 * partial_data2;
  const double partial_data4 = partial_data3 + uncomp * uncomp * uncomp * par_p11_;

  return partial_out1 + partial_out2 + partial_data4;
}

}  // namespace hemerion::sensors::baro::bmp390
