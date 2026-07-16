// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_imu_conversion.cpp
//
// Native unit test for IMU raw-to-SI conversion. Run by CTest under the
// test-native preset. Unity (referenced in modules/README.md as the project's
// test framework) is not yet vendored, so this uses plain asserts and an exit
// code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "Hemerion/imu/imu_conversion.h"

using hemerion::sensors::imu::convert_raw_to_si;
using hemerion::sensors::imu::ImuConversionError;
using hemerion::sensors::imu::ImuRawSample;
using hemerion::sensors::imu::ImuSample;
using hemerion::sensors::imu::ImuScale;

namespace
{

bool near(float a, float b, float tol = 1e-4F) { return std::fabs(a - b) <= tol; }

void test_one_g_on_z_converts_to_standard_gravity()
{
  // ICM-42688-P-style +-16g range: 2048 LSB/g.
  const ImuScale scale{ .accel_lsb_per_g = 2048.0F, .gyro_lsb_per_dps = 16.4F };
  ImuRawSample raw;
  raw.accel_z = 2048;
  raw.timestamp_us = 42;

  ImuSample out;
  const ImuConversionError error = convert_raw_to_si(raw, scale, out);

  assert(error == ImuConversionError::kNone);
  assert(near(out.accel_z, 9.80665F));
  assert(near(out.accel_x, 0.0F));
  assert(out.timestamp_us == 42);
}

void test_gyro_counts_convert_to_radians_per_second()
{
  // +-2000 dps range: 16.4 LSB/dps. 16.4 counts == 1 dps == pi/180 rad/s.
  const ImuScale scale{ .accel_lsb_per_g = 2048.0F, .gyro_lsb_per_dps = 16.4F };
  ImuRawSample raw;
  raw.gyro_y = 16;  // ~0.9756 dps

  ImuSample out;
  const ImuConversionError error = convert_raw_to_si(raw, scale, out);

  assert(error == ImuConversionError::kNone);
  const float expected_dps = 16.0F / 16.4F;
  const float expected_rad_s = expected_dps * (std::numbers::pi_v<float> / 180.0F);
  assert(near(out.gyro_y, expected_rad_s));
}

void test_zero_accel_scale_is_rejected()
{
  const ImuScale scale{ .accel_lsb_per_g = 0.0F, .gyro_lsb_per_dps = 16.4F };
  const ImuRawSample raw;
  ImuSample out;

  assert(convert_raw_to_si(raw, scale, out) == ImuConversionError::kInvalidScale);
}

void test_negative_gyro_scale_is_rejected()
{
  const ImuScale scale{ .accel_lsb_per_g = 2048.0F, .gyro_lsb_per_dps = -1.0F };
  const ImuRawSample raw;
  ImuSample out;

  assert(convert_raw_to_si(raw, scale, out) == ImuConversionError::kInvalidScale);
}

}  // namespace

int main()
{
  test_one_g_on_z_converts_to_standard_gravity();
  test_gyro_counts_convert_to_radians_per_second();
  test_zero_accel_scale_is_rejected();
  test_negative_gyro_scale_is_rejected();

  std::puts("test_imu_conversion: all checks passed");
  return 0;
}
