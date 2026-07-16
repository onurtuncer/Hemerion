// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_noise_model.h
/// @brief IMU error model + register quantization for the hardware-simulator FMU.
///
/// Turns a noiseless body-frame truth sample (specific force + angular
/// rate, as a 6-DoF plant such as Aetherion's TwoStageRocket FMU provides)
/// into the raw register counts a real MEMS IMU would latch: a constant
/// per-run turn-on bias plus white measurement noise per axis, then
/// quantization to the configured full-scale sensitivity with int16
/// register saturation. Host-only -- this lives under the fmu/ subtree
/// built only when HEMERION_BUILD_FMU is on, never cross-compiled, so
/// <random> is fine here (unlike the on-target conversion code one
/// directory up).
///
/// The counts produced here are the exact inverse of convert_raw_to_si()
/// (modules/sensors/src/imu/imu_conversion.cpp): feeding them back through
/// that function with the same ImuScale recovers the noisy SI sample to
/// quantization error. The gravity constant below must therefore match
/// kStandardGravityMps2 there.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

#include "Hemerion/imu/imu_types.h"

/// @namespace hemerion::sensors::imu::fmu
/// @brief Host-only IMU hardware-simulator FMU: error model, packet
/// emitter, and FMI 2.0 glue.
namespace hemerion::sensors::imu::fmu
{

/// One step of noiseless body-frame truth, in SI units.
struct ImuTruthSample
{
  double specific_force_x_mps2 = 0.0;  ///< True specific force, body X (nose) [m/s^2].
  double specific_force_y_mps2 = 0.0;  ///< True specific force, body Y [m/s^2].
  double specific_force_z_mps2 = 0.0;  ///< True specific force, body Z [m/s^2].
  double angular_rate_x_rad_s = 0.0;   ///< True body roll rate p [rad/s].
  double angular_rate_y_rad_s = 0.0;   ///< True body pitch rate q [rad/s].
  double angular_rate_z_rad_s = 0.0;   ///< True body yaw rate r [rad/s].
  std::uint64_t timestamp_us = 0;      ///< Simulation clock at this sample [microseconds].
};

/// Error magnitudes and register sensitivity applied by ImuNoiseModel.
/// Defaults approximate a tactical-grade MEMS part (ADIS16470-class) at its
/// widest ranges: +/-40 g accelerometer, +/-2000 deg/s gyroscope, 16-bit
/// registers.
struct ImuNoiseConfig
{
  float accel_noise_mps2 = 0.05F;        ///< Accelerometer white noise, 1-sigma per axis [m/s^2].
  float gyro_noise_rad_s = 0.002F;       ///< Gyroscope white noise, 1-sigma per axis [rad/s].
  float accel_bias_sigma_mps2 = 0.02F;   ///< Turn-on bias 1-sigma per axis, drawn once per run [m/s^2].
  float gyro_bias_sigma_rad_s = 0.001F;  ///< Turn-on bias 1-sigma per axis, drawn once per run [rad/s].
  ImuScale scale{ 800.0F, 16.4F };       ///< Register sensitivity; the driver must convert with the same values.
};

/// @brief Applies turn-on bias + white noise to truth samples and quantizes
/// them to raw register counts.
class ImuNoiseModel
{
public:
  /// @param config Error magnitudes and register sensitivity.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The six turn-on
  ///               biases are drawn from this stream at construction.
  explicit ImuNoiseModel(const ImuNoiseConfig& config = {}, std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
    // std::normal_distribution requires a strictly positive stddev, so a
    // zero (disabled) sigma must skip the distribution entirely, not
    // construct it with 0.
    if (config_.accel_bias_sigma_mps2 > 0.0F)
    {
      std::normal_distribution<float> accel_bias(0.0F, config_.accel_bias_sigma_mps2);
      for (float& bias : accel_bias_mps2_)
      {
        bias = accel_bias(rng_);
      }
    }
    if (config_.gyro_bias_sigma_rad_s > 0.0F)
    {
      std::normal_distribution<float> gyro_bias(0.0F, config_.gyro_bias_sigma_rad_s);
      for (float& bias : gyro_bias_rad_s_)
      {
        bias = gyro_bias(rng_);
      }
    }
  }

  /// @brief Produces one raw register sample from one truth sample.
  ///
  /// Per axis: truth + turn-on bias + white noise, scaled to counts with
  /// the configured sensitivity and saturated to the int16 register range
  /// exactly as real silicon clips at full scale.
  ///
  /// @param truth Noiseless body-frame sample.
  /// @return Raw counts stamped with `truth.timestamp_us`.
  [[nodiscard]] ImuRawSample apply(const ImuTruthSample& truth)
  {
    // Zero (disabled) sigmas must not reach std::normal_distribution's
    // constructor -- it requires a strictly positive stddev.
    auto accel = [&](double truth_mps2, float bias) {
      float noise = 0.0F;
      if (config_.accel_noise_mps2 > 0.0F)
      {
        noise = std::normal_distribution<float>(0.0F, config_.accel_noise_mps2)(rng_);
      }
      // Inverse of convert_raw_to_si(): counts = m/s^2 * (LSB/g) / (m/s^2 per g).
      return quantize((truth_mps2 + bias + noise) * config_.scale.accel_lsb_per_g / kStandardGravityMps2);
    };
    auto gyro = [&](double truth_rad_s, float bias) {
      float noise = 0.0F;
      if (config_.gyro_noise_rad_s > 0.0F)
      {
        noise = std::normal_distribution<float>(0.0F, config_.gyro_noise_rad_s)(rng_);
      }
      // counts = rad/s * (LSB per deg/s) / (rad per deg).
      return quantize((truth_rad_s + bias + noise) * config_.scale.gyro_lsb_per_dps / kDegToRad);
    };

    ImuRawSample raw;
    raw.accel_x = accel(truth.specific_force_x_mps2, accel_bias_mps2_[0]);
    raw.accel_y = accel(truth.specific_force_y_mps2, accel_bias_mps2_[1]);
    raw.accel_z = accel(truth.specific_force_z_mps2, accel_bias_mps2_[2]);
    raw.gyro_x = gyro(truth.angular_rate_x_rad_s, gyro_bias_rad_s_[0]);
    raw.gyro_y = gyro(truth.angular_rate_y_rad_s, gyro_bias_rad_s_[1]);
    raw.gyro_z = gyro(truth.angular_rate_z_rad_s, gyro_bias_rad_s_[2]);
    raw.timestamp_us = truth.timestamp_us;
    return raw;
  }

  /// The sensitivity this model quantizes with (what the consuming driver
  /// must pass to convert_raw_to_si()).
  [[nodiscard]] const ImuScale& scale() const { return config_.scale; }

private:
  // Must match kStandardGravityMps2 in imu_conversion.cpp (itself a
  // placeholder pending Aetherion's environment model) or the round trip
  // through convert_raw_to_si() drifts by the ratio.
  static constexpr double kStandardGravityMps2 = 9.80665;
  static constexpr double kDegToRad = std::numbers::pi / 180.0;

  [[nodiscard]] static std::int32_t quantize(double counts)
  {
    // 16-bit data registers: saturate at full scale like the real part.
    const double clamped = std::clamp(counts, -32768.0, 32767.0);
    return static_cast<std::int32_t>(std::lround(clamped));
  }

  ImuNoiseConfig config_;
  std::mt19937_64 rng_;
  float accel_bias_mps2_[3] = { 0.0F, 0.0F, 0.0F };
  float gyro_bias_rad_s_[3] = { 0.0F, 0.0F, 0.0F };
};

}  // namespace hemerion::sensors::imu::fmu
