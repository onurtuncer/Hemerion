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
/// `<random>` is fine here (unlike the on-target conversion code one
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

  /// Bias random walk, the rate at which the turn-on bias wanders away from
  /// where it started [m/s^2 per sqrt(s)]. A constant bias converges once in a
  /// filter and stops mattering; a wandering one is what a bias state is
  /// actually for. Consumer MEMS sit around 1e-4; 0 disables the walk and
  /// leaves the constant turn-on bias alone.
  float accel_bias_walk_mps2_sqrt_s = 0.0F;
  /// Gyroscope bias random walk [rad/s per sqrt(s)]. A 10 deg/h/sqrt(h) part
  /// is about 8.1e-6 in these units.
  float gyro_bias_walk_rad_s_sqrt_s = 0.0F;

  /// Scale-factor error, 1-sigma per axis, drawn once per run [fraction]. A
  /// part that reads 0.3 % high is 0.003 here. Invisible at rest and
  /// proportional to the signal, so it shows up only under manoeuvre.
  float accel_scale_sigma = 0.0F;
  float gyro_scale_sigma = 0.0F;

  /// Axis misalignment, 1-sigma per angle, drawn once per run [rad]. The
  /// triad is not perfectly square to the airframe, so each axis picks up a
  /// little of the other two; 0.001 rad is about 0.06 deg. Like scale, it is
  /// a cross-axis error that only a manoeuvre reveals.
  float misalignment_sigma_rad = 0.0F;

  ImuScale scale{ 800.0F, 16.4F };  ///< Register sensitivity; the driver must convert with the same values.
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
    // Scale and misalignment are drawn after the biases so that adding them
    // does not shift the bias draws: a configuration with them disabled
    // reproduces the previous model's stream for a given seed, exactly.
    draw_scale(config_.accel_scale_sigma, accel_scale_);
    draw_scale(config_.gyro_scale_sigma, gyro_scale_);
    draw_misalignment(accel_misalignment_);
    draw_misalignment(gyro_misalignment_);
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
    // Elapsed time drives the bias walk. The first sample, or one that does
    // not advance the clock, leaves the biases where they are.
    double dt_s = 0.0;
    if (have_last_timestamp_ && truth.timestamp_us > last_timestamp_us_)
    {
      dt_s = static_cast<double>(truth.timestamp_us - last_timestamp_us_) * 1e-6;
    }
    last_timestamp_us_ = truth.timestamp_us;
    have_last_timestamp_ = true;

    walk(accel_bias_mps2_, config_.accel_bias_walk_mps2_sqrt_s, dt_s);
    walk(gyro_bias_rad_s_, config_.gyro_bias_walk_rad_s_sqrt_s, dt_s);

    // The measured triad, before bias and noise: each axis reads its own
    // truth scaled by its sensitivity error, plus a little of the other two
    // through the misalignment. Both are identity when their sigmas are zero,
    // so this reduces to the truth vector exactly.
    const double accel_truth[3] = { truth.specific_force_x_mps2,
                                    truth.specific_force_y_mps2,
                                    truth.specific_force_z_mps2 };
    const double gyro_truth[3] = { truth.angular_rate_x_rad_s, truth.angular_rate_y_rad_s, truth.angular_rate_z_rad_s };
    double accel_sensed[3];
    double gyro_sensed[3];
    sense(accel_truth, accel_scale_, accel_misalignment_, accel_sensed);
    sense(gyro_truth, gyro_scale_, gyro_misalignment_, gyro_sensed);

    // Zero (disabled) sigmas must not reach std::normal_distribution's
    // constructor -- it requires a strictly positive stddev.
    auto accel = [&](double sensed_mps2, float bias) {
      float noise = 0.0F;
      if (config_.accel_noise_mps2 > 0.0F)
      {
        noise = std::normal_distribution<float>(0.0F, config_.accel_noise_mps2)(rng_);
      }
      // Inverse of convert_raw_to_si(): counts = m/s^2 * (LSB/g) / (m/s^2 per g).
      return quantize((sensed_mps2 + bias + noise) * config_.scale.accel_lsb_per_g / kStandardGravityMps2);
    };
    auto gyro = [&](double sensed_rad_s, float bias) {
      float noise = 0.0F;
      if (config_.gyro_noise_rad_s > 0.0F)
      {
        noise = std::normal_distribution<float>(0.0F, config_.gyro_noise_rad_s)(rng_);
      }
      // counts = rad/s * (LSB per deg/s) / (rad per deg).
      return quantize((sensed_rad_s + bias + noise) * config_.scale.gyro_lsb_per_dps / kDegToRad);
    };

    ImuRawSample raw;
    raw.accel_x = accel(accel_sensed[0], accel_bias_mps2_[0]);
    raw.accel_y = accel(accel_sensed[1], accel_bias_mps2_[1]);
    raw.accel_z = accel(accel_sensed[2], accel_bias_mps2_[2]);
    raw.gyro_x = gyro(gyro_sensed[0], gyro_bias_rad_s_[0]);
    raw.gyro_y = gyro(gyro_sensed[1], gyro_bias_rad_s_[1]);
    raw.gyro_z = gyro(gyro_sensed[2], gyro_bias_rad_s_[2]);
    raw.timestamp_us = truth.timestamp_us;
    return raw;
  }

  /// @brief The accelerometer bias each axis currently carries [m/s^2].
  ///
  /// Moves with the random walk, so a test can watch it wander rather than
  /// inferring it from a long mean.
  [[nodiscard]] const float* accel_bias_mps2() const { return accel_bias_mps2_; }

  /// @brief The gyroscope bias each axis currently carries [rad/s].
  [[nodiscard]] const float* gyro_bias_rad_s() const { return gyro_bias_rad_s_; }

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
  /// Draws a per-axis scale-factor error; identity (1.0) when disabled.
  void draw_scale(float sigma, float (&scale)[3])
  {
    if (sigma <= 0.0F)
    {
      return;
    }
    std::normal_distribution<float> draw(0.0F, sigma);
    for (float& axis : scale)
    {
      axis = 1.0F + draw(rng_);
    }
  }

  /// Draws a small-angle misalignment; identity when disabled.
  ///
  /// Small angles, so the rotation is I + skew(eps) rather than a full
  /// orthonormal matrix: at 0.06 deg the difference is second order, and
  /// keeping it linear means a disabled misalignment is exactly the identity
  /// rather than the identity to within rounding.
  void draw_misalignment(float (&matrix)[3][3])
  {
    if (config_.misalignment_sigma_rad <= 0.0F)
    {
      return;
    }
    std::normal_distribution<float> draw(0.0F, config_.misalignment_sigma_rad);
    const float ex = draw(rng_);
    const float ey = draw(rng_);
    const float ez = draw(rng_);
    matrix[0][1] = -ez;
    matrix[0][2] = ey;
    matrix[1][0] = ez;
    matrix[1][2] = -ex;
    matrix[2][0] = -ey;
    matrix[2][1] = ex;
  }

  /// truth -> what the (imperfect) triad senses, before bias and noise.
  static void sense(const double (&truth)[3], const float (&scale)[3], const float (&matrix)[3][3], double (&out)[3])
  {
    for (int i = 0; i < 3; ++i)
    {
      double sum = 0.0;
      for (int j = 0; j < 3; ++j)
      {
        sum += static_cast<double>(matrix[i][j]) * truth[j];
      }
      out[i] = sum * static_cast<double>(scale[i]);
    }
  }

  /// Advances a bias triad by one random-walk step of `dt_s`.
  ///
  /// The increment is sigma * sqrt(dt), which is what makes the walk's
  /// variance grow linearly with time and independent of the sample rate --
  /// stepping twice as often must not double the drift.
  void walk(float (&bias)[3], float rate, double dt_s)
  {
    if (rate <= 0.0F || dt_s <= 0.0)
    {
      return;
    }
    std::normal_distribution<float> step(0.0F, static_cast<float>(rate * std::sqrt(dt_s)));
    for (float& axis : bias)
    {
      axis += step(rng_);
    }
  }

  float accel_bias_mps2_[3] = { 0.0F, 0.0F, 0.0F };
  float gyro_bias_rad_s_[3] = { 0.0F, 0.0F, 0.0F };
  float accel_scale_[3] = { 1.0F, 1.0F, 1.0F };
  float gyro_scale_[3] = { 1.0F, 1.0F, 1.0F };
  float accel_misalignment_[3][3] = { { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F }, { 0.0F, 0.0F, 1.0F } };
  float gyro_misalignment_[3][3] = { { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F }, { 0.0F, 0.0F, 1.0F } };
  std::uint64_t last_timestamp_us_ = 0;
  bool have_last_timestamp_ = false;
};

}  // namespace hemerion::sensors::imu::fmu
