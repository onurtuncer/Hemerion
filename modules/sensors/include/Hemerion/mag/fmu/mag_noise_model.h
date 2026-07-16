// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_noise_model.h
/// @brief Magnetometer error model + register quantization for the
/// hardware-simulator FMU.
///
/// Turns a noiseless body-frame truth magnetic field (as a 6-DoF plant's
/// attitude applied to a geomagnetic reference field provides) into the raw
/// register counts a real magnetometer would latch: a constant per-run
/// hard-iron-like turn-on bias plus white measurement noise per axis, then
/// quantization to the configured sensitivity with int16 register
/// saturation. Soft-iron distortion is deliberately out of scope for now --
/// a diagonal-only error model keeps the inverse trivially equal to
/// convert_raw_to_si(). Host-only -- this lives under the fmu/ subtree built
/// only when HEMERION_BUILD_FMU is on, never cross-compiled, so <random> is
/// fine here (unlike the on-target conversion code one directory up).
///
/// The counts produced here are the exact inverse of convert_raw_to_si()
/// (modules/sensors/src/mag/mag_conversion.cpp): feeding them back through
/// that function with the same MagScale recovers the noisy field to
/// quantization error.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

#include "Hemerion/mag/mag_types.h"

/// @namespace hemerion::sensors::mag::fmu
/// @brief Host-only magnetometer hardware-simulator FMU: error model, packet
/// emitter, and FMI 2.0 glue.
namespace hemerion::sensors::mag::fmu
{

/// One step of noiseless body-frame truth, in microtesla.
struct MagTruthSample
{
  double mag_x_ut = 0.0;           ///< True magnetic field, body X [uT].
  double mag_y_ut = 0.0;           ///< True magnetic field, body Y [uT].
  double mag_z_ut = 0.0;           ///< True magnetic field, body Z [uT].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this sample [microseconds].
};

/// Error magnitudes and register sensitivity applied by MagNoiseModel.
/// Defaults approximate a compass-grade three-axis part (RM3100/IIS2MDC
/// class): sub-uT noise, a hard-iron-like offset of a few uT, 16-bit
/// registers at 100 LSB/uT (+/-327 uT full scale, comfortably above Earth's
/// ~25-65 uT field plus installation offsets).
struct MagNoiseConfig
{
  float mag_noise_ut = 0.2F;       ///< Magnetometer white noise, 1-sigma per axis [uT].
  float mag_bias_sigma_ut = 1.0F;  ///< Turn-on (hard-iron-like) bias 1-sigma per axis, drawn once per run [uT].
  MagScale scale{ 100.0F };        ///< Register sensitivity; the driver must convert with the same value.
};

/// @brief Applies turn-on bias + white noise to truth fields and quantizes
/// them to raw register counts.
class MagNoiseModel
{
public:
  /// @param config Error magnitudes and register sensitivity.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The three turn-on
  ///               biases are drawn from this stream at construction.
  explicit MagNoiseModel(const MagNoiseConfig& config = {}, std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
    // normal_distribution requires sigma > 0 (MS STL debug builds enforce
    // it), so a zero-noise config must skip construction, not just the draw.
    if (config_.mag_bias_sigma_ut > 0.0F)
    {
      std::normal_distribution<float> mag_bias(0.0F, config_.mag_bias_sigma_ut);
      for (float& bias : mag_bias_ut_)
      {
        bias = mag_bias(rng_);
      }
    }
  }

  /// @brief Produces one raw register sample from one truth sample.
  ///
  /// Per axis: truth + turn-on bias + white noise, scaled to counts with
  /// the configured sensitivity and saturated to the int16 register range
  /// exactly as real silicon clips at full scale.
  ///
  /// @param truth Noiseless body-frame field sample.
  /// @return Raw counts stamped with `truth.timestamp_us`.
  [[nodiscard]] MagRawSample apply(const MagTruthSample& truth)
  {
    auto axis = [&](double truth_ut, float bias) {
      // Inverse of convert_raw_to_si(): counts = uT * (LSB per uT).
      return quantize((truth_ut + bias + draw_noise(config_.mag_noise_ut)) * config_.scale.mag_lsb_per_ut);
    };

    MagRawSample raw;
    raw.mag_x = axis(truth.mag_x_ut, mag_bias_ut_[0]);
    raw.mag_y = axis(truth.mag_y_ut, mag_bias_ut_[1]);
    raw.mag_z = axis(truth.mag_z_ut, mag_bias_ut_[2]);
    raw.timestamp_us = truth.timestamp_us;
    return raw;
  }

  /// The sensitivity this model quantizes with (what the consuming driver
  /// must pass to convert_raw_to_si()).
  [[nodiscard]] const MagScale& scale() const { return config_.scale; }

private:
  [[nodiscard]] static std::int32_t quantize(double counts)
  {
    // 16-bit data registers: saturate at full scale like the real part.
    const double clamped = std::clamp(counts, -32768.0, 32767.0);
    return static_cast<std::int32_t>(std::lround(clamped));
  }

  // normal_distribution requires sigma > 0, so a zero-noise config must
  // skip construction, not just the draw.
  [[nodiscard]] float draw_noise(float sigma)
  {
    if (sigma <= 0.0F)
    {
      return 0.0F;
    }
    std::normal_distribution<float> noise(0.0F, sigma);
    return noise(rng_);
  }

  MagNoiseConfig config_;
  std::mt19937_64 rng_;
  float mag_bias_ut_[3] = { 0.0F, 0.0F, 0.0F };
};

}  // namespace hemerion::sensors::mag::fmu
