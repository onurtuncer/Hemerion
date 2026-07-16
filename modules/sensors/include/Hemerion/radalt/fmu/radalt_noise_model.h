// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_noise_model.h
/// @brief Radar altimeter error model + register quantization for the
/// hardware-simulator FMU.
///
/// Turns a noiseless truth height above ground level (as a 6-DoF plant plus
/// terrain model provides) into the raw range register counts a real radar
/// altimeter would latch: a constant per-run turn-on bias plus white
/// measurement noise, then quantization to the configured sensitivity.
/// Beyond the configured maximum tracking range (or below zero height) the
/// model reports kRadAltStatusNoReturn with a zeroed range word, exactly as
/// a real part loses ground track rather than clipping. Host-only -- this
/// lives under the fmu/ subtree built only when HEMERION_BUILD_FMU is on,
/// never cross-compiled, so <random> is fine here (unlike the on-target
/// conversion code one directory up).
///
/// The counts produced here are the exact inverse of convert_raw_to_si()
/// (modules/sensors/src/radalt/radalt_conversion.cpp): feeding them back
/// through that function with the same RadAltScale recovers the noisy range
/// to quantization error.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

#include "Hemerion/radalt/radalt_types.h"

/// @namespace hemerion::sensors::radalt::fmu
/// @brief Host-only radar altimeter hardware-simulator FMU: error model,
/// packet emitter, and FMI 2.0 glue.
namespace hemerion::sensors::radalt::fmu
{

/// One step of noiseless truth, in SI units.
struct RadAltTruthSample
{
  double height_agl_m = 0.0;       ///< True height above ground level along the beam [m].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this sample [microseconds].
};

/// Error magnitudes and register sensitivity applied by RadAltNoiseModel.
/// Defaults approximate a small FMCW radar altimeter: centimeter-class
/// resolution, decimeter-class noise, a few kilometers of tracking range.
struct RadAltNoiseConfig
{
  float range_noise_m = 0.1F;        ///< Range white noise, 1-sigma [m].
  float range_bias_sigma_m = 0.05F;  ///< Turn-on range bias 1-sigma, drawn once per run [m].
  float max_range_m = 6000.0F;       ///< Maximum tracking range; beyond it the model reports no return [m].
  RadAltScale scale{ 100.0F };       ///< Register sensitivity; the driver must convert with the same value.
};

/// @brief Applies turn-on bias + white noise to truth heights and quantizes
/// them to raw range register counts, with loss-of-track beyond max range.
class RadAltNoiseModel
{
public:
  /// @param config Error magnitudes and register sensitivity.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The turn-on bias is
  ///               drawn from this stream at construction.
  explicit RadAltNoiseModel(const RadAltNoiseConfig& config = {}, std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
    // normal_distribution requires sigma > 0 (MS STL debug builds enforce
    // it), so a zero-noise config must skip construction, not just the draw.
    if (config_.range_bias_sigma_m > 0.0F)
    {
      std::normal_distribution<float> range_bias(0.0F, config_.range_bias_sigma_m);
      range_bias_m_ = range_bias(rng_);
    }
  }

  /// @brief Produces one raw register sample from one truth sample.
  ///
  /// Within tracking range: truth + turn-on bias + white noise, scaled to
  /// counts with the configured sensitivity, floored at zero (a radar range
  /// is never negative). Beyond max range or below zero truth height: a
  /// no-return sample with a zeroed range word.
  ///
  /// @param truth Noiseless height-above-ground sample.
  /// @return Raw counts + status stamped with `truth.timestamp_us`.
  [[nodiscard]] RadAltRawSample apply(const RadAltTruthSample& truth)
  {
    RadAltRawSample raw;
    raw.timestamp_us = truth.timestamp_us;

    if (truth.height_agl_m < 0.0 || truth.height_agl_m > static_cast<double>(config_.max_range_m))
    {
      raw.range = 0;
      raw.status = kRadAltStatusNoReturn;
      return raw;
    }

    float noise = 0.0F;
    if (config_.range_noise_m > 0.0F)
    {
      std::normal_distribution<float> range_noise(0.0F, config_.range_noise_m);
      noise = range_noise(rng_);
    }
    raw.range = quantize((truth.height_agl_m + range_bias_m_ + noise) * config_.scale.range_lsb_per_m);
    raw.status = kRadAltStatusTrackValid;
    return raw;
  }

  /// The sensitivity this model quantizes with (what the consuming driver
  /// must pass to convert_raw_to_si()).
  [[nodiscard]] const RadAltScale& scale() const { return config_.scale; }

private:
  [[nodiscard]] static std::int32_t quantize(double counts)
  {
    // 24-bit range register: never negative, saturate at full scale.
    const double clamped = std::clamp(counts, 0.0, 16777215.0);
    return static_cast<std::int32_t>(std::lround(clamped));
  }

  RadAltNoiseConfig config_;
  std::mt19937_64 rng_;
  float range_bias_m_ = 0.0F;
};

}  // namespace hemerion::sensors::radalt::fmu
