// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_noise_model.h
/// @brief Barometer atmosphere + error model + register quantization for the
/// hardware-simulator FMU.
///
/// Turns a noiseless truth altitude (geometric altitude above mean sea
/// level, as a 6-DoF plant such as Aetherion's TwoStageRocket FMU provides)
/// into the compensated pressure/temperature counts a real barometric part
/// would output: the ICAO Standard Atmosphere maps altitude to ambient
/// pressure and temperature, then a constant per-run turn-on bias plus white
/// measurement noise per channel, then quantization to the configured
/// sensitivity with 24-bit conversion-word saturation (MS5611-class parts
/// output 24-bit compensated words). Host-only -- this lives under the fmu/
/// subtree built only when HEMERION_BUILD_FMU is on, never cross-compiled,
/// so <random> is fine here (unlike the on-target conversion code one
/// directory up).
///
/// The counts produced here are the exact inverse of convert_raw_to_si()
/// (modules/sensors/src/baro/baro_conversion.cpp): feeding them back through
/// that function with the same BaroScale recovers the noisy sample to
/// quantization error.
///
/// TODO: source the atmosphere from Aetherion's environment model instead of
/// the local ISA layers below once one exists -- the same placeholder policy
/// as kStandardGravityMps2 in imu_conversion.cpp.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

#include "Hemerion/baro/baro_types.h"

/// @namespace hemerion::sensors::baro::fmu
/// @brief Host-only barometer hardware-simulator FMU: atmosphere + error
/// model, packet emitter, and FMI 2.0 glue.
namespace hemerion::sensors::baro::fmu
{

/// One step of noiseless truth, in SI units.
struct BaroTruthSample
{
  double altitude_m = 0.0;         ///< True geometric altitude above mean sea level [m].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this sample [microseconds].
};

/// Error magnitudes and register sensitivity applied by BaroNoiseModel.
/// Defaults approximate an MS5611-class part: ~1 Pa output resolution
/// (0.01 mbar LSB), a few Pa of conversion noise, and an absolute offset
/// within the +/-1.5 mbar datasheet band.
struct BaroNoiseConfig
{
  float pressure_noise_pa = 3.0F;        ///< Pressure white noise, 1-sigma [Pa].
  float temperature_noise_c = 0.05F;     ///< Temperature white noise, 1-sigma [degrees Celsius].
  float pressure_bias_sigma_pa = 50.0F;  ///< Turn-on pressure bias 1-sigma, drawn once per run [Pa].
  float temperature_bias_sigma_c = 0.5F; ///< Turn-on temperature bias 1-sigma, drawn once per run [degrees Celsius].
  BaroScale scale{ 1.0F, 100.0F };       ///< Output sensitivity; the driver must convert with the same values.
};

/// @brief Maps truth altitude through the ICAO Standard Atmosphere, applies
/// turn-on bias + white noise, and quantizes to raw conversion-word counts.
class BaroNoiseModel
{
public:
  /// @param config Error magnitudes and output sensitivity.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The two turn-on
  ///               biases are drawn from this stream at construction.
  explicit BaroNoiseModel(const BaroNoiseConfig& config = {}, std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
    // normal_distribution requires sigma > 0 (MS STL debug builds enforce
    // it), so a zero-noise config must skip construction, not just the draw.
    if (config_.pressure_bias_sigma_pa > 0.0F)
    {
      std::normal_distribution<float> pressure_bias(0.0F, config_.pressure_bias_sigma_pa);
      pressure_bias_pa_ = pressure_bias(rng_);
    }
    if (config_.temperature_bias_sigma_c > 0.0F)
    {
      std::normal_distribution<float> temperature_bias(0.0F, config_.temperature_bias_sigma_c);
      temperature_bias_c_ = temperature_bias(rng_);
    }
  }

  /// @brief Ambient pressure of the ICAO Standard Atmosphere at `altitude_m`
  /// [Pa].
  ///
  /// Two layers: the 6.5 K/km gradient troposphere up to 11 km, then the
  /// isothermal 216.65 K stratosphere extended upward indefinitely -- the
  /// 20 km+ gradient layers are ignored, which understates pressure slightly
  /// above 20 km (where a rocket's baro reading is aerodynamically marginal
  /// anyway). Below sea level the troposphere expression extrapolates.
  [[nodiscard]] static double isa_pressure_pa(double altitude_m)
  {
    if (altitude_m <= kTropopauseAltitudeM)
    {
      return kSeaLevelPressurePa *
             std::pow(1.0 - kTemperatureLapseKPerM * altitude_m / kSeaLevelTemperatureK, kGravityOverLapseR);
    }
    return kTropopausePressurePa *
           std::exp(-kStandardGravityMps2 * (altitude_m - kTropopauseAltitudeM) / (kAirGasConstant * kTropopauseTemperatureK));
  }

  /// @brief Ambient temperature of the ICAO Standard Atmosphere at
  /// `altitude_m` [degrees Celsius]; same two layers as isa_pressure_pa().
  /// Reported as the die temperature -- a small sensor tracks ambient.
  [[nodiscard]] static double isa_temperature_c(double altitude_m)
  {
    const double kelvin = (altitude_m <= kTropopauseAltitudeM)
                              ? kSeaLevelTemperatureK - kTemperatureLapseKPerM * altitude_m
                              : kTropopauseTemperatureK;
    return kelvin - 273.15;
  }

  /// @brief Produces one raw conversion-word sample from one truth sample.
  ///
  /// Altitude through the ISA to ambient pressure/temperature, plus turn-on
  /// bias and white noise per channel, scaled to counts with the configured
  /// sensitivity and saturated to the signed 24-bit conversion-word range.
  ///
  /// @param truth Noiseless altitude sample.
  /// @return Raw counts stamped with `truth.timestamp_us`.
  [[nodiscard]] BaroRawSample apply(const BaroTruthSample& truth)
  {
    const float p_noise = draw_noise(config_.pressure_noise_pa);
    const float t_noise = draw_noise(config_.temperature_noise_c);

    const double pressure_pa = isa_pressure_pa(truth.altitude_m) + pressure_bias_pa_ + p_noise;
    const double temperature_c = isa_temperature_c(truth.altitude_m) + temperature_bias_c_ + t_noise;

    BaroRawSample raw;
    raw.pressure = quantize(pressure_pa * config_.scale.pressure_lsb_per_pa);
    raw.temperature = quantize(temperature_c * config_.scale.temperature_lsb_per_c);
    raw.timestamp_us = truth.timestamp_us;
    return raw;
  }

  /// The sensitivity this model quantizes with (what the consuming driver
  /// must pass to convert_raw_to_si()).
  [[nodiscard]] const BaroScale& scale() const { return config_.scale; }

private:
  // ICAO Standard Atmosphere constants (lower two layers).
  static constexpr double kSeaLevelPressurePa = 101325.0;
  static constexpr double kSeaLevelTemperatureK = 288.15;
  static constexpr double kTemperatureLapseKPerM = 0.0065;
  static constexpr double kTropopauseAltitudeM = 11000.0;
  static constexpr double kTropopauseTemperatureK = 216.65;
  // Must match kStandardGravityMps2 in imu_conversion.cpp (itself a
  // placeholder pending Aetherion's environment model).
  static constexpr double kStandardGravityMps2 = 9.80665;
  static constexpr double kAirGasConstant = 287.05287;  // dry air [J/(kg K)]
  static constexpr double kGravityOverLapseR = kStandardGravityMps2 / (kTemperatureLapseKPerM * kAirGasConstant);
  // ISA pressure at the tropopause, from the troposphere expression at 11 km.
  static constexpr double kTropopausePressurePa = 22632.06;

  [[nodiscard]] static std::int32_t quantize(double counts)
  {
    // 24-bit compensated conversion words: saturate like the real part.
    const double clamped = std::clamp(counts, -8388608.0, 8388607.0);
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

  BaroNoiseConfig config_;
  std::mt19937_64 rng_;
  float pressure_bias_pa_ = 0.0F;
  float temperature_bias_c_ = 0.0F;
};

}  // namespace hemerion::sensors::baro::fmu
