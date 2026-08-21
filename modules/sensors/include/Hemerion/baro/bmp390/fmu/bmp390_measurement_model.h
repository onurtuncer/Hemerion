// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_measurement_model.h
/// @brief Truth altitude to raw BMP390 conversion words: atmosphere + error
/// model + inverse compensation.
///
/// The generic barometer FMU (baro/fmu/baro_noise_model.h) quantizes with a
/// made-up linear scale; this model instead produces the 24-bit words a
/// BMP390 with @ref hemerion::sensors::baro::bmp390::fmu::kBmp390ReferenceCalibration "kBmp390ReferenceCalibration"
/// would have converted:
///
///   1. truth altitude through the ICAO Standard Atmosphere (reused from
///      BaroNoiseModel's layers) to ambient pressure and temperature;
///   2. a constant per-run turn-on bias plus white noise per channel -- the
///      part's absolute-accuracy and noise floors;
///   3. numeric inversion of the *real* Bosch compensation polynomial
///      (bmp390_compensation.cpp, the exact code the on-target driver runs)
///      to the raw words whose compensated value is that noisy truth.
///
/// Step 3 is a bisection rather than an algebraic inverse: the polynomial is
/// monotonic in each raw word over the part's range (which the reference
/// calibration guarantees), and 24 halvings cost nothing at sensor rates.
/// Inverting the real forward code -- instead of maintaining a hand-derived
/// inverse -- is what keeps simulator and driver in exact agreement: the
/// driver's compensation recovers the noisy truth to quantization error, by
/// construction. Words beyond the ADC range saturate, as the real converter
/// does outside its operating envelope.
///
/// Host-only: `<random>` lives here, never in the on-target code.

#pragma once

#include <cstdint>
#include <random>

#include "Hemerion/baro/bmp390/bmp390_compensation.h"
#include "Hemerion/baro/bmp390/bmp390_registers.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_reference_calibration.h"
#include "Hemerion/baro/fmu/baro_noise_model.h"

namespace hemerion::sensors::baro::bmp390::fmu
{

/// Error magnitudes and calibration burn applied by Bmp390MeasurementModel.
/// Defaults follow datasheet table 2: ~0.03 hPa RMS noise at mid
/// oversampling, +/-0.5 hPa absolute accuracy band, ~0.005 °C temperature
/// resolution with a modest absolute offset.
struct Bmp390MeasurementConfig
{
  float pressure_noise_pa = 3.0F;         ///< Pressure white noise, 1-sigma [Pa].
  float temperature_noise_c = 0.005F;     ///< Temperature white noise, 1-sigma [°C].
  float pressure_bias_sigma_pa = 30.0F;   ///< Turn-on pressure bias 1-sigma, drawn once per run [Pa].
  float temperature_bias_sigma_c = 0.3F;  ///< Turn-on temperature bias 1-sigma, drawn once per run [°C].
  Bmp390CalibData calibration = kBmp390ReferenceCalibration;  ///< The part's NVM burn.
};

/// @brief Maps truth altitude to the raw conversion words the simulated part
/// latches.
class Bmp390MeasurementModel
{
public:
  /// One conversion's raw words.
  struct Conversion
  {
    std::uint32_t uncomp_press = 0;  ///< Raw 24-bit pressure word.
    std::uint32_t uncomp_temp = 0;   ///< Raw 24-bit temperature word.
  };

  /// @param config Error magnitudes and calibration burn.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The two turn-on
  ///               biases are drawn from this stream at construction.
  explicit Bmp390MeasurementModel(const Bmp390MeasurementConfig& config = {},
                                  std::uint64_t seed = std::random_device{}())
    : config_(config), compensator_(Bmp390Compensator::from_calibration(config.calibration)), rng_(seed)
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

  /// @brief Produces one conversion from one truth altitude.
  ///
  /// Temperature is inverted first: the pressure polynomial is conditioned
  /// on `t_lin`, and the value used is the *compensated* temperature of the
  /// chosen raw word -- i.e. exactly what the driver will compute -- so the
  /// pressure inversion is conditioned the same way the forward path will be.
  ///
  /// @param altitude_m True geometric altitude above mean sea level [m].
  [[nodiscard]] Conversion measure(double altitude_m)
  {
    const double pressure_pa = baro::fmu::BaroNoiseModel::isa_pressure_pa(altitude_m) + pressure_bias_pa_ +
                               draw_noise(config_.pressure_noise_pa);
    const double temperature_c = baro::fmu::BaroNoiseModel::isa_temperature_c(altitude_m) + temperature_bias_c_ +
                                 draw_noise(config_.temperature_noise_c);

    Conversion conversion;
    conversion.uncomp_temp = invert_monotonic(
        [this](std::uint32_t word) { return compensator_.compensate_temperature(word); }, temperature_c);
    const double t_lin = compensator_.compensate_temperature(conversion.uncomp_temp);
    conversion.uncomp_press = invert_monotonic(
        [this, t_lin](std::uint32_t word) { return compensator_.compensate_pressure(word, t_lin); }, pressure_pa);
    return conversion;
  }

  /// The compensation this model inverts (what the consuming driver runs
  /// forward).
  [[nodiscard]] const Bmp390Compensator& compensator() const { return compensator_; }

  /// The calibration burn, for handing to the device model's NVM.
  [[nodiscard]] const Bmp390CalibData& calibration() const { return config_.calibration; }

private:
  static constexpr std::uint32_t kAdcMax = 0x00FFFFFF;  // 24-bit conversion words

  // Largest word whose image is <= target under a monotonically increasing
  // f, saturating at the ADC rails -- the closest a real converter gets to a
  // value outside its span.
  template <class F>
  [[nodiscard]] static std::uint32_t invert_monotonic(F&& f, double target)
  {
    if (f(0) >= target)
    {
      return 0;
    }
    if (f(kAdcMax) <= target)
    {
      return kAdcMax;
    }
    std::uint32_t low = 0;
    std::uint32_t high = kAdcMax;
    while (high - low > 1)
    {
      const std::uint32_t mid = low + (high - low) / 2;
      if (f(mid) <= target)
      {
        low = mid;
      }
      else
      {
        high = mid;
      }
    }
    return low;
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

  Bmp390MeasurementConfig config_;
  Bmp390Compensator compensator_;
  std::mt19937_64 rng_;
  float pressure_bias_pa_ = 0.0F;
  float temperature_bias_c_ = 0.0F;
};

}  // namespace hemerion::sensors::baro::bmp390::fmu
