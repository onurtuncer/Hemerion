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
/// The ADC rails are the only limit the *words* respect. The part's rated
/// operating envelope (300--1250 hPa, -40..+85 C) is narrower than what the
/// converter can express, and a real BMP390 taken outside it keeps producing
/// plausible-looking conversions with no accuracy guarantee -- so this model
/// does too, deliberately. What it adds is a side channel the silicon does
/// not have: each Conversion carries flags saying whether the ambient
/// pressure and die temperature it was produced under were inside the rated
/// envelope, so a bench can *say* the part was out of rating without the
/// register path behaving any differently. NASA check-case 12
/// (examples/f16_trim_ecos, Mach 2 at 30 013 ft) is the flight that
/// motivated them: 253 of 286 conversions below the rated pressure floor,
/// all reading plausibly.
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

/// @name The part's rated operating envelope (datasheet section 1)
///
/// Simulation-side knowledge only: no register reflects these, the real part
/// converts outside them without complaint, and the on-target driver has no
/// business knowing them -- which is why they live here and not in
/// bmp390_registers.h.
/// @{
inline constexpr float kBmp390RatedPressureMinPa = 30000.0F;   ///< 300 hPa rated floor.
inline constexpr float kBmp390RatedPressureMaxPa = 125000.0F;  ///< 1250 hPa rated ceiling.
inline constexpr float kBmp390RatedTemperatureMinC = -40.0F;   ///< Rated die-temperature floor [°C].
inline constexpr float kBmp390RatedTemperatureMaxC = 85.0F;    ///< Rated die-temperature ceiling [°C].
/// @}

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
  /// One conversion's raw words, plus the side channel the silicon does not
  /// have: whether the conditions it was produced under were inside the
  /// part's rated envelope. The flags describe the *stimulus* (noisy ambient
  /// pressure, die temperature), never the words -- out of rating, the words
  /// still read plausibly, exactly as the real converter's do.
  struct Conversion
  {
    std::uint32_t uncomp_press = 0;     ///< Raw 24-bit pressure word.
    std::uint32_t uncomp_temp = 0;      ///< Raw 24-bit temperature word.
    bool pressure_in_rating = true;     ///< Ambient pressure inside 300--1250 hPa.
    bool temperature_in_rating = true;  ///< Die temperature inside -40..+85 °C.
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

  /// @brief Produces one conversion from one truth altitude, through the ISA.
  ///
  /// Convenience for a caller with no atmosphere of its own: the altitude is
  /// mapped to ambient pressure and temperature by the standard atmosphere
  /// and handed to measure_ambient(). A caller whose plant integrates its own
  /// air -- a non-standard day, a pressure that is not ISA(h) -- should use
  /// that directly instead, or the part will read the standard atmosphere
  /// while the vehicle flies through a different one.
  ///
  /// @param altitude_m True geometric altitude above mean sea level [m].
  [[nodiscard]] Conversion measure(double altitude_m)
  {
    return measure_ambient(baro::fmu::BaroNoiseModel::isa_pressure_pa(altitude_m),
                           baro::fmu::BaroNoiseModel::isa_temperature_c(altitude_m));
  }

  /// @brief Produces one conversion from the ambient the die is actually in.
  ///
  /// Temperature is inverted first: the pressure polynomial is conditioned
  /// on `t_lin`, and the value used is the *compensated* temperature of the
  /// chosen raw word -- i.e. exactly what the driver will compute -- so the
  /// pressure inversion is conditioned the same way the forward path will be.
  ///
  /// @param ambient_pressure_pa    True static pressure at the part [Pa].
  /// @param ambient_temperature_c  True die temperature [degrees Celsius].
  [[nodiscard]] Conversion measure_ambient(double ambient_pressure_pa, double ambient_temperature_c)
  {
    const double pressure_pa = ambient_pressure_pa + pressure_bias_pa_ + draw_noise(config_.pressure_noise_pa);
    const double temperature_c = ambient_temperature_c + temperature_bias_c_ + draw_noise(config_.temperature_noise_c);

    Conversion conversion;
    conversion.uncomp_temp = invert_monotonic(
        [this](std::uint32_t word) { return compensator_.compensate_temperature(word); }, temperature_c);
    const double t_lin = compensator_.compensate_temperature(conversion.uncomp_temp);
    conversion.uncomp_press = invert_monotonic(
        [this, t_lin](std::uint32_t word) { return compensator_.compensate_pressure(word, t_lin); }, pressure_pa);
    // Judged on the noisy values -- what the die is actually subjected to --
    // not on the clean ISA truth, so a part biased toward its rated floor
    // crosses it when its conversions do.
    conversion.pressure_in_rating =
        pressure_pa >= kBmp390RatedPressureMinPa && pressure_pa <= kBmp390RatedPressureMaxPa;
    conversion.temperature_in_rating =
        temperature_c >= kBmp390RatedTemperatureMinC && temperature_c <= kBmp390RatedTemperatureMaxC;
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
