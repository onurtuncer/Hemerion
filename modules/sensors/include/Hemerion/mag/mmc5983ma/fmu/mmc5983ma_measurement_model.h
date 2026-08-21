// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_measurement_model.h
/// @brief Truth body-frame field to raw MMC5983MA counts: installation
/// error, bridge offset, noise, and the part's fixed-scale quantization.
///
/// The generic magnetometer FMU (mag/fmu/mag_noise_model.h) quantizes with a
/// made-up 100 LSB/uT scale into int16 registers; this model instead produces
/// the 18-bit unsigned words a real MMC5983MA would latch:
///
///   1. the truth field plus a constant per-run **hard-iron** offset -- a
///      real magnetic field the installation adds, so it is measured like any
///      other field -- plus white noise at the datasheet's RMS figure;
///   2. the sensing polarity the part's SET/RESET state currently implies,
///      which flips the *field* and nothing else;
///   3. a constant per-run **bridge offset** -- an electrical property of the
///      Wheatstone bridges, so it does *not* flip with polarity;
///   4. the fixed 163.84 LSB/uT sensitivity, centred on the part's
///      131072-count null field output and saturated to 18 bits.
///
/// Steps 2 and 3 are the whole point. Because the field flips and the offset
/// does not, a driver that runs a SET/RESET pair recovers both separately --
/// and one that does not cannot tell them apart. The bridge offset defaults
/// large enough to make that unmissable: the part's null field output is
/// specified only to +/-0.5 gauss, which is Earth's entire field, so an
/// uncalibrated driver here is wrong by roughly 100% and not by a rounding
/// error. That is the bug this simulator exists to make visible.
///
/// There is no inverse-compensation bisection as in the BMP390 model, and
/// nothing to invert: the part's transfer function is one multiply, so the
/// forward direction closes in exact arithmetic and
/// @ref hemerion::sensors::mag::convert_raw_to_si() recovers the noisy truth
/// to quantization error by construction.
///
/// Host-only: `<random>` lives here, never in the on-target code.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

#include "Hemerion/mag/mmc5983ma/mmc5983ma_registers.h"

/// @namespace hemerion::sensors::mag::mmc5983ma::fmu
/// @brief Host-only MMC5983MA hardware simulator: device model, measurement
/// model, and FMI glue.
namespace hemerion::sensors::mag::mmc5983ma::fmu
{

/// Error magnitudes applied by Mmc5983maMeasurementModel. Defaults follow the
/// datasheet's specification table for the 8 ms bandwidth.
struct Mmc5983maMeasurementConfig
{
  /// White noise, 1-sigma per axis [uT]. The datasheet's total RMS noise at
  /// the 8 ms bandwidth (0.4 mG); use mmc5983ma_rms_noise_ut() to match a
  /// different bandwidth.
  float noise_ut = mmc5983ma_rms_noise_ut(Mmc5983maBandwidth::k100Hz);

  /// Hard-iron offset, 1-sigma per axis, drawn once per run [uT]. A real
  /// field the board's own ferrous content and steady currents add, so
  /// SET/RESET does *not* remove it -- only a magnetic calibration flying
  /// the vehicle through attitudes can. A few uT is a clean control card;
  /// one with a motor near the part is far worse.
  float hard_iron_sigma_ut = 1.0F;

  /// Bridge offset, 1-sigma per axis, drawn once per run [uT]. Taking the
  /// datasheet's +/-0.5 gauss null-field-output tolerance as a 3-sigma
  /// bound gives 0.167 gauss = 16.7 uT. This is what a SET/RESET pair
  /// removes, and it is deliberately the same order as Earth's field.
  float bridge_offset_sigma_ut = 16.7F;

  /// Die temperature white noise, 1-sigma [degrees C]. Small against the
  /// part's 0.8 C output step, which dominates.
  float temperature_noise_c = 0.2F;
};

/// @brief Maps truth body-frame field to the raw counts the simulated part
/// latches.
class Mmc5983maMeasurementModel
{
public:
  /// @param config Error magnitudes.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs. The per-run hard-iron
  ///               and bridge offsets are drawn from this stream at
  ///               construction.
  explicit Mmc5983maMeasurementModel(const Mmc5983maMeasurementConfig& config = {},
                                     std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
    // normal_distribution requires sigma > 0 (MS STL debug builds enforce
    // it), so a zero-error config must skip construction, not just the draw.
    if (config_.hard_iron_sigma_ut > 0.0F)
    {
      std::normal_distribution<float> hard_iron(0.0F, config_.hard_iron_sigma_ut);
      for (float& axis : hard_iron_ut_)
      {
        axis = hard_iron(rng_);
      }
    }
    if (config_.bridge_offset_sigma_ut > 0.0F)
    {
      std::normal_distribution<float> offset(0.0F, config_.bridge_offset_sigma_ut);
      bridge_offset_.x = to_counts(offset(rng_));
      bridge_offset_.y = to_counts(offset(rng_));
      bridge_offset_.z = to_counts(offset(rng_));
    }
  }

  /// @brief Produces one field measurement's raw counts.
  ///
  /// @param truth_x_ut Truth body-frame field, X axis [uT].
  /// @param truth_y_ut Truth body-frame field, Y axis [uT].
  /// @param truth_z_ut Truth body-frame field, Z axis [uT].
  /// @param sensing    The part's magnetization and automatic-set/reset
  ///                   state, from Mmc5983maI2cSlave::sensing_state().
  [[nodiscard]] Mmc5983maFieldCounts
  measure(double truth_x_ut, double truth_y_ut, double truth_z_ut, const Mmc5983maSensingState& sensing)
  {
    // With Auto_SR_en the part differences its own SET/RESET pair: the
    // result is positive-polarity and the bridge offset is already gone,
    // whatever the standing magnetization happens to be.
    const double polarity = sensing.automatic_set_reset ? 1.0 : static_cast<double>(sensing.magnetization);

    Mmc5983maFieldCounts counts;
    counts.x = axis(truth_x_ut, hard_iron_ut_[0], polarity, sensing.automatic_set_reset ? 0 : bridge_offset_.x);
    counts.y = axis(truth_y_ut, hard_iron_ut_[1], polarity, sensing.automatic_set_reset ? 0 : bridge_offset_.y);
    counts.z = axis(truth_z_ut, hard_iron_ut_[2], polarity, sensing.automatic_set_reset ? 0 : bridge_offset_.z);
    return counts;
  }

  /// @brief Produces one temperature measurement's `Tout` count.
  /// @param truth_temperature_c Truth die temperature [degrees C].
  [[nodiscard]] std::uint8_t measure_temperature(double truth_temperature_c)
  {
    return mmc5983ma_encode_temperature(static_cast<float>(truth_temperature_c) +
                                        draw_noise(config_.temperature_noise_c));
  }

  /// The bridge offset this simulated part was born with -- the truth a
  /// driver's calibrate_offset() should recover.
  [[nodiscard]] const Mmc5983maBridgeOffset& bridge_offset() const { return bridge_offset_; }

  /// The hard-iron offset this simulated installation adds [uT]. Unlike the
  /// bridge offset, no amount of SET/RESET removes it; it is here so a test
  /// can account for it rather than mistake it for a driver bug.
  [[nodiscard]] const float* hard_iron_ut() const { return hard_iron_ut_; }

private:
  [[nodiscard]] static std::int32_t to_counts(float microtesla)
  {
    return static_cast<std::int32_t>(std::lround(static_cast<double>(microtesla) * kMmc5983maLsbPerMicrotesla));
  }

  // One axis: the real applied field (truth + hard iron + noise) enters the
  // bridges with the sensing polarity, the bridge's electrical offset does
  // not, and the sum lands on the part's null-field-centred 18-bit scale.
  [[nodiscard]] std::uint32_t axis(double truth_ut, float hard_iron, double polarity, std::int32_t offset_counts)
  {
    const double applied_ut =
        truth_ut + static_cast<double>(hard_iron) + static_cast<double>(draw_noise(config_.noise_ut));
    const double counts = static_cast<double>(kMmc5983maNullFieldOutput) +
                          (polarity * applied_ut * static_cast<double>(kMmc5983maLsbPerMicrotesla)) +
                          static_cast<double>(offset_counts);
    // 18-bit unsigned converter: saturate at the rails, as real silicon does
    // outside its +/-8 gauss span.
    const double clamped = std::clamp(counts, 0.0, static_cast<double>(kMmc5983maFullScaleCounts));
    return static_cast<std::uint32_t>(std::llround(clamped));
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

  Mmc5983maMeasurementConfig config_;
  std::mt19937_64 rng_;
  float hard_iron_ut_[3] = { 0.0F, 0.0F, 0.0F };
  Mmc5983maBridgeOffset bridge_offset_{};
};

}  // namespace hemerion::sensors::mag::mmc5983ma::fmu
