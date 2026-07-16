// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsNoiseModel.hpp
/// @brief GPS receiver noise model for the hardware-simulator FMU.
///
/// Turns a noiseless truth trajectory sample (as Aetherion would provide
/// one) into a GpsFix that looks like it came from a real u-blox M9N:
/// Gaussian position/velocity noise plus the receiver's own self-reported
/// accuracy estimate. Host-only -- this lives under the fmu/ subtree built
/// only when HEMERION_BUILD_FMU is on, never cross-compiled, so <random>
/// and dynamic allocation are fine here (unlike the on-target driver code
/// one directory up).
///
/// The horizontal noise is generated in local metres (north/east) and
/// converted to degrees with a flat-Earth approximation -- adequate for a
/// sensor noise model; this is not a navigation-grade datum transform.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

#include "Hemerion/gps/gpsTypes.hpp"

/// @namespace hemerion::sensors::gps::fmu
/// @brief Host-only GPS hardware-simulator FMU: noise model, UBX emitter,
/// UDP transport, and FMI 2.0 glue.
namespace hemerion::sensors::gps::fmu
{

/// One step of noiseless truth, in the same units GpsFix reports.
struct GpsTruthSample
{
  double latitude_deg = 0.0;       ///< True geodetic latitude [degrees].
  double longitude_deg = 0.0;      ///< True geodetic longitude [degrees].
  float altitude_m = 0.0F;         ///< True altitude above mean sea level [m].
  float ground_speed_mps = 0.0F;   ///< True speed over ground [m/s].
  float course_deg = 0.0F;         ///< True course over ground [degrees].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this step [microseconds].
};

/// Noise magnitudes and constant receiver-status fields applied by
/// GpsNoiseModel. Defaults approximate a u-blox M9N in open sky.
struct GpsNoiseConfig
{
  float horizontal_pos_noise_m = 1.5F;       ///< 1-sigma, applied independently to north/east [m].
  float vertical_pos_noise_m = 3.0F;         ///< 1-sigma [m].
  float speed_noise_mps = 0.1F;              ///< 1-sigma [m/s].
  float course_noise_deg = 1.0F;             ///< 1-sigma [degrees].
  std::uint8_t num_satellites = 11;          ///< Constant satellite count to report.
  GpsFixType fix_type = GpsFixType::kFix3D;  ///< Constant fix type to report.
};

/// @brief Applies configured Gaussian noise to truth samples, producing
/// realistic GpsFix values.
class GpsNoiseModel
{
public:
  /// @param config Noise magnitudes and constant status fields.
  /// @param seed   RNG seed; defaults to a nondeterministic seed. Pass a
  ///               fixed value for reproducible runs.
  explicit GpsNoiseModel(const GpsNoiseConfig& config = {}, std::uint64_t seed = std::random_device{}())
    : config_(config), rng_(seed)
  {
  }

  /// @brief Produces one noisy fix from one truth sample.
  ///
  /// Horizontal noise is drawn in local metres and converted to degrees
  /// with a flat-Earth approximation; the receiver's self-reported accuracy
  /// fields are filled with the configured 1-sigma values.
  ///
  /// @param truth Noiseless trajectory sample.
  /// @return A GpsFix carrying `truth` plus noise, stamped with
  ///         `truth.timestamp_us`.
  [[nodiscard]] GpsFix apply(const GpsTruthSample& truth)
  {
    std::normal_distribution<float> horizontal_noise(0.0F, config_.horizontal_pos_noise_m);
    std::normal_distribution<float> vertical_noise(0.0F, config_.vertical_pos_noise_m);
    std::normal_distribution<float> speed_noise(0.0F, config_.speed_noise_mps);
    std::normal_distribution<float> course_noise(0.0F, config_.course_noise_deg);

    const float north_error_m = horizontal_noise(rng_);
    const float east_error_m = horizontal_noise(rng_);

    constexpr double kMetersPerDegLat = 111320.0;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double lat_rad = truth.latitude_deg * kDegToRad;
    const double meters_per_deg_lon = kMetersPerDegLat * std::cos(lat_rad);

    GpsFix fix;
    fix.latitude_deg = truth.latitude_deg + static_cast<double>(north_error_m) / kMetersPerDegLat;
    fix.longitude_deg =
        truth.longitude_deg + (meters_per_deg_lon > 1.0 ? static_cast<double>(east_error_m) / meters_per_deg_lon : 0.0);
    fix.altitude_m = truth.altitude_m + vertical_noise(rng_);
    fix.ground_speed_mps = std::max(0.0F, truth.ground_speed_mps + speed_noise(rng_));
    fix.course_deg = std::fmod(truth.course_deg + course_noise(rng_) + 360.0F, 360.0F);
    fix.horizontal_accuracy_m = config_.horizontal_pos_noise_m;
    fix.vertical_accuracy_m = config_.vertical_pos_noise_m;
    fix.num_satellites = config_.num_satellites;
    fix.fix_type = config_.fix_type;
    fix.timestamp_us = truth.timestamp_us;
    return fix;
  }

private:
  GpsNoiseConfig config_;
  std::mt19937_64 rng_;
};

}  // namespace hemerion::sensors::gps::fmu
