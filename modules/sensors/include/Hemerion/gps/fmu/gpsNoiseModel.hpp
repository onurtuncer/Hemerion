// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsNoiseModel.hpp
/// @brief GPS receiver noise model for the hardware-simulator FMU.
///
/// Turns a noiseless truth trajectory sample (as Aetherion would provide
/// one) into a GpsFix that looks like it came from a real u-blox M9N: a
/// white per-epoch error plus a **time-correlated** one on each channel,
/// and the receiver's own self-reported accuracy estimate. Host-only --
/// this lives under the fmu/ subtree built only when HEMERION_BUILD_FMU is
/// on, never cross-compiled, so `<random>` and dynamic allocation are fine
/// here (unlike the on-target driver code one directory up).
///
/// **Why two terms.** A receiver's position error is not white. Multipath,
/// the ionospheric and tropospheric residuals and the broadcast ephemeris
/// all change over seconds to minutes, so consecutive fixes share most of
/// their error -- which is exactly what defeats the averaging a filter would
/// otherwise do, and is why a fusion filter has to carry the GPS error as a
/// state rather than treat every fix as independent. The correlated term is
/// a first-order Gauss-Markov process per channel:
///
///     x[k] = phi x[k-1] + sigma sqrt(1 - phi^2) w[k],   phi = exp(-dt / tau)
///
/// with stationary standard deviation `sigma` and correlation time `tau`,
/// started from its stationary distribution so the first fix is as wrong as
/// any other. Position carries one `tau` (minutes: the error sources above),
/// velocity a shorter one (seconds: velocity comes from Doppler and
/// decorrelates far faster). The white term is what is left after that --
/// receiver noise proper.
///
/// **Defaults preserve the previous model.** Every correlated sigma defaults
/// to zero, and a zero sigma draws nothing, so a default-configured model
/// with a given seed produces the same fix sequence it did before the
/// correlated terms existed. The realistic configuration is opt-in; see the
/// examples' `--gps-errors correlated`.
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
  float v_down_mps = 0.0F;         ///< True vertical velocity, NED convention: positive downwards [m/s].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this step [microseconds].
};

/// Noise magnitudes and constant receiver-status fields applied by
/// GpsNoiseModel. The white terms' defaults approximate a u-blox M9N in
/// open sky; the correlated terms default to off (see the file comment).
struct GpsNoiseConfig
{
  // White per-epoch error, 1-sigma.
  float horizontal_pos_noise_m = 1.5F;  ///< Applied independently to north/east [m].
  float vertical_pos_noise_m = 3.0F;    ///< [m].
  float speed_noise_mps = 0.1F;         ///< [m/s].
  float course_noise_deg = 1.0F;        ///< [degrees].

  // Time-correlated error: first-order Gauss-Markov per channel, stationary
  // 1-sigma and correlation time. A zero sigma disables the term.
  float horizontal_pos_correlated_m = 0.0F;    ///< Stationary 1-sigma, north and east independently [m].
  float vertical_pos_correlated_m = 0.0F;      ///< Stationary 1-sigma [m].
  float position_correlation_time_s = 100.0F;  ///< tau for the three position channels [s].
  float speed_correlated_mps = 0.0F;           ///< Stationary 1-sigma [m/s].
  float course_correlated_deg = 0.0F;          ///< Stationary 1-sigma [degrees].
  float velocity_correlation_time_s = 10.0F;   ///< tau for speed and course [s].

  /// What the receiver *claims* over what it *does*: `hAcc`/`vAcc` are the
  /// total 1-sigma (white and correlated in quadrature) scaled by this.
  /// 1.0 is an honest receiver; real ones run optimistic, 0.5-0.8.
  float accuracy_scale = 1.0F;

  std::uint8_t num_satellites = 11;          ///< Constant satellite count to report.
  GpsFixType fix_type = GpsFixType::kFix3D;  ///< Constant fix type to report.
};

/// @brief Applies configured white and time-correlated noise to truth
/// samples, producing realistic GpsFix values.
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

  /// @brief Forgets the correlated error state, so the next fix starts a
  /// fresh stationary draw.
  ///
  /// For a co-simulation reset: the RNG stream is this instance's physical
  /// part and survives, but the error it is *currently* carrying belongs to
  /// the environment the previous run flew through, and does not.
  void reset_state()
  {
    for (Channel* channel : { &north_, &east_, &vertical_, &speed_, &course_ })
    {
      channel->primed = false;
      channel->state = 0.0F;
    }
    have_last_timestamp_ = false;
  }

  /// @brief Total 1-sigma horizontal position error per axis [m], white and
  /// correlated in quadrature -- what an honest receiver would report.
  [[nodiscard]] float total_horizontal_sigma_m() const
  {
    return std::hypot(config_.horizontal_pos_noise_m, config_.horizontal_pos_correlated_m);
  }

  /// @brief Total 1-sigma vertical position error [m].
  [[nodiscard]] float total_vertical_sigma_m() const
  {
    return std::hypot(config_.vertical_pos_noise_m, config_.vertical_pos_correlated_m);
  }

  /// @brief Produces one noisy fix from one truth sample.
  ///
  /// Horizontal noise is drawn in local metres and converted to degrees
  /// with a flat-Earth approximation; the receiver's self-reported accuracy
  /// fields carry the total 1-sigma scaled by `accuracy_scale`.
  ///
  /// @param truth Noiseless trajectory sample.
  /// @return A GpsFix carrying `truth` plus noise, stamped with
  ///         `truth.timestamp_us`.
  [[nodiscard]] GpsFix apply(const GpsTruthSample& truth)
  {
    // Elapsed time drives the correlated terms. The first sample, or one
    // that does not advance the clock, holds them where they are.
    double dt_s = 0.0;
    if (have_last_timestamp_ && truth.timestamp_us > last_timestamp_us_)
    {
      dt_s = static_cast<double>(truth.timestamp_us - last_timestamp_us_) * 1e-6;
    }
    last_timestamp_us_ = truth.timestamp_us;
    have_last_timestamp_ = true;

    // White terms. These distributions are constructed per call and drawn in
    // this order -- north and east from one object, then vertical, speed,
    // course -- exactly as the previous model did. That is not incidental: a
    // normal_distribution caches the second half of each polar-method pair,
    // so hoisting them into members or splitting the horizontal object would
    // change the sequence a given seed produces. Constructed at a tiny
    // non-zero width and never drawn when a sigma is zero, since MS STL
    // debug builds reject a zero-width distribution outright.
    std::normal_distribution<float> horizontal_noise(0.0F, std::max(config_.horizontal_pos_noise_m, kMinWidth));
    std::normal_distribution<float> vertical_noise(0.0F, std::max(config_.vertical_pos_noise_m, kMinWidth));
    std::normal_distribution<float> speed_noise(0.0F, std::max(config_.speed_noise_mps, kMinWidth));
    std::normal_distribution<float> course_noise(0.0F, std::max(config_.course_noise_deg, kMinWidth));

    const float north_error_m =
        white(config_.horizontal_pos_noise_m, horizontal_noise) +
        correlated(north_, config_.horizontal_pos_correlated_m, config_.position_correlation_time_s, dt_s);
    const float east_error_m =
        white(config_.horizontal_pos_noise_m, horizontal_noise) +
        correlated(east_, config_.horizontal_pos_correlated_m, config_.position_correlation_time_s, dt_s);
    const float vertical_error_m =
        white(config_.vertical_pos_noise_m, vertical_noise) +
        correlated(vertical_, config_.vertical_pos_correlated_m, config_.position_correlation_time_s, dt_s);
    const float speed_error_mps =
        white(config_.speed_noise_mps, speed_noise) +
        correlated(speed_, config_.speed_correlated_mps, config_.velocity_correlation_time_s, dt_s);
    const float course_error_deg =
        white(config_.course_noise_deg, course_noise) +
        correlated(course_, config_.course_correlated_deg, config_.velocity_correlation_time_s, dt_s);

    constexpr double kMetersPerDegLat = 111320.0;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double lat_rad = truth.latitude_deg * kDegToRad;
    const double meters_per_deg_lon = kMetersPerDegLat * std::cos(lat_rad);

    GpsFix fix;
    fix.latitude_deg = truth.latitude_deg + static_cast<double>(north_error_m) / kMetersPerDegLat;
    fix.longitude_deg =
        truth.longitude_deg + (meters_per_deg_lon > 1.0 ? static_cast<double>(east_error_m) / meters_per_deg_lon : 0.0);
    fix.altitude_m = truth.altitude_m + vertical_error_m;
    fix.ground_speed_mps = std::max(0.0F, truth.ground_speed_mps + speed_error_mps);
    fix.course_deg = std::fmod(truth.course_deg + course_error_deg + 360.0F, 360.0F);
    fix.horizontal_accuracy_m = config_.accuracy_scale * total_horizontal_sigma_m();
    fix.vertical_accuracy_m = config_.accuracy_scale * total_vertical_sigma_m();
    fix.num_satellites = config_.num_satellites;
    fix.fix_type = config_.fix_type;
    fix.timestamp_us = truth.timestamp_us;
    return fix;
  }

private:
  /// One channel's Gauss-Markov state.
  struct Channel
  {
    float state = 0.0F;
    bool primed = false;
  };

  /// Width a disabled term's distribution is constructed at instead of zero.
  /// It is never drawn, so the value is irrelevant; it only has to be legal.
  static constexpr float kMinWidth = 1e-6F;

  /// A white draw, or exactly nothing for a disabled term.
  float white(float sigma, std::normal_distribution<float>& distribution)
  {
    if (sigma <= 0.0F)
    {
      return 0.0F;
    }
    return distribution(rng_);
  }

  /// Advances one Gauss-Markov channel by `dt_s` and returns its new value.
  /// Off (zero sigma) draws nothing at all, which is what keeps the default
  /// configuration's RNG sequence identical to the previous model's.
  float correlated(Channel& channel, float sigma, float tau_s, double dt_s)
  {
    if (sigma <= 0.0F)
    {
      return 0.0F;
    }
    if (!channel.primed)
    {
      // Stationary start: the receiver has been tracking for a while before
      // the first fix this model sees, so its error is already "somewhere".
      channel.state = sigma * unit_(rng_);
      channel.primed = true;
      return channel.state;
    }
    if (dt_s <= 0.0)
    {
      return channel.state;
    }
    // tau <= 0 degenerates to white (phi = 0): no memory at all.
    const double phi = (tau_s > 0.0F) ? std::exp(-dt_s / static_cast<double>(tau_s)) : 0.0;
    const double driving = std::sqrt(std::max(0.0, 1.0 - phi * phi));
    channel.state = static_cast<float>(phi * channel.state + static_cast<double>(sigma) * driving * unit_(rng_));
    return channel.state;
  }

  GpsNoiseConfig config_;
  std::mt19937_64 rng_;

  // Unit normal for the correlated terms. Only ever drawn for an enabled
  // channel, so it cannot disturb the white terms' sequence when all the
  // correlated sigmas are zero.
  std::normal_distribution<float> unit_{ 0.0F, 1.0F };

  Channel north_;
  Channel east_;
  Channel vertical_;
  Channel speed_;
  Channel course_;

  std::uint64_t last_timestamp_us_ = 0;
  bool have_last_timestamp_ = false;
};

}  // namespace hemerion::sensors::gps::fmu
