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
///
/// **Velocity: two error models** (GpsVelocityErrorModel, selected by
/// `velocity_error_model`; decided 2026-11-19).
///
/// - **Doppler (the default).** A receiver measures velocity from the
///   carrier's Doppler shift, per axis, and its error does not grow with
///   speed: independent white errors on north, east and down (0.05, 0.05 and
///   0.10 m/s by default), drawn from a **third RNG stream**. Speed over
///   ground and course are then derived from the noisy north and east
///   components, as the receiver derives them -- except at low speed, where
///   the derived course would be the direction of the noise: below
///   kDopplerCourseSpeedFactor horizontal sigmas the course is the
///   speed/course model's (truth plus course error), as it always was.
/// - **Speed/course.** The previous model, kept for runs that must
///   reproduce (Paper 1's rocket case): white and correlated errors on speed
///   and course, the horizontal velocity their resolution into north and
///   east, so the four fields agree as in a real NAV-PVT; vertical velocity
///   with its own white and correlated terms from a **second RNG stream**.
///   Its error is not isotropic and grows with speed (1 deg of course is
///   3 m/s across track at 172 m/s).
///
/// The main stream is drawn in exactly the same order whichever model is
/// selected -- speed and course errors included, which the Doppler model
/// then discards or uses only for its low-speed course -- so the selection
/// moves no position draw, and in speed/course mode a seeded run reproduces
/// every number the model produced before the Doppler model existed. The
/// reported `sAcc` is the per-axis 1-sigma that an isotropic covariance
/// would need to carry the same total error as the velocity actually has --
/// see velocity_sigma_mps().

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
  float altitude_m = 0.0F;         ///< True geodetic height [m]; see GpsFix::altitude_m.
  float ground_speed_mps = 0.0F;   ///< True speed over ground [m/s].
  float course_deg = 0.0F;         ///< True course over ground [degrees].
  float v_down_mps = 0.0F;         ///< True vertical velocity, NED convention: positive downwards [m/s].
  std::uint64_t timestamp_us = 0;  ///< Simulation clock at this step [microseconds].
};

/// How the receiver's velocity error is modelled; see the file comment. The
/// numeric values are the FMU's `velocity_error_model` parameter.
enum class GpsVelocityErrorModel : std::uint8_t
{
  kDoppler = 0,     ///< Independent white per-axis NED errors; speed and course derived (default).
  kSpeedCourse = 1  ///< White and correlated speed and course errors; the model before 2026-11-19.
};

/// @brief Maps the FMU's integer parameter onto the enum.
/// @return false, leaving `model` unchanged, for a code that is not a model.
[[nodiscard]] constexpr bool velocity_error_model_from_code(int code, GpsVelocityErrorModel& model)
{
  switch (code)
  {
    case static_cast<int>(GpsVelocityErrorModel::kDoppler):
      model = GpsVelocityErrorModel::kDoppler;
      return true;
    case static_cast<int>(GpsVelocityErrorModel::kSpeedCourse):
      model = GpsVelocityErrorModel::kSpeedCourse;
      return true;
    default:
      return false;
  }
}

/// Noise magnitudes and constant receiver-status fields applied by
/// GpsNoiseModel. The white terms' defaults approximate a u-blox M9N in
/// open sky; the correlated terms default to off (see the file comment).
struct GpsNoiseConfig
{
  // White per-epoch error, 1-sigma.
  float horizontal_pos_noise_m = 1.5F;  ///< Applied independently to north/east [m].
  float vertical_pos_noise_m = 3.0F;    ///< [m].
  float speed_noise_mps = 0.1F;         ///< [m/s]; speed/course model only.
  float course_noise_deg = 1.0F;        ///< [degrees]; speed/course model, and the Doppler model's low-speed course.

  /// Which velocity error model applies. The speed, course and vertical-speed
  /// terms below belong to kSpeedCourse; the doppler_* terms to kDoppler.
  GpsVelocityErrorModel velocity_error_model = GpsVelocityErrorModel::kDoppler;

  // Doppler model: white per-epoch NED velocity error, 1-sigma, independent
  // per axis and of speed. Vertical is twice horizontal, the same geometry
  // ratio as the position defaults. A zero sigma draws nothing.
  float doppler_north_noise_mps = 0.05F;  ///< [m/s].
  float doppler_east_noise_mps = 0.05F;   ///< [m/s].
  float doppler_down_noise_mps = 0.10F;   ///< [m/s], positive downwards as velD.

  // Time-correlated error: first-order Gauss-Markov per channel, stationary
  // 1-sigma and correlation time. A zero sigma disables the term.
  float horizontal_pos_correlated_m = 0.0F;    ///< Stationary 1-sigma, north and east independently [m].
  float vertical_pos_correlated_m = 0.0F;      ///< Stationary 1-sigma [m].
  float position_correlation_time_s = 100.0F;  ///< tau for the three position channels [s].
  float speed_correlated_mps = 0.0F;           ///< Stationary 1-sigma [m/s].
  float course_correlated_deg = 0.0F;          ///< Stationary 1-sigma [degrees].
  float velocity_correlation_time_s = 10.0F;   ///< tau for speed, course and vertical velocity [s].

  /// Vertical-velocity error of the speed/course model, white and correlated
  /// 1-sigma [m/s]. On by default, unlike the other correlated terms, because velD used to be a
  /// hard-coded zero rather than a quiet default: there is no previous
  /// behaviour to preserve. Twice speed_noise_mps, the same vertical-to-
  /// horizontal ratio as the position defaults -- GNSS geometry is weaker in
  /// the vertical for velocity as for position.
  float vertical_speed_noise_mps = 0.2F;
  float vertical_speed_correlated_mps = 0.0F;  ///< Stationary 1-sigma [m/s]; 0 disables the term.

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
    : config_(config)
    , rng_(seed)
    , vertical_velocity_rng_(seed ^ kVerticalVelocityStream)
    , doppler_rng_(seed ^ kDopplerStream)
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
    for (Channel* channel : { &north_, &east_, &vertical_, &speed_, &course_, &vertical_velocity_ })
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

  /// @brief The NED velocity error as one per-axis 1-sigma [m/s], before
  /// `accuracy_scale` -- what an honest receiver reports as `sAcc`.
  ///
  /// A consumer that weights a fix by `sAcc` uses it for all three axes alike
  /// (Aetherion's replay builds sAcc^2 I), so the honest single number is the
  /// one whose isotropic covariance has the same trace as the real one: the
  /// root of the mean of the three variances. Then the expected normalised
  /// innovation is right on average, even where no single axis is.
  ///
  /// Doppler: sqrt((sN^2 + sE^2 + sD^2) / 3), whatever the speed -- 0.071 m/s
  /// at the defaults. Speed/course: speed error lies along the track, course
  /// error across it at (speed x course error in radians), vertical error on
  /// its own, so the figure grows with speed: 1 deg of course error is 3 m/s
  /// across track at 172 m/s.
  ///
  /// @param ground_speed_mps Speed over ground the course error is scaled by
  ///                         [m/s]; unused by the Doppler model.
  [[nodiscard]] float velocity_sigma_mps(float ground_speed_mps) const
  {
    if (config_.velocity_error_model == GpsVelocityErrorModel::kDoppler)
    {
      const double north = config_.doppler_north_noise_mps;
      const double east = config_.doppler_east_noise_mps;
      const double down = config_.doppler_down_noise_mps;
      return static_cast<float>(std::sqrt((north * north + east * east + down * down) / 3.0));
    }
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double along = std::hypot(config_.speed_noise_mps, config_.speed_correlated_mps);
    const double across = static_cast<double>(ground_speed_mps) *
                          std::hypot(config_.course_noise_deg, config_.course_correlated_deg) * kDegToRad;
    const double down = std::hypot(config_.vertical_speed_noise_mps, config_.vertical_speed_correlated_mps);
    return static_cast<float>(std::sqrt((along * along + across * across + down * down) / 3.0));
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

    if (config_.velocity_error_model == GpsVelocityErrorModel::kDoppler)
    {
      apply_doppler_velocity(truth, fix);
    }
    else
    {
      apply_speed_course_velocity(truth, fix, dt_s);
    }

    fix.num_satellites = config_.num_satellites;
    fix.fix_type = config_.fix_type;
    fix.timestamp_us = truth.timestamp_us;
    return fix;
  }

  /// Below this many horizontal Doppler sigmas of derived speed, the course
  /// is not derived from the noisy velocity -- its direction would be mostly
  /// the noise's -- and the speed/course model's course is kept instead.
  /// Ten sigmas bounds the derived course's error near 6 deg (0.5 m/s at the
  /// defaults); a rocket on the pad stays well below it.
  static constexpr double kDopplerCourseSpeedFactor = 10.0;

private:
  /// Doppler model: truth NED velocity plus independent white per-axis
  /// errors from the Doppler stream; speed over ground and course derived
  /// from the noisy north and east, the low-speed course excepted. `fix`
  /// arrives with the main stream's speed and course already in it.
  void apply_doppler_velocity(const GpsTruthSample& truth, GpsFix& fix)
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    // The truth carries speed and course (the FMU's NED inputs are reduced to
    // them); resolve them back. In double, so the round trip costs ~1e-6 m/s.
    const double true_course_rad = static_cast<double>(truth.course_deg) * kDegToRad;
    const double true_speed_mps = truth.ground_speed_mps;
    const double north_mps = true_speed_mps * std::cos(true_course_rad) + doppler_draw(config_.doppler_north_noise_mps);
    const double east_mps = true_speed_mps * std::sin(true_course_rad) + doppler_draw(config_.doppler_east_noise_mps);
    const double down_mps = static_cast<double>(truth.v_down_mps) + doppler_draw(config_.doppler_down_noise_mps);

    fix.vel_north_mps = static_cast<float>(north_mps);
    fix.vel_east_mps = static_cast<float>(east_mps);
    fix.vel_down_mps = static_cast<float>(down_mps);
    const double speed_mps = std::hypot(north_mps, east_mps);
    fix.ground_speed_mps = static_cast<float>(speed_mps);

    const double horizontal_sigma_mps =
        std::sqrt((static_cast<double>(config_.doppler_north_noise_mps) * config_.doppler_north_noise_mps +
                   static_cast<double>(config_.doppler_east_noise_mps) * config_.doppler_east_noise_mps) /
                  2.0);
    if (speed_mps > 0.0 && speed_mps > kDopplerCourseSpeedFactor * horizontal_sigma_mps)
    {
      double course_deg = std::atan2(east_mps, north_mps) / kDegToRad;
      if (course_deg < 0.0)
      {
        course_deg += 360.0;
      }
      fix.course_deg = static_cast<float>(course_deg);
      if (fix.course_deg >= 360.0F)  // a course a hair below 360 can round up in float
      {
        fix.course_deg = 0.0F;
      }
    }
    // else: fix.course_deg keeps the speed/course model's value, set in apply().

    fix.speed_accuracy_mps = config_.accuracy_scale * velocity_sigma_mps(fix.ground_speed_mps);
    fix.ned_velocity_valid = true;
  }

  /// One Doppler-stream draw at `sigma`, or exactly nothing for a disabled
  /// axis -- no zero-width distribution is ever constructed.
  double doppler_draw(float sigma)
  {
    if (sigma <= 0.0F)
    {
      return 0.0;
    }
    return static_cast<double>(sigma * doppler_unit_(doppler_rng_));
  }

  /// Speed/course model: the noisy speed and course resolved into north and
  /// east, velD with its own terms. This is the model as it stood before the
  /// Doppler model, statement for statement; its draws must not change.
  void apply_speed_course_velocity(const GpsTruthSample& truth, GpsFix& fix, double dt_s)
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    // NED velocity. Horizontal: the noisy speed and course resolved, so the
    // fields agree as a real receiver's do. Vertical: its own terms, from its
    // own stream and its own distribution objects -- a normal_distribution
    // caches half of each pair, so even sharing `unit_` with the main stream
    // would shift that stream's draws.
    float vertical_velocity_error_mps = 0.0F;
    if (config_.vertical_speed_noise_mps > 0.0F)
    {
      vertical_velocity_error_mps =
          std::normal_distribution<float>(0.0F, config_.vertical_speed_noise_mps)(vertical_velocity_rng_);
    }
    vertical_velocity_error_mps += correlated(vertical_velocity_,
                                              config_.vertical_speed_correlated_mps,
                                              config_.velocity_correlation_time_s,
                                              dt_s,
                                              vertical_velocity_rng_,
                                              vertical_velocity_unit_);
    const double course_rad = static_cast<double>(fix.course_deg) * kDegToRad;
    fix.vel_north_mps = static_cast<float>(static_cast<double>(fix.ground_speed_mps) * std::cos(course_rad));
    fix.vel_east_mps = static_cast<float>(static_cast<double>(fix.ground_speed_mps) * std::sin(course_rad));
    fix.vel_down_mps = truth.v_down_mps + vertical_velocity_error_mps;
    fix.speed_accuracy_mps = config_.accuracy_scale * velocity_sigma_mps(fix.ground_speed_mps);
    fix.ned_velocity_valid = true;
  }

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
    return correlated(channel, sigma, tau_s, dt_s, rng_, unit_);
  }

  /// The same, drawing from a given engine through a given unit normal --
  /// which must be used with that engine only, for the reason in apply().
  static float correlated(Channel& channel,
                          float sigma,
                          float tau_s,
                          double dt_s,
                          std::mt19937_64& engine,
                          std::normal_distribution<float>& unit)
  {
    if (sigma <= 0.0F)
    {
      return 0.0F;
    }
    if (!channel.primed)
    {
      // Stationary start: the receiver has been tracking for a while before
      // the first fix this model sees, so its error is already "somewhere".
      channel.state = sigma * unit(engine);
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
    channel.state = static_cast<float>(phi * channel.state + static_cast<double>(sigma) * driving * unit(engine));
    return channel.state;
  }

  /// Mixed into the seed for the vertical-velocity stream. Any constant does;
  /// this one only has to differ from the other per-part offsets in the tree
  /// (the sensor clocks use 0x5DEECE66D).
  static constexpr std::uint64_t kVerticalVelocityStream = 0x9E3779B97F4A7C15ULL;

  /// Mixed into the seed for the Doppler velocity stream; differs from the
  /// vertical-velocity constant and the sensor clocks' 0x5DEECE66D.
  static constexpr std::uint64_t kDopplerStream = 0xD1B54A32D192ED03ULL;

  GpsNoiseConfig config_;
  std::mt19937_64 rng_;

  // Unit normal for the correlated terms. Only ever drawn for an enabled
  // channel, so it cannot disturb the white terms' sequence when all the
  // correlated sigmas are zero.
  std::normal_distribution<float> unit_{ 0.0F, 1.0F };

  // The vertical-velocity channel's stream, and its own unit normal.
  std::mt19937_64 vertical_velocity_rng_;
  std::normal_distribution<float> vertical_velocity_unit_{ 0.0F, 1.0F };

  // The Doppler model's stream and its own unit normal, for the same reason.
  std::mt19937_64 doppler_rng_;
  std::normal_distribution<float> doppler_unit_{ 0.0F, 1.0F };

  Channel north_;
  Channel east_;
  Channel vertical_;
  Channel speed_;
  Channel course_;
  Channel vertical_velocity_;

  std::uint64_t last_timestamp_us_ = 0;
  bool have_last_timestamp_ = false;
};

}  // namespace hemerion::sensors::gps::fmu
