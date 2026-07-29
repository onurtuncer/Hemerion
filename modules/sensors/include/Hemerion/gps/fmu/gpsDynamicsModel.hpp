// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsDynamicsModel.hpp
/// @brief Receiver dynamics envelope (speed / vertical-rate / acceleration /
/// altitude limits) for the GPS hardware-simulator FMU.
///
/// A real GNSS receiver does not keep reporting a valid solution however
/// hard the vehicle it is bolted to accelerates. Two independent mechanisms
/// take the fix away, and this model reproduces both:
///
/// 1. **Navigation-engine sanity checks.** u-blox receivers run their
///    solution against the limits of the configured *dynamic platform model*
///    (CFG-NAVSPG-DYNMODEL, `dynModel` in the legacy UBX-CFG-NAV5 message).
///    Each model declares a maximum altitude, horizontal velocity and
///    vertical velocity; a solution outside them is rejected rather than
///    output. The airborne models additionally name the acceleration the
///    tracking loops and navigation filter are tuned for (<1 g, <2 g, <4 g) --
///    which is exactly why a launch vehicle must be configured for
///    airborne <4 g and still loses the fix through a hard boost phase.
/// 2. **COCOM export limits.** Independently of the platform model, the
///    receiver stops producing navigation output when altitude *and* speed
///    both exceed the export-control thresholds (18 000 m and 515 m/s ~
///    1000 knots). u-blox applies these as an AND, so a high-altitude slow
///    balloon and a fast low-altitude sled both keep their fix; a sounding
///    rocket past first-stage burnout does not.
///
/// Both mechanisms are evaluated against the *truth* state, not the noisy
/// fix: it is the vehicle's real motion that breaks carrier tracking, not
/// the receiver's estimate of it. When either trips, the fix is degraded in
/// place -- fix type drops to `kNoFix` (so UbxEmitter clears NAV-PVT's
/// `gnssFixOK` flag), the satellite count goes to zero, and the
/// self-reported accuracy fields inflate to `kNoFixAccuracyM`. Position and
/// velocity fields keep carrying the invalid solution, as a real receiver
/// does; consumers are expected to gate on the fix type.
///
/// Recovery is not instantaneous either: after the vehicle re-enters the
/// envelope the model holds the fix invalid for `reacquisition_time_s`,
/// representing the re-acquisition the receiver has to do once the loops
/// have dropped lock.
///
/// Host-only, like the rest of the fmu/ subtree -- built only when
/// HEMERION_BUILD_FMU is on, never cross-compiled.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"

namespace hemerion::sensors::gps::fmu
{

/// @brief Navigation-engine dynamic platform model.
///
/// Enumerator values are the u-blox `dynModel` codes themselves
/// (CFG-NAVSPG-DYNMODEL / UBX-CFG-NAV5), so the FMU's `dynamic_platform`
/// parameter takes the same number a firmware engineer would write into the
/// receiver -- 8 for airborne <4 g, and so on. Code 1 is reserved by u-blox
/// and has no enumerator here.
enum class GpsDynamicPlatform : std::int8_t
{
  kUnlimited = -1,  ///< Not a u-blox code: disables every envelope check (COCOM stays separate).
  kPortable = 0,    ///< u-blox default model.
  kStationary = 2,  ///< Fixed installation.
  kPedestrian = 3,  ///< Walking speeds.
  kAutomotive = 4,  ///< Ground vehicle.
  kSea = 5,         ///< Surface vessel, sea level.
  kAirborne1g = 6,  ///< Airborne, up to 1 g manoeuvres.
  kAirborne2g = 7,  ///< Airborne, up to 2 g manoeuvres.
  kAirborne4g = 8,  ///< Airborne, up to 4 g manoeuvres -- the launch-vehicle setting.
  kWrist = 9,       ///< Wrist-worn device.
  kBike = 10,       ///< Bicycle.
};

/// Envelope a dynamic platform model declares. Solutions outside it are
/// rejected by the receiver's navigation engine.
struct GpsDynamicsLimits
{
  float max_altitude_m;            ///< Maximum altitude above MSL [m].
  float max_horizontal_speed_mps;  ///< Maximum horizontal velocity [m/s].
  float max_vertical_speed_mps;    ///< Maximum vertical velocity, either sign [m/s].
  float max_acceleration_mps2;     ///< Maximum total acceleration the navigation filter tracks [m/s^2].
};

/// @brief Envelope for one dynamic platform model.
///
/// The altitude and velocity columns are the u-blox "dynamic platform model
/// details" table (u-blox 8 / M8 / M9 interface descriptions), reproduced as
/// published. The acceleration column is *not* in that table: for the three
/// airborne models it is the g-class in the model's own name (1 g / 2 g /
/// 4 g of standard gravity), and for the remaining models it is a
/// representative figure for that kind of platform rather than a
/// manufacturer specification -- documented as such so nobody mistakes it
/// for a datasheet number.
[[nodiscard]] constexpr GpsDynamicsLimits limits_for(GpsDynamicPlatform platform)
{
  constexpr float kG = 9.80665F;
  constexpr float kInf = std::numeric_limits<float>::infinity();

  switch (platform)
  {
    case GpsDynamicPlatform::kPortable:
      return { 12000.0F, 310.0F, 50.0F, 2.0F * kG };
    case GpsDynamicPlatform::kStationary:
      return { 9000.0F, 10.0F, 6.0F, 0.2F * kG };
    case GpsDynamicPlatform::kPedestrian:
      return { 9000.0F, 30.0F, 20.0F, 1.0F * kG };
    case GpsDynamicPlatform::kAutomotive:
      return { 6000.0F, 100.0F, 15.0F, 1.0F * kG };
    case GpsDynamicPlatform::kSea:
      return { 500.0F, 25.0F, 5.0F, 0.5F * kG };
    case GpsDynamicPlatform::kAirborne1g:
      return { 80000.0F, 100.0F, 6.0F, 1.0F * kG };
    case GpsDynamicPlatform::kAirborne2g:
      return { 80000.0F, 250.0F, 15.0F, 2.0F * kG };
    case GpsDynamicPlatform::kAirborne4g:
      return { 80000.0F, 500.0F, 100.0F, 4.0F * kG };
    case GpsDynamicPlatform::kWrist:
      return { 9000.0F, 30.0F, 20.0F, 4.0F * kG };
    case GpsDynamicPlatform::kBike:
      return { 6000.0F, 100.0F, 15.0F, 1.0F * kG };
    case GpsDynamicPlatform::kUnlimited:
    default:
      return { kInf, kInf, kInf, kInf };
  }
}

/// @brief Maps a raw u-blox `dynModel` code (or -1 for kUnlimited) to a
/// platform.
/// @param code Wire-level dynModel code.
/// @param out  Set to the matching platform only when the code is known.
/// @return False if the code is not one this model knows (u-blox reserves
///         code 1), leaving `out` untouched.
[[nodiscard]] constexpr bool platform_from_code(int code, GpsDynamicPlatform& out)
{
  switch (code)
  {
    case -1:
    case 0:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      out = static_cast<GpsDynamicPlatform>(code);
      return true;
    default:
      return false;
  }
}

/// COCOM (export-control) altitude threshold [m]. Navigation output stops
/// only when this *and* kCocomSpeedLimitMps are both exceeded.
inline constexpr float kCocomAltitudeLimitM = 18000.0F;

/// COCOM (export-control) speed threshold [m/s]. u-blox states 515 m/s; the
/// underlying regulation is 1000 knots (514.4 m/s).
inline constexpr float kCocomSpeedLimitMps = 515.0F;

/// Self-reported accuracy a receiver falls back to while it has no valid
/// solution [m]. Real parts report kilometre-scale (or saturated) hAcc/vAcc
/// there; the exact figure is meaningless, its magnitude is the point.
inline constexpr float kNoFixAccuracyM = 1000.0F;

/// Why a fix was (or was not) invalidated. Ordered checks: the first limit
/// that trips is the one reported.
enum class GpsDynamicsVerdict : std::uint8_t
{
  kValid,                 ///< Inside the envelope and past any re-acquisition hold-off.
  kCocomLimit,            ///< Altitude and speed both past the export-control thresholds.
  kAltitudeLimit,         ///< Above the platform model's maximum altitude.
  kHorizontalSpeedLimit,  ///< Faster than the platform model's maximum horizontal velocity.
  kVerticalSpeedLimit,    ///< Climbing/descending faster than the platform model allows.
  kAccelerationLimit,     ///< Accelerating harder than the navigation filter tracks.
  kReacquiring,           ///< Back inside the envelope, still re-acquiring.
};

/// Configuration of GpsDynamicsModel. Defaults describe a stock u-blox M9N
/// configured for a launch vehicle: airborne <4 g, COCOM limits in force (an
/// export-licensed receiver would clear that flag).
struct GpsDynamicsConfig
{
  GpsDynamicPlatform platform = GpsDynamicPlatform::kAirborne4g;  ///< Navigation-engine platform model.
  bool cocom_limits_enabled = true;                               ///< Apply the COCOM altitude+speed cut-off.
  float reacquisition_time_s = 2.0F;  ///< Hold-off before a fix returns after any limit trips [s].
};

/// @brief Invalidates fixes whose truth state is outside the receiver's
/// dynamics envelope, with a re-acquisition hold-off afterwards.
///
/// Stateful across steps: acceleration is estimated by differencing the
/// truth velocity between consecutive samples, so apply() must be called
/// once per navigation epoch, in order.
class GpsDynamicsModel
{
public:
  /// @param config Platform model, COCOM switch and re-acquisition time.
  explicit GpsDynamicsModel(const GpsDynamicsConfig& config = {}) : config_(config) {}

  /// @brief Evaluates one truth sample and degrades `fix` if the receiver
  /// would not have produced a valid solution for it.
  ///
  /// @param truth Noiseless trajectory sample for this navigation epoch;
  ///              `timestamp_us` must advance between calls for the
  ///              acceleration check to engage (the first sample never
  ///              trips it).
  /// @param fix   Fix to gate, modified in place when a limit is active.
  /// @return Which limit tripped, or kValid.
  GpsDynamicsVerdict apply(const GpsTruthSample& truth, GpsFix& fix)
  {
    const GpsDynamicsVerdict verdict = evaluate(truth);
    if (verdict != GpsDynamicsVerdict::kValid)
    {
      fix.fix_type = GpsFixType::kNoFix;
      fix.num_satellites = 0;
      fix.horizontal_accuracy_m = kNoFixAccuracyM;
      fix.vertical_accuracy_m = kNoFixAccuracyM;
    }
    return verdict;
  }

  /// @brief Replaces the configuration and clears the derived state.
  ///
  /// Changing the platform mid-run would otherwise be judged against an
  /// acceleration estimate and a hold-off belonging to the previous one.
  void set_config(const GpsDynamicsConfig& config)
  {
    config_ = config;
    reset();
  }

  /// The configuration currently in force.
  [[nodiscard]] const GpsDynamicsConfig& config() const { return config_; }

  /// The envelope currently in force.
  [[nodiscard]] GpsDynamicsLimits limits() const { return limits_for(config_.platform); }

  /// @brief Forgets the previous sample and any pending hold-off, leaving
  /// the configuration alone (fmi2Reset's velocity/history equivalent).
  void reset()
  {
    has_previous_sample_ = false;
    previous_timestamp_us_ = 0;
    previous_v_north_mps_ = 0.0;
    previous_v_east_mps_ = 0.0;
    previous_v_down_mps_ = 0.0;
    reacquired_at_us_ = 0;
    acceleration_mps2_ = 0.0;
  }

  /// Total acceleration estimated from the last pair of truth samples
  /// [m/s^2]; zero until two samples have been seen.
  [[nodiscard]] double acceleration_mps2() const { return acceleration_mps2_; }

private:
  [[nodiscard]] GpsDynamicsVerdict evaluate(const GpsTruthSample& truth)
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double course_rad = static_cast<double>(truth.course_deg) * kDegToRad;
    const double horizontal_speed_mps = static_cast<double>(truth.ground_speed_mps);
    const double v_north_mps = horizontal_speed_mps * std::cos(course_rad);
    const double v_east_mps = horizontal_speed_mps * std::sin(course_rad);
    const double v_down_mps = static_cast<double>(truth.v_down_mps);

    // Acceleration is what the receiver's tracking loops actually feel, so
    // it is the magnitude of the 3-D velocity difference over the epoch, not
    // a per-axis figure.
    acceleration_mps2_ = 0.0;
    if (has_previous_sample_ && truth.timestamp_us > previous_timestamp_us_)
    {
      const double dt_s = static_cast<double>(truth.timestamp_us - previous_timestamp_us_) * 1e-6;
      const double dv_north = v_north_mps - previous_v_north_mps_;
      const double dv_east = v_east_mps - previous_v_east_mps_;
      const double dv_down = v_down_mps - previous_v_down_mps_;
      acceleration_mps2_ = std::sqrt(dv_north * dv_north + dv_east * dv_east + dv_down * dv_down) / dt_s;
    }
    has_previous_sample_ = true;
    previous_timestamp_us_ = truth.timestamp_us;
    previous_v_north_mps_ = v_north_mps;
    previous_v_east_mps_ = v_east_mps;
    previous_v_down_mps_ = v_down_mps;

    const GpsDynamicsLimits limits = limits_for(config_.platform);
    const double speed_3d_mps = std::sqrt(horizontal_speed_mps * horizontal_speed_mps + v_down_mps * v_down_mps);
    const double altitude_m = static_cast<double>(truth.altitude_m);

    GpsDynamicsVerdict verdict = GpsDynamicsVerdict::kValid;
    if (config_.cocom_limits_enabled && altitude_m > static_cast<double>(kCocomAltitudeLimitM) &&
        speed_3d_mps > static_cast<double>(kCocomSpeedLimitMps))
    {
      verdict = GpsDynamicsVerdict::kCocomLimit;
    }
    else if (altitude_m > static_cast<double>(limits.max_altitude_m))
    {
      verdict = GpsDynamicsVerdict::kAltitudeLimit;
    }
    else if (horizontal_speed_mps > static_cast<double>(limits.max_horizontal_speed_mps))
    {
      verdict = GpsDynamicsVerdict::kHorizontalSpeedLimit;
    }
    else if (std::fabs(v_down_mps) > static_cast<double>(limits.max_vertical_speed_mps))
    {
      verdict = GpsDynamicsVerdict::kVerticalSpeedLimit;
    }
    else if (acceleration_mps2_ > static_cast<double>(limits.max_acceleration_mps2))
    {
      verdict = GpsDynamicsVerdict::kAccelerationLimit;
    }

    if (verdict != GpsDynamicsVerdict::kValid)
    {
      const auto hold_off_us =
          static_cast<std::uint64_t>(static_cast<double>(std::max(0.0F, config_.reacquisition_time_s)) * 1e6);
      reacquired_at_us_ = truth.timestamp_us + hold_off_us;
      return verdict;
    }
    if (truth.timestamp_us < reacquired_at_us_)
    {
      return GpsDynamicsVerdict::kReacquiring;
    }
    return GpsDynamicsVerdict::kValid;
  }

  GpsDynamicsConfig config_;
  bool has_previous_sample_ = false;
  std::uint64_t previous_timestamp_us_ = 0;
  double previous_v_north_mps_ = 0.0;
  double previous_v_east_mps_ = 0.0;
  double previous_v_down_mps_ = 0.0;
  std::uint64_t reacquired_at_us_ = 0;
  double acceleration_mps2_ = 0.0;
};

}  // namespace hemerion::sensors::gps::fmu
