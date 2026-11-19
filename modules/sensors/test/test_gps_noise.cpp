// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_gps_noise.cpp
//
// Error model of the GPS FMU simulator
// (modules/sensors/include/Hemerion/gps/fmu/gpsNoiseModel.hpp): the white
// per-epoch term, the time-correlated Gauss-Markov term, the reported
// accuracy, reproducibility from a seed, and the two velocity error models --
// per-axis Doppler (the default) and speed/course, which must reproduce the
// model as it stood before Doppler existed, bit for bit.
//
// These assert *statistics*, not that the code ran: the sample
// autocorrelation of a long error sequence at chosen lags, and its RMS,
// against what the configured model predicts. A white model must show no
// correlation at lag 1; a Gauss-Markov model must show exp(-1) at lag tau
// and nothing at 5 tau. The sequences are 20 000 fixes long, so the standard
// error of a sample correlation is about 0.007 and the bands below are
// several of those wide -- and every model is seeded, so a failure is a
// change in the model, not a bad draw.
//
// Plain asserts + exit code, matching test_gps_dynamics.cpp.
// ------------------------------------------------------------------------------
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numbers>
#include <vector>

#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::fmu::GpsNoiseConfig;
using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;
using hemerion::sensors::gps::fmu::GpsVelocityErrorModel;

namespace
{

constexpr double kMetersPerDegLat = 111320.0;
constexpr std::uint64_t kEpochUs = 100000;  // 10 Hz, as the FMU is stepped
constexpr int kSamples = 20000;             // 2000 s of fixes

GpsTruthSample truth_at(std::uint64_t timestamp_us)
{
  GpsTruthSample truth;
  truth.latitude_deg = 36.019167;  // Kitty Hawk, as the F-16 cases
  truth.longitude_deg = -75.674444;
  truth.altitude_m = 3052.0F;
  truth.ground_speed_mps = 172.0F;
  truth.course_deg = 45.0F;
  truth.timestamp_us = timestamp_us;
  return truth;
}

/// North position error of one fix [m], the channel every statistic below is
/// read off. Latitude is the cleanest channel to invert: its scale is a
/// constant, so the error comes back exactly as it went in.
double north_error_m(const GpsFix& fix, const GpsTruthSample& truth)
{
  return (fix.latitude_deg - truth.latitude_deg) * kMetersPerDegLat;
}

/// A sequence of north errors at 10 Hz from a model.
std::vector<double> north_errors(GpsNoiseModel& model, int count)
{
  std::vector<double> errors;
  errors.reserve(static_cast<std::size_t>(count));
  for (int k = 0; k < count; ++k)
  {
    const GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k + 1) * kEpochUs);
    errors.push_back(north_error_m(model.apply(truth), truth));
  }
  return errors;
}

double mean(const std::vector<double>& x)
{
  double sum = 0.0;
  for (const double v : x)
  {
    sum += v;
  }
  return sum / static_cast<double>(x.size());
}

double rms(const std::vector<double>& x)
{
  double sum = 0.0;
  for (const double v : x)
  {
    sum += v * v;
  }
  return std::sqrt(sum / static_cast<double>(x.size()));
}

/// Normalised sample autocorrelation at `lag`.
double autocorrelation(const std::vector<double>& x, std::size_t lag)
{
  const double m = mean(x);
  double numerator = 0.0;
  double denominator = 0.0;
  for (std::size_t k = 0; k < x.size(); ++k)
  {
    denominator += (x[k] - m) * (x[k] - m);
    if (k + lag < x.size())
    {
      numerator += (x[k] - m) * (x[k + lag] - m);
    }
  }
  return numerator / denominator;
}

bool near(double value, double expected, double tolerance) { return std::fabs(value - expected) <= tolerance; }

/// The speed/course model, every other setting as `base`.
GpsNoiseConfig speed_course(GpsNoiseConfig base = {})
{
  base.velocity_error_model = GpsVelocityErrorModel::kSpeedCourse;
  return base;
}

// Same seed, same configuration: the same fixes, bit for bit. This is what
// the FMU's `seed` parameter buys a co-simulation.
void test_seed_reproduces_the_sequence()
{
  GpsNoiseConfig config;
  config.horizontal_pos_correlated_m = 2.0F;
  GpsNoiseModel a(config, /*seed=*/42);
  GpsNoiseModel b(config, /*seed=*/42);
  for (int k = 1; k <= 200; ++k)
  {
    const GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    const GpsFix fa = a.apply(truth);
    const GpsFix fb = b.apply(truth);
    assert(fa.latitude_deg == fb.latitude_deg);
    assert(fa.longitude_deg == fb.longitude_deg);
    assert(fa.altitude_m == fb.altitude_m);
    assert(fa.ground_speed_mps == fb.ground_speed_mps);
    assert(fa.course_deg == fb.course_deg);
  }
  GpsNoiseModel c(config, /*seed=*/43);
  const GpsTruthSample truth = truth_at(kEpochUs);
  assert(a.apply(truth).latitude_deg != c.apply(truth).latitude_deg);
}

// The default configuration is the previous model: white noise at the
// configured sigma and no memory between epochs.
void test_defaults_are_white()
{
  GpsNoiseModel model(GpsNoiseConfig{}, /*seed=*/1);
  const std::vector<double> errors = north_errors(model, kSamples);

  assert(near(rms(errors), 1.5, 0.05));                  // sigma recovered to ~3 %
  assert(near(mean(errors), 0.0, 0.05));                 // unbiased
  assert(std::fabs(autocorrelation(errors, 1)) < 0.03);  // no memory: ~4 standard errors
  assert(std::fabs(autocorrelation(errors, 100)) < 0.03);
}

// A Gauss-Markov term alone: stationary at its sigma, exp(-1) correlated at
// one correlation time, decorrelated well before five.
void test_correlated_term_is_gauss_markov()
{
  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 0.0F;  // isolate the correlated term
  config.horizontal_pos_correlated_m = 2.0F;
  config.position_correlation_time_s = 10.0F;  // 100 epochs at 10 Hz
  GpsNoiseModel model(config, /*seed=*/2);
  const std::vector<double> errors = north_errors(model, kSamples);

  // Stationary variance. The band is wider than the white test's because
  // correlated samples carry fewer effective degrees of freedom.
  assert(near(rms(errors), 2.0, 0.2));
  assert(near(autocorrelation(errors, 1), std::exp(-0.1 / 10.0), 0.02));  // 0.990
  assert(near(autocorrelation(errors, 100), std::exp(-1.0), 0.06));       // 0.368 at lag tau
  assert(std::fabs(autocorrelation(errors, 500)) < 0.1);                  // 0.007 at 5 tau
}

// White and correlated together: the RMS adds in quadrature and the
// autocorrelation at lag tau is the correlated term's share of the variance
// times exp(-1) -- the white term dilutes it, it does not remove it.
void test_terms_add_in_quadrature()
{
  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 1.5F;
  config.horizontal_pos_correlated_m = 2.0F;
  config.position_correlation_time_s = 10.0F;
  GpsNoiseModel model(config, /*seed=*/3);
  const std::vector<double> errors = north_errors(model, kSamples);

  const double total = std::hypot(1.5, 2.0);  // 2.5
  assert(near(rms(errors), total, 0.15));
  const double share = (2.0 * 2.0) / (total * total);                        // 0.64
  assert(near(autocorrelation(errors, 100), share * std::exp(-1.0), 0.06));  // 0.235
}

// A zero or negative correlation time degenerates to white, cleanly.
void test_zero_correlation_time_is_white()
{
  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 0.0F;
  config.horizontal_pos_correlated_m = 2.0F;
  config.position_correlation_time_s = 0.0F;
  GpsNoiseModel model(config, /*seed=*/4);
  const std::vector<double> errors = north_errors(model, kSamples);

  assert(near(rms(errors), 2.0, 0.07));
  assert(std::fabs(autocorrelation(errors, 1)) < 0.03);
}

// The reported accuracy is the total 1-sigma scaled by accuracy_scale: an
// honest receiver at 1, an optimistic one below it.
void test_reported_accuracy()
{
  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 0.3F;
  config.horizontal_pos_correlated_m = 0.4F;  // total 0.5
  config.vertical_pos_noise_m = 0.6F;
  config.vertical_pos_correlated_m = 0.8F;  // total 1.0
  GpsNoiseModel honest(config, /*seed=*/5);
  GpsFix fix = honest.apply(truth_at(kEpochUs));
  assert(near(fix.horizontal_accuracy_m, 0.5, 1e-6));
  assert(near(fix.vertical_accuracy_m, 1.0, 1e-6));

  config.accuracy_scale = 0.7F;
  GpsNoiseModel optimistic(config, /*seed=*/5);
  fix = optimistic.apply(truth_at(kEpochUs));
  assert(near(fix.horizontal_accuracy_m, 0.35, 1e-6));
  assert(near(fix.vertical_accuracy_m, 0.7, 1e-6));

  // The default reports exactly the previous model's values.
  GpsNoiseModel previous(GpsNoiseConfig{}, /*seed=*/5);
  fix = previous.apply(truth_at(kEpochUs));
  assert(fix.horizontal_accuracy_m == 1.5F);
  assert(fix.vertical_accuracy_m == 3.0F);
}

// The correlated state holds when the clock does not advance, and starts
// from its stationary distribution after a reset rather than from zero.
void test_state_hold_and_reset()
{
  GpsNoiseConfig config;
  config.horizontal_pos_noise_m = 0.0F;
  config.horizontal_pos_correlated_m = 2.0F;
  GpsNoiseModel model(config, /*seed=*/6);

  const GpsTruthSample truth = truth_at(kEpochUs);
  const double first = north_error_m(model.apply(truth), truth);
  const double again = north_error_m(model.apply(truth), truth);  // same timestamp
  assert(first == again);

  // Over many resets the very first error of each run is a fresh stationary
  // draw: its spread is the configured sigma, not zero.
  std::vector<double> firsts;
  for (int k = 0; k < 2000; ++k)
  {
    model.reset_state();
    firsts.push_back(north_error_m(model.apply(truth), truth));
  }
  assert(near(rms(firsts), 2.0, 0.12));
}

// Behaviours the previous model had, kept: speed floored at zero, course
// wrapped into [0, 360).
void test_speed_floor_and_course_wrap()
{
  GpsNoiseConfig config;
  config.speed_noise_mps = 5.0F;
  config.course_noise_deg = 30.0F;
  GpsNoiseModel model(config, /*seed=*/7);
  for (int k = 1; k <= 2000; ++k)
  {
    GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    truth.ground_speed_mps = 0.0F;
    truth.course_deg = 359.0F;
    const GpsFix fix = model.apply(truth);
    assert(fix.ground_speed_mps >= 0.0F);
    assert(fix.course_deg >= 0.0F && fix.course_deg < 360.0F);
  }
}

// The vertical-velocity channel draws from its own stream, so turning it on
// -- white and correlated -- moves no draw of any other channel. Every other
// correlated term is on too: that is the case a shared normal_distribution
// would break, since it caches half of each pair across calls.
void test_vertical_velocity_moves_no_other_channel()
{
  GpsNoiseConfig without = speed_course();
  without.horizontal_pos_correlated_m = 1.5F;
  without.vertical_pos_correlated_m = 3.0F;
  without.speed_correlated_mps = 0.1F;
  without.course_correlated_deg = 1.0F;
  without.vertical_speed_noise_mps = 0.0F;
  without.vertical_speed_correlated_mps = 0.0F;
  GpsNoiseConfig with = without;
  with.vertical_speed_noise_mps = 0.3F;
  with.vertical_speed_correlated_mps = 0.4F;

  GpsNoiseModel a(without, /*seed=*/21);
  GpsNoiseModel b(with, /*seed=*/21);
  bool vertical_differs = false;
  for (int k = 1; k <= 2000; ++k)
  {
    GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    truth.v_down_mps = -4.0F;
    const GpsFix fa = a.apply(truth);
    const GpsFix fb = b.apply(truth);
    assert(fa.latitude_deg == fb.latitude_deg);
    assert(fa.longitude_deg == fb.longitude_deg);
    assert(fa.altitude_m == fb.altitude_m);
    assert(fa.ground_speed_mps == fb.ground_speed_mps);
    assert(fa.course_deg == fb.course_deg);
    assert(fa.vel_north_mps == fb.vel_north_mps);
    assert(fa.vel_east_mps == fb.vel_east_mps);
    assert(fa.vel_down_mps == -4.0F);  // off: exactly the truth
    vertical_differs = vertical_differs || (fb.vel_down_mps != fa.vel_down_mps);
  }
  assert(vertical_differs);
}

// NAV-PVT's fields agree with each other, as a real receiver's do: the
// horizontal velocity is the speed along the course -- in the speed/course
// model by construction, in the Doppler model because speed and course are
// derived from the noisy north and east components.
void test_velocity_agrees_with_speed_and_course()
{
  for (const GpsNoiseConfig& config : { speed_course(), GpsNoiseConfig{} })
  {
    GpsNoiseModel model(config, /*seed=*/22);
    for (int k = 1; k <= 500; ++k)
    {
      const GpsFix fix = model.apply(truth_at(static_cast<std::uint64_t>(k) * kEpochUs));
      assert(fix.ned_velocity_valid);
      assert(near(std::hypot(fix.vel_north_mps, fix.vel_east_mps), fix.ground_speed_mps, 1e-3));
      double course = std::atan2(fix.vel_east_mps, fix.vel_north_mps) * 180.0 / std::numbers::pi;
      course = std::fmod(course + 360.0, 360.0);
      assert(near(course, fix.course_deg, 1e-3));
    }
  }
}

// velD is the truth's vertical velocity plus the configured noise: unbiased,
// white at the default sigma, and Gauss-Markov when the correlated term is on.
void test_vertical_velocity_statistics()
{
  std::vector<double> errors;
  GpsNoiseModel white(speed_course(), /*seed=*/23);
  for (int k = 1; k <= kSamples; ++k)
  {
    GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    truth.v_down_mps = -5.0F;
    errors.push_back(static_cast<double>(white.apply(truth).vel_down_mps) + 5.0);
  }
  assert(near(rms(errors), 0.2, 0.007));
  assert(near(mean(errors), 0.0, 0.005));
  assert(std::fabs(autocorrelation(errors, 1)) < 0.03);

  GpsNoiseConfig config = speed_course();
  config.vertical_speed_noise_mps = 0.0F;
  config.vertical_speed_correlated_mps = 0.5F;
  config.velocity_correlation_time_s = 10.0F;  // 100 epochs
  GpsNoiseModel correlated(config, /*seed=*/24);
  errors.clear();
  for (int k = 1; k <= kSamples; ++k)
  {
    const GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    errors.push_back(static_cast<double>(correlated.apply(truth).vel_down_mps));
  }
  assert(near(rms(errors), 0.5, 0.05));
  assert(near(autocorrelation(errors, 100), std::exp(-1.0), 0.06));
}

// sAcc is honest in the sense an isotropic consumer needs: sAcc^2 is the mean
// per-axis variance of the actual NED velocity error. Measured, not asserted
// from the formula -- at 172 m/s, where the course term dominates (1 deg is
// 3 m/s across track) and the error is far from isotropic.
void test_speed_accuracy_is_honest_on_average()
{
  GpsNoiseModel model(speed_course(), /*seed=*/25);
  constexpr double kDeg = std::numbers::pi / 180.0;
  double sum_squared = 0.0;
  double reported = 0.0;
  for (int k = 1; k <= kSamples; ++k)
  {
    GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    truth.v_down_mps = 2.0F;
    const GpsFix fix = model.apply(truth);
    const double dn = fix.vel_north_mps - (172.0 * std::cos(45.0 * kDeg));
    const double de = fix.vel_east_mps - (172.0 * std::sin(45.0 * kDeg));
    const double dd = fix.vel_down_mps - 2.0;
    sum_squared += (dn * dn) + (de * de) + (dd * dd);
    reported += fix.speed_accuracy_mps;
  }
  const double per_axis = std::sqrt(sum_squared / (3.0 * kSamples));
  reported /= kSamples;
  assert(near(reported, 1.737, 0.01));                // sqrt((0.1^2 + (172 * 1 deg)^2 + 0.2^2) / 3)
  assert(near(per_axis, reported, 0.02 * reported));  // what it does, to 2 %

  // An optimistic receiver scales sAcc like hAcc and vAcc; a slow one
  // reports little more than its speed and vertical terms.
  GpsNoiseConfig optimistic = speed_course();
  optimistic.accuracy_scale = 0.7F;
  GpsNoiseModel scaled(optimistic, /*seed=*/25);
  assert(near(scaled.apply(truth_at(kEpochUs)).speed_accuracy_mps, 0.7 * model.velocity_sigma_mps(172.0F), 0.01));
  assert(near(model.velocity_sigma_mps(0.0F), std::sqrt((0.01 + 0.04) / 3.0), 1e-6));
}

// ---------------------------------------------------------------------------
// Velocity error models
// ---------------------------------------------------------------------------

/// FNV-1a over the bytes of every numeric field of the fixes of a fixed
/// scenario: three configurations (defaults; every correlated term on; that
/// with an optimistic receiver and no white speed error), 2000 fixes each,
/// every third one at zero speed, velD non-zero.
std::uint64_t scenario_hash(const GpsNoiseConfig& base)
{
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  const auto mix = [&hash](const auto& value) {
    unsigned char bytes[sizeof value];
    std::memcpy(bytes, &value, sizeof value);
    for (const unsigned char byte : bytes)
    {
      hash = (hash ^ byte) * 0x100000001b3ULL;
    }
  };
  for (int variant = 0; variant < 3; ++variant)
  {
    GpsNoiseConfig config = base;
    if (variant >= 1)
    {
      config.horizontal_pos_correlated_m = 1.5F;
      config.vertical_pos_correlated_m = 3.0F;
      config.speed_correlated_mps = 0.1F;
      config.course_correlated_deg = 1.0F;
      config.vertical_speed_correlated_mps = 0.2F;
    }
    if (variant == 2)
    {
      config.accuracy_scale = 0.7F;
      config.speed_noise_mps = 0.0F;
    }
    GpsNoiseModel model(config, 17U + static_cast<std::uint64_t>(variant));
    for (int k = 1; k <= 2000; ++k)
    {
      GpsTruthSample truth = truth_at(100000ULL * static_cast<std::uint64_t>(k));
      truth.altitude_m = 3052.0F;
      truth.ground_speed_mps = (k % 3 == 0) ? 0.0F : 172.0F;
      truth.course_deg = 45.0F + 0.01F * static_cast<float>(k);
      truth.v_down_mps = -3.0F;
      const GpsFix fix = model.apply(truth);
      mix(fix.latitude_deg);
      mix(fix.longitude_deg);
      mix(fix.altitude_m);
      mix(fix.ground_speed_mps);
      mix(fix.course_deg);
      mix(fix.vel_north_mps);
      mix(fix.vel_east_mps);
      mix(fix.vel_down_mps);
      mix(fix.speed_accuracy_mps);
      mix(fix.horizontal_accuracy_m);
      mix(fix.vertical_accuracy_m);
    }
  }
  return hash;
}

// Speed/course mode is the model as it stood before the Doppler model
// (commit 2ad8c56), bit for bit: the scenario's hash is the one that model
// produced. std::normal_distribution is not specified draw for draw, so the
// reference is per standard library -- both computed from 2ad8c56's
// gpsNoiseModel.hpp on 2026-11-19 (clang 20 on the MSVC STL; g++ 13 on
// libstdc++), each identical at -O0 and -O2. Another library has no
// reference and is reported, not failed.
void test_speed_course_reproduces_the_previous_model()
{
  const std::uint64_t hash = scenario_hash(speed_course());
#if defined(_MSVC_STL_VERSION)
  assert(hash == 0x6035EC2ED604FADCULL);
#elif defined(__GLIBCXX__)
  assert(hash == 0x6714E70BACB962FDULL);
#else
  std::printf("test_gps_noise: no speed/course reference for this standard library (hash 0x%016llX)\n",
              static_cast<unsigned long long>(hash));
#endif
  // And the Doppler model is something else.
  assert(scenario_hash(GpsNoiseConfig{}) != hash);
}

// Which velocity model is selected moves no position draw: the main stream
// draws speed and course errors in both, so latitude, longitude and altitude
// are identical, and so are hAcc and vAcc.
void test_velocity_model_moves_no_position_draw()
{
  GpsNoiseConfig doppler;
  doppler.horizontal_pos_correlated_m = 1.5F;
  doppler.course_correlated_deg = 1.0F;
  GpsNoiseModel a(doppler, /*seed=*/31);
  GpsNoiseModel b(speed_course(doppler), /*seed=*/31);
  for (int k = 1; k <= 2000; ++k)
  {
    const GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    const GpsFix fa = a.apply(truth);
    const GpsFix fb = b.apply(truth);
    assert(fa.latitude_deg == fb.latitude_deg);
    assert(fa.longitude_deg == fb.longitude_deg);
    assert(fa.altitude_m == fb.altitude_m);
    assert(fa.horizontal_accuracy_m == fb.horizontal_accuracy_m);
    assert(fa.vertical_accuracy_m == fb.vertical_accuracy_m);
  }
}

/// Per-axis NED velocity errors of a run of Doppler fixes.
struct AxisErrors
{
  std::vector<double> north;
  std::vector<double> east;
  std::vector<double> down;
};

AxisErrors doppler_errors(GpsNoiseModel& model, float speed_mps, int count)
{
  constexpr double kDeg = std::numbers::pi / 180.0;
  AxisErrors errors;
  for (int k = 1; k <= count; ++k)
  {
    GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    truth.ground_speed_mps = speed_mps;
    truth.v_down_mps = -7.0F;
    const GpsFix fix = model.apply(truth);
    errors.north.push_back(fix.vel_north_mps - (speed_mps * std::cos(45.0 * kDeg)));
    errors.east.push_back(fix.vel_east_mps - (speed_mps * std::sin(45.0 * kDeg)));
    errors.down.push_back(static_cast<double>(fix.vel_down_mps) + 7.0);
  }
  return errors;
}

/// The sample sigma matches `sigma` to four standard errors (sigma / sqrt(2n)
/// for a normal sample), the mean is zero to four (sigma / sqrt(n)), and there
/// is no memory between fixes.
void assert_white_at(const std::vector<double>& errors, double sigma)
{
  const double n = static_cast<double>(errors.size());
  assert(near(rms(errors), sigma, 4.0 * sigma / std::sqrt(2.0 * n)));
  assert(near(mean(errors), 0.0, 4.0 * sigma / std::sqrt(n)));
  assert(std::fabs(autocorrelation(errors, 1)) < 0.03);
}

// The Doppler default: white per-axis errors at 0.05, 0.05 and 0.10 m/s, the
// same at rest as at 172 m/s -- speed-independent, unlike the course term of
// the speed/course model, which is 3 m/s across track at that speed.
void test_doppler_axes_are_white_and_speed_independent()
{
  const GpsNoiseConfig defaults;
  assert(defaults.velocity_error_model == GpsVelocityErrorModel::kDoppler);
  assert(defaults.doppler_north_noise_mps == 0.05F);
  assert(defaults.doppler_east_noise_mps == 0.05F);
  assert(defaults.doppler_down_noise_mps == 0.10F);

  for (const float speed_mps : { 0.0F, 172.0F })
  {
    GpsNoiseModel model(defaults, /*seed=*/32);
    const AxisErrors errors = doppler_errors(model, speed_mps, kSamples);
    assert_white_at(errors.north, 0.05);
    assert_white_at(errors.east, 0.05);
    assert_white_at(errors.down, 0.10);
  }

  // Per axis, not shared: a different sigma on each comes back on each.
  GpsNoiseConfig config;
  config.doppler_north_noise_mps = 0.02F;
  config.doppler_east_noise_mps = 0.08F;
  config.doppler_down_noise_mps = 0.3F;
  GpsNoiseModel model(config, /*seed=*/33);
  const AxisErrors errors = doppler_errors(model, 172.0F, kSamples);
  assert_white_at(errors.north, 0.02);
  assert_white_at(errors.east, 0.08);
  assert_white_at(errors.down, 0.3);
  // And independent: the sample correlation of north and east is zero to
  // about four standard errors (1 / sqrt(n) = 0.007).
  double cross = 0.0;
  for (std::size_t k = 0; k < errors.north.size(); ++k)
  {
    cross += errors.north[k] * errors.east[k];
  }
  cross /= static_cast<double>(errors.north.size()) * rms(errors.north) * rms(errors.east);
  assert(std::fabs(cross) < 0.03);
}

// sAcc under Doppler: sqrt((sN^2 + sE^2 + sD^2) / 3) times accuracy_scale,
// whatever the speed -- 0.0707 m/s at the defaults -- and honest on average:
// the measured per-axis RMS of the velocity error is that number.
void test_doppler_speed_accuracy()
{
  GpsNoiseModel model(GpsNoiseConfig{}, /*seed=*/34);
  const double expected = std::sqrt((0.05 * 0.05 + 0.05 * 0.05 + 0.10 * 0.10) / 3.0);  // 0.0707
  assert(near(model.velocity_sigma_mps(0.0F), expected, 1e-6));
  assert(near(model.velocity_sigma_mps(172.0F), expected, 1e-6));

  GpsTruthSample truth = truth_at(kEpochUs);
  assert(near(model.apply(truth).speed_accuracy_mps, expected, 1e-6));
  truth.ground_speed_mps = 0.0F;
  assert(near(model.apply(truth).speed_accuracy_mps, expected, 1e-6));

  GpsNoiseConfig optimistic;
  optimistic.accuracy_scale = 0.7F;
  GpsNoiseModel scaled(optimistic, /*seed=*/34);
  assert(near(scaled.apply(truth_at(kEpochUs)).speed_accuracy_mps, 0.7 * expected, 1e-6));

  GpsNoiseModel measured(GpsNoiseConfig{}, /*seed=*/35);
  const AxisErrors errors = doppler_errors(measured, 172.0F, kSamples);
  const double per_axis = std::sqrt((rms(errors.north) * rms(errors.north) + rms(errors.east) * rms(errors.east) +
                                     rms(errors.down) * rms(errors.down)) /
                                    3.0);
  assert(near(per_axis, expected, 0.02 * expected));
}

// Speed over ground is the magnitude of the noisy horizontal velocity at
// every speed. The course is its direction above ten horizontal sigmas
// (0.5 m/s at the defaults); below, the derived direction would be mostly
// the noise's, so the course is the speed/course model's -- truth plus the
// configured course error -- as it was before the Doppler model existed.
void test_doppler_course_at_low_speed()
{
  GpsNoiseModel model(GpsNoiseConfig{}, /*seed=*/36);
  double worst_course_error_deg = 0.0;
  bool any_derived_off_track = false;
  for (int k = 1; k <= 2000; ++k)
  {
    for (const float speed_mps : { 0.0F, 0.2F })
    {
      GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
      truth.ground_speed_mps = speed_mps;
      const GpsFix fix = model.apply(truth);
      assert(near(std::hypot(fix.vel_north_mps, fix.vel_east_mps), fix.ground_speed_mps, 1e-6));
      assert(fix.course_deg >= 0.0F && fix.course_deg < 360.0F);
      worst_course_error_deg = std::max(worst_course_error_deg, std::fabs(static_cast<double>(fix.course_deg) - 45.0));
      const double derived = std::atan2(fix.vel_east_mps, fix.vel_north_mps) * 180.0 / std::numbers::pi;
      any_derived_off_track = any_derived_off_track || std::fabs(derived - 45.0) > 30.0;
    }
  }
  // 1 deg of course noise: 4000 draws stay inside 6 sigma, while the noise's
  // own direction at these speeds goes everywhere.
  assert(worst_course_error_deg < 6.0);
  assert(any_derived_off_track);

  // Above the threshold the course is the derived one.
  GpsTruthSample truth = truth_at(kEpochUs);
  truth.ground_speed_mps = 1.0F;
  const GpsFix fix = model.apply(truth);
  const double derived = std::atan2(fix.vel_east_mps, fix.vel_north_mps) * 180.0 / std::numbers::pi;
  assert(near(fix.course_deg, std::fmod(derived + 360.0, 360.0), 1e-3));
}

// A zero Doppler sigma draws nothing: that axis is exactly the truth, with no
// zero-width distribution constructed (MSVC debug builds abort on one), and
// with every sigma zero the velocity is the truth and sAcc is zero.
void test_doppler_zero_sigma_is_exact()
{
  GpsNoiseConfig config;
  config.doppler_north_noise_mps = 0.0F;
  config.doppler_east_noise_mps = 0.0F;
  config.doppler_down_noise_mps = 0.0F;
  GpsNoiseModel model(config, /*seed=*/37);
  constexpr double kDeg = std::numbers::pi / 180.0;
  GpsTruthSample truth = truth_at(kEpochUs);
  truth.v_down_mps = 2.5F;
  const GpsFix fix = model.apply(truth);
  assert(near(fix.vel_north_mps, 172.0 * std::cos(45.0 * kDeg), 1e-4));
  assert(near(fix.vel_east_mps, 172.0 * std::sin(45.0 * kDeg), 1e-4));
  assert(fix.vel_down_mps == 2.5F);
  assert(near(fix.ground_speed_mps, 172.0, 1e-4));
  assert(near(fix.course_deg, 45.0, 1e-4));
  assert(fix.speed_accuracy_mps == 0.0F);
}

// Same seed, same Doppler fixes, bit for bit; another seed, other fixes.
void test_doppler_seed_reproduces_the_sequence()
{
  GpsNoiseModel a(GpsNoiseConfig{}, /*seed=*/42);
  GpsNoiseModel b(GpsNoiseConfig{}, /*seed=*/42);
  GpsNoiseModel c(GpsNoiseConfig{}, /*seed=*/43);
  bool differs = false;
  for (int k = 1; k <= 200; ++k)
  {
    const GpsTruthSample truth = truth_at(static_cast<std::uint64_t>(k) * kEpochUs);
    const GpsFix fa = a.apply(truth);
    const GpsFix fb = b.apply(truth);
    const GpsFix fc = c.apply(truth);
    assert(fa.vel_north_mps == fb.vel_north_mps);
    assert(fa.vel_east_mps == fb.vel_east_mps);
    assert(fa.vel_down_mps == fb.vel_down_mps);
    assert(fa.ground_speed_mps == fb.ground_speed_mps);
    assert(fa.course_deg == fb.course_deg);
    differs = differs || (fa.vel_north_mps != fc.vel_north_mps);
  }
  assert(differs);
}

// The FMU's integer parameter: 0 and 1 are the two models, anything else is
// refused and leaves the selection alone.
void test_velocity_model_codes()
{
  using hemerion::sensors::gps::fmu::velocity_error_model_from_code;
  GpsVelocityErrorModel model = GpsVelocityErrorModel::kSpeedCourse;
  assert(velocity_error_model_from_code(0, model) && model == GpsVelocityErrorModel::kDoppler);
  assert(velocity_error_model_from_code(1, model) && model == GpsVelocityErrorModel::kSpeedCourse);
  assert(!velocity_error_model_from_code(2, model) && model == GpsVelocityErrorModel::kSpeedCourse);
  assert(!velocity_error_model_from_code(-1, model));
}

}  // namespace

int main()
{
  test_seed_reproduces_the_sequence();
  test_vertical_velocity_moves_no_other_channel();
  test_velocity_agrees_with_speed_and_course();
  test_vertical_velocity_statistics();
  test_speed_accuracy_is_honest_on_average();
  test_defaults_are_white();
  test_correlated_term_is_gauss_markov();
  test_terms_add_in_quadrature();
  test_zero_correlation_time_is_white();
  test_reported_accuracy();
  test_state_hold_and_reset();
  test_speed_floor_and_course_wrap();
  test_speed_course_reproduces_the_previous_model();
  test_velocity_model_moves_no_position_draw();
  test_doppler_axes_are_white_and_speed_independent();
  test_doppler_speed_accuracy();
  test_doppler_course_at_low_speed();
  test_doppler_zero_sigma_is_exact();
  test_doppler_seed_reproduces_the_sequence();
  test_velocity_model_codes();

  std::puts("test_gps_noise: all checks passed");
  return 0;
}
