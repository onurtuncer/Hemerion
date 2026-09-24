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
// accuracy, and reproducibility from a seed.
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
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::fmu::GpsNoiseConfig;
using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;

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

}  // namespace

int main()
{
  test_seed_reproduces_the_sequence();
  test_defaults_are_white();
  test_correlated_term_is_gauss_markov();
  test_terms_add_in_quadrature();
  test_zero_correlation_time_is_white();
  test_reported_accuracy();
  test_state_hold_and_reset();
  test_speed_floor_and_course_wrap();

  std::puts("test_gps_noise: all checks passed");
  return 0;
}
