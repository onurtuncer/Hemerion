// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_sensor_seeding.cpp
//
// One property, asserted for every sensor error model: seed + config
// determine the output exactly. Same seed, same numbers; different seed,
// different numbers; and a turn-on bias that is drawn once stays put while a
// white term keeps moving.
//
// This is what makes a run reproducible, and it is worth its own file because
// it is the property the FMUs' `seed` parameters sell. Before those existed
// every run of these four parts drew a fresh instance from std::random_device,
// so two runs of the same scenario differed by the whole turn-on bias -- 3.4 m
// of indicated altitude on the BMP390 -- and no figure could separate a
// systematic effect from the draw.
//
// Statistics, not shapes: where a sigma is asserted it is against a sample
// over enough draws that the tolerance is several standard errors, and every
// model here is seeded, so a failure is a change in the model rather than a
// bad draw.
//
// Plain asserts + exit code, matching test_gps_noise.cpp.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <random>
#include <utility>
#include <vector>

#include "Hemerion/sensor_clock.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h"
#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "Hemerion/radalt/fmu/radalt_noise_model.h"
#include "Hemerion/radalt/radalt_types.h"

using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementConfig;
using hemerion::sensors::baro::bmp390::fmu::Bmp390MeasurementModel;
using hemerion::sensors::imu::fmu::ImuNoiseConfig;
using hemerion::sensors::imu::fmu::ImuNoiseModel;
using hemerion::sensors::imu::fmu::ImuTruthSample;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maSensingState;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementConfig;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;
using hemerion::sensors::radalt::fmu::RadAltNoiseConfig;
using hemerion::sensors::radalt::fmu::RadAltNoiseModel;
using hemerion::sensors::radalt::fmu::RadAltTruthSample;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG, and these files are compiled (though
// not run) by the Release SWIL job -- where every check below would vanish,
// taking the variables feeding it with it. A test that silently asserts
// nothing is exactly the vacuous green this repo has been bitten by before, so
// the checks here survive the optimiser rather than depending on the build
// type.
[[noreturn]] void check_failed(const char* expression, const char* file, int line)
{
  std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
  std::abort();
}

void check(bool condition, const char* expression, const char* file, int line)
{
  if (!condition)
  {
    check_failed(expression, file, line);
  }
}

constexpr int kDraws = 4000;

bool near(double value, double expected, double tolerance) { return std::fabs(value - expected) <= tolerance; }

double mean(const std::vector<double>& x)
{
  double sum = 0.0;
  for (const double v : x)
  {
    sum += v;
  }
  return sum / static_cast<double>(x.size());
}

double stddev(const std::vector<double>& x)
{
  const double m = mean(x);
  double sum = 0.0;
  for (const double v : x)
  {
    sum += (v - m) * (v - m);
  }
  return std::sqrt(sum / static_cast<double>(x.size()));
}

ImuTruthSample imu_truth()
{
  ImuTruthSample truth;
  truth.specific_force_x_mps2 = 0.46;  // check-case 11's trim, roughly
  truth.specific_force_y_mps2 = 0.0;
  truth.specific_force_z_mps2 = -9.79;
  truth.angular_rate_x_rad_s = 0.0;
  truth.angular_rate_y_rad_s = 0.0;
  truth.angular_rate_z_rad_s = 0.0;
  return truth;
}

// --- IMU ---------------------------------------------------------------------

void test_imu_seeding()
{
  const ImuTruthSample truth = imu_truth();

  ImuNoiseModel a(ImuNoiseConfig{}, /*seed=*/1234);
  ImuNoiseModel b(ImuNoiseConfig{}, /*seed=*/1234);
  ImuNoiseModel c(ImuNoiseConfig{}, /*seed=*/1235);
  bool any_difference = false;
  for (int k = 0; k < 200; ++k)
  {
    const auto ra = a.apply(truth);
    const auto rb = b.apply(truth);
    const auto rc = c.apply(truth);
    CHECK(ra.accel_x == rb.accel_x && ra.accel_z == rb.accel_z && ra.gyro_y == rb.gyro_y);
    any_difference = any_difference || (ra.accel_x != rc.accel_x);
  }
  CHECK(any_difference);  // a different seed is a different part

  // The turn-on bias is drawn once: the mean of a long run sits off truth by
  // that draw and does not wander, while the samples themselves scatter by the
  // white sigma. Both are read off the same stream, in counts converted back
  // to SI by the configured scale.
  ImuNoiseConfig config;
  ImuNoiseModel model(config, /*seed=*/9);
  std::vector<double> gyro_x;
  gyro_x.reserve(kDraws);
  const double gyro_lsb_per_rad_s = static_cast<double>(config.scale.gyro_lsb_per_dps) * (180.0 / std::numbers::pi);
  for (int k = 0; k < kDraws; ++k)
  {
    gyro_x.push_back(static_cast<std::int16_t>(model.apply(truth).gyro_x) / gyro_lsb_per_rad_s);
  }
  // White sigma recovered (quantisation adds a little; 15 % is several
  // standard errors of a 4000-sample estimate either way).
  CHECK(near(stddev(gyro_x), config.gyro_noise_rad_s, 0.15 * config.gyro_noise_rad_s));
  // And the mean is the turn-on bias, not zero: within a few sigma of the
  // configured 1-sigma, and far outside the standard error of the mean.
  CHECK(std::fabs(mean(gyro_x)) < 4.0 * config.gyro_bias_sigma_rad_s);

  // A zero-sigma configuration draws nothing at all and is exactly truth.
  ImuNoiseConfig quiet;
  quiet.accel_noise_mps2 = 0.0F;
  quiet.gyro_noise_rad_s = 0.0F;
  quiet.accel_bias_sigma_mps2 = 0.0F;
  quiet.gyro_bias_sigma_rad_s = 0.0F;
  ImuNoiseModel noiseless(quiet, /*seed=*/1);
  const auto first = noiseless.apply(truth);
  const auto second = noiseless.apply(truth);
  CHECK(first.accel_x == second.accel_x && first.gyro_z == second.gyro_z);
}

// Bias random walk: the bias must wander, its spread must grow as sqrt(t), and
// -- the property that matters -- the drift after a given elapsed time must not
// depend on how often the model was sampled. A walk implemented as a per-sample
// step rather than a per-second one would double its drift when the rate
// doubles, which is the classic way to get this wrong.
void test_imu_bias_random_walk()
{
  const ImuTruthSample truth = imu_truth();
  constexpr double kRate = 0.01;  // rad/s per sqrt(s), deliberately large so it is visible
  constexpr double kSeconds = 100.0;

  const auto drift_after = [&](double dt_s, int seed) {
    ImuNoiseConfig config;
    config.gyro_noise_rad_s = 0.0F;       // isolate the walk
    config.gyro_bias_sigma_rad_s = 0.0F;  // start from zero bias
    config.gyro_bias_walk_rad_s_sqrt_s = static_cast<float>(kRate);
    ImuNoiseModel model(config, static_cast<std::uint64_t>(seed));
    ImuTruthSample sample = truth;
    const auto steps = static_cast<int>(kSeconds / dt_s);
    for (int k = 1; k <= steps; ++k)
    {
      sample.timestamp_us = static_cast<std::uint64_t>(k * dt_s * 1e6);
      (void)model.apply(sample);
    }
    return static_cast<double>(model.gyro_bias_rad_s()[0]);
  };

  // Spread over many seeds, at two sample rates a decade apart.
  const auto spread = [&](double dt_s) {
    std::vector<double> drift;
    drift.reserve(300);
    for (int seed = 1; seed <= 300; ++seed)
    {
      drift.push_back(drift_after(dt_s, seed));
    }
    return stddev(drift);
  };

  const double expected = kRate * std::sqrt(kSeconds);  // 0.1 rad/s
  const double coarse = spread(0.1);
  const double fine = spread(0.01);
  CHECK(near(coarse, expected, 0.15 * expected));
  CHECK(near(fine, expected, 0.15 * expected));
  // Rate independence, the point of the sqrt(dt) increment.
  CHECK(near(coarse / fine, 1.0, 0.2));

  // And with the walk off the bias does not move at all.
  ImuNoiseConfig still;
  still.gyro_bias_walk_rad_s_sqrt_s = 0.0F;
  ImuNoiseModel fixed_bias(still, /*seed=*/3);
  ImuTruthSample sample = truth;
  const double before = static_cast<double>(fixed_bias.gyro_bias_rad_s()[0]);
  for (int k = 1; k <= 500; ++k)
  {
    sample.timestamp_us = static_cast<std::uint64_t>(k * 10000);
    (void)fixed_bias.apply(sample);
  }
  CHECK(static_cast<double>(fixed_bias.gyro_bias_rad_s()[0]) == before);
}

// Scale factor and misalignment are cross-axis errors invisible at rest: they
// scale with the signal. A part with 1 % scale error reads 1 % high on a rate
// it is actually turning at, and nothing extra on a rate of zero.
void test_imu_scale_and_misalignment()
{
  const auto sensed_gyro_x = [](const ImuNoiseConfig& config, double rate_x, double rate_y) {
    ImuNoiseModel model(config, /*seed=*/11);
    ImuTruthSample truth = imu_truth();
    truth.angular_rate_x_rad_s = rate_x;
    truth.angular_rate_y_rad_s = rate_y;
    truth.timestamp_us = 10000;
    const double lsb_per_rad_s = static_cast<double>(config.scale.gyro_lsb_per_dps) * (180.0 / std::numbers::pi);
    return static_cast<std::int16_t>(model.apply(truth).gyro_x) / lsb_per_rad_s;
  };

  ImuNoiseConfig quiet;  // no noise or bias, so the geometry is all that is left
  quiet.accel_noise_mps2 = 0.0F;
  quiet.gyro_noise_rad_s = 0.0F;
  quiet.accel_bias_sigma_mps2 = 0.0F;
  quiet.gyro_bias_sigma_rad_s = 0.0F;

  // Perfect part: what goes in comes out, to the quantiser.
  const double one_count = 1.0 / (static_cast<double>(quiet.scale.gyro_lsb_per_dps) * (180.0 / std::numbers::pi));
  CHECK(near(sensed_gyro_x(quiet, 1.0, 0.0), 1.0, one_count));
  CHECK(near(sensed_gyro_x(quiet, 0.0, 1.0), 0.0, one_count));

  // Scale error: proportional to the rate, and exactly zero at zero rate.
  ImuNoiseConfig scaled = quiet;
  scaled.gyro_scale_sigma = 0.05F;  // large, so one draw is unambiguous
  const double at_one = sensed_gyro_x(scaled, 1.0, 0.0);
  const double at_two = sensed_gyro_x(scaled, 2.0, 0.0);
  CHECK(std::fabs(at_one - 1.0) > 3.0 * one_count);  // the error is there
  CHECK(near(at_two / at_one, 2.0, 0.01));           // and it is proportional
  CHECK(near(sensed_gyro_x(scaled, 0.0, 0.0), 0.0, one_count));

  // Misalignment: X picks up a little of Y, and still reads nothing when the
  // vehicle is not turning at all.
  ImuNoiseConfig skewed = quiet;
  skewed.misalignment_sigma_rad = 0.02F;
  CHECK(std::fabs(sensed_gyro_x(skewed, 0.0, 1.0)) > 3.0 * one_count);
  CHECK(near(sensed_gyro_x(skewed, 0.0, 0.0), 0.0, one_count));

  // Both default to off, and then the triad is exactly the identity: the
  // default model must be bit-identical to the one before these existed.
  CHECK(sensed_gyro_x(quiet, 0.0, 1.0) == 0.0);
}

// --- BMP390 ------------------------------------------------------------------

void test_bmp390_seeding()
{
  Bmp390MeasurementModel a(Bmp390MeasurementConfig{}, /*seed=*/77);
  Bmp390MeasurementModel b(Bmp390MeasurementConfig{}, /*seed=*/77);
  Bmp390MeasurementModel c(Bmp390MeasurementConfig{}, /*seed=*/78);
  bool any_difference = false;
  for (int k = 0; k < 200; ++k)
  {
    const auto ra = a.measure(3052.0);
    const auto rb = b.measure(3052.0);
    const auto rc = c.measure(3052.0);
    CHECK(ra.uncomp_press == rb.uncomp_press && ra.uncomp_temp == rb.uncomp_temp);
    any_difference = any_difference || (ra.uncomp_press != rc.uncomp_press);
  }
  CHECK(any_difference);

  // The turn-on pressure bias is what makes two unseeded runs of the same
  // scenario differ by metres of indicated altitude: across seeds its spread
  // is the configured sigma. One draw per model, so one model per sample.
  Bmp390MeasurementConfig config;
  std::vector<double> bias_pa;
  bias_pa.reserve(600);
  for (int seed = 1; seed <= 600; ++seed)
  {
    Bmp390MeasurementModel model(config, static_cast<std::uint64_t>(seed));
    const auto conversion = model.measure_ambient(70000.0, -5.0);
    bias_pa.push_back(model.compensator().compensate_pressure(
                          conversion.uncomp_press, model.compensator().compensate_temperature(conversion.uncomp_temp)) -
                      70000.0);
  }
  CHECK(near(stddev(bias_pa), config.pressure_bias_sigma_pa, 0.2 * config.pressure_bias_sigma_pa));
  CHECK(std::fabs(mean(bias_pa)) < 0.3 * config.pressure_bias_sigma_pa);
}

// --- MMC5983MA ---------------------------------------------------------------

void test_mmc5983ma_seeding()
{
  Mmc5983maSensingState sensing;
  Mmc5983maMeasurementModel a(Mmc5983maMeasurementConfig{}, /*seed=*/5);
  Mmc5983maMeasurementModel b(Mmc5983maMeasurementConfig{}, /*seed=*/5);
  Mmc5983maMeasurementModel c(Mmc5983maMeasurementConfig{}, /*seed=*/6);
  bool any_difference = false;
  for (int k = 0; k < 200; ++k)
  {
    const auto ra = a.measure(13.6, -15.4, 45.0, sensing);
    const auto rb = b.measure(13.6, -15.4, 45.0, sensing);
    const auto rc = c.measure(13.6, -15.4, 45.0, sensing);
    CHECK(ra.x == rb.x && ra.y == rb.y && ra.z == rb.z);
    any_difference = any_difference || (ra.x != rc.x);
  }
  CHECK(any_difference);

  // Hard iron is drawn once per run and survives SET/RESET by construction --
  // it is a real field, not an electrical null error. Its spread across seeds
  // is the configured sigma, and it is the 5.3 deg of heading bias the F-16
  // page's magnetometer figure measures.
  Mmc5983maMeasurementConfig config;
  std::vector<double> hard_iron_x;
  hard_iron_x.reserve(600);
  for (int seed = 1; seed <= 600; ++seed)
  {
    Mmc5983maMeasurementModel model(config, static_cast<std::uint64_t>(seed));
    hard_iron_x.push_back(model.hard_iron_ut()[0]);
  }
  CHECK(near(stddev(hard_iron_x), config.hard_iron_sigma_ut, 0.2 * config.hard_iron_sigma_ut));
  CHECK(std::fabs(mean(hard_iron_x)) < 0.3 * config.hard_iron_sigma_ut);
}

// --- Radar altimeter ---------------------------------------------------------

void test_radalt_seeding()
{
  RadAltTruthSample truth;
  truth.height_agl_m = 3052.0;

  RadAltNoiseModel a(RadAltNoiseConfig{}, /*seed=*/31);
  RadAltNoiseModel b(RadAltNoiseConfig{}, /*seed=*/31);
  RadAltNoiseModel c(RadAltNoiseConfig{}, /*seed=*/32);
  bool any_difference = false;
  for (int k = 0; k < 200; ++k)
  {
    const auto ra = a.apply(truth);
    const auto rb = b.apply(truth);
    const auto rc = c.apply(truth);
    CHECK(ra.range == rb.range && ra.status == rb.status);
    any_difference = any_difference || (ra.range != rc.range);
  }
  CHECK(any_difference);

  RadAltNoiseConfig config;
  RadAltNoiseModel model(config, /*seed=*/3);
  std::vector<double> range_m;
  range_m.reserve(kDraws);
  for (int k = 0; k < kDraws; ++k)
  {
    range_m.push_back(model.apply(truth).range / static_cast<double>(config.scale.range_lsb_per_m));
  }
  CHECK(near(stddev(range_m), config.range_noise_m, 0.15 * config.range_noise_m));
  CHECK(std::fabs(mean(range_m) - truth.height_agl_m) < 4.0 * config.range_bias_sigma_m);

  // Past the tracking range the part reports no ground, whatever the seed.
  truth.height_agl_m = config.max_range_m + 1000.0;
  CHECK(model.apply(truth).status == hemerion::sensors::radalt::kRadAltStatusNoReturn);
}

// The beam geometry: a radar altimeter measures along an axis fixed to the
// airframe, so over flat ground it reads h / (cos roll cos pitch) -- always
// longer than the height, never shorter -- and past the beam's half-angle it
// has no ground in its footprint at all.
void test_radalt_beam_geometry()
{
  RadAltNoiseConfig config;
  config.range_noise_m = 0.0F;  // isolate the geometry
  config.range_bias_sigma_m = 0.0F;
  config.beam_half_angle_rad = static_cast<float>(20.0 * std::numbers::pi / 180.0);

  const auto range_at = [&config](double roll_deg, double pitch_deg) {
    RadAltNoiseModel model(config, /*seed=*/1);
    RadAltTruthSample truth;
    truth.height_agl_m = 1000.0;
    truth.roll_rad = roll_deg * std::numbers::pi / 180.0;
    truth.pitch_rad = pitch_deg * std::numbers::pi / 180.0;
    const auto raw = model.apply(truth);
    return std::pair<double, bool>{ raw.range / static_cast<double>(config.scale.range_lsb_per_m),
                                    raw.status == hemerion::sensors::radalt::kRadAltStatusTrackValid };
  };

  // Level: the slant range is the height.
  const auto level = range_at(0.0, 0.0);
  CHECK(level.second);
  CHECK(near(level.first, 1000.0, 0.02));

  // Banked 15 degrees: 1000 / cos(15 deg) = 1035.3 m. Longer, and by the
  // amount the geometry says rather than by an arbitrary fudge.
  const auto banked = range_at(15.0, 0.0);
  CHECK(banked.second);
  CHECK(near(banked.first, 1000.0 / std::cos(15.0 * std::numbers::pi / 180.0), 0.02));
  CHECK(banked.first > level.first);

  // Pitch counts the same way, and the two combine through the product of
  // their cosines rather than by adding.
  const auto pitched = range_at(0.0, 15.0);
  CHECK(near(pitched.first, banked.first, 0.02));
  const auto both = range_at(10.0, 10.0);
  const double cos_tilt = std::cos(10.0 * std::numbers::pi / 180.0) * std::cos(10.0 * std::numbers::pi / 180.0);
  CHECK(near(both.first, 1000.0 / cos_tilt, 0.02));

  // Past the beam: no ground in the footprint, reported exactly as an
  // out-of-range return is, because the firmware cannot tell them apart.
  CHECK(!range_at(25.0, 0.0).second);
  CHECK(!range_at(0.0, -25.0).second);  // nose-up counts too
  CHECK(range_at(19.0, 0.0).second);    // and it is the half-angle, not a rounding of it

  // With the beam disabled the part ignores attitude entirely, which is what
  // it did before this existed.
  RadAltNoiseConfig flat = config;
  flat.beam_half_angle_rad = 0.0F;
  RadAltNoiseModel blind(flat, /*seed=*/1);
  RadAltTruthSample tilted;
  tilted.height_agl_m = 1000.0;
  tilted.roll_rad = 60.0 * std::numbers::pi / 180.0;
  const auto ignored = blind.apply(tilted);
  CHECK(ignored.status == hemerion::sensors::radalt::kRadAltStatusTrackValid);
  CHECK(near(ignored.range / static_cast<double>(flat.scale.range_lsb_per_m), 1000.0, 0.02));
}

// A part's own clock. Skew is a rate error and accumulates; jitter is a
// per-sample error and does not. Telling them apart matters to a filter: one
// slides two sensors' streams against each other over a flight and has to be
// estimated or tolerated, the other just blurs a timestamp.
void test_sensor_clock_skew_and_jitter()
{
  using hemerion::sensors::SensorClock;
  using hemerion::sensors::SensorClockConfig;

  // A perfect clock reports the instant it was given, and draws nothing.
  std::mt19937_64 rng(1);
  SensorClock perfect(SensorClockConfig{}, rng);
  CHECK(perfect.skew_ppm() == 0.0F);
  CHECK(perfect.stamp(1.0, rng) == 1000000U);
  CHECK(perfect.stamp(123.456789, rng) == 123456789U);

  // Skew is a rate: the error grows with elapsed time, linearly, and a given
  // instance's is constant.
  std::mt19937_64 skew_rng(7);
  SensorClock skewed(SensorClockConfig{ 50.0F, 0.0F }, skew_rng);
  const double ppm = static_cast<double>(skewed.skew_ppm());
  CHECK(ppm != 0.0);
  for (const double seconds : { 1.0, 10.0, 1000.0 })
  {
    const double reported = static_cast<double>(skewed.stamp(seconds, skew_rng)) * 1e-6;
    CHECK(near(reported - seconds, seconds * ppm * 1e-6, 1e-6 + 1e-9 * seconds));
  }

  // Across instances its spread is the configured sigma.
  std::vector<double> draws;
  draws.reserve(500);
  for (int seed = 1; seed <= 500; ++seed)
  {
    std::mt19937_64 one(static_cast<std::uint64_t>(seed));
    draws.push_back(static_cast<double>(SensorClock(SensorClockConfig{ 50.0F, 0.0F }, one).skew_ppm()));
  }
  CHECK(near(stddev(draws), 50.0, 8.0));
  CHECK(std::fabs(mean(draws)) < 12.0);

  // Jitter has no memory: its spread is the configured sigma and does not
  // grow with elapsed time, which is exactly how it differs from skew.
  std::mt19937_64 jitter_rng(9);
  SensorClock jittery(SensorClockConfig{ 0.0F, 1e-3F }, jitter_rng);
  const auto jitter_spread = [&](double seconds) {
    std::vector<double> error;
    error.reserve(2000);
    for (int k = 0; k < 2000; ++k)
    {
      error.push_back(static_cast<double>(jittery.stamp(seconds, jitter_rng)) * 1e-6 - seconds);
    }
    return stddev(error);
  };
  const double early = jitter_spread(1.0);
  const double late = jitter_spread(1000.0);
  CHECK(near(early, 1e-3, 1.5e-4));
  CHECK(near(late, 1e-3, 1.5e-4));
  CHECK(near(late / early, 1.0, 0.25));
}

}  // namespace

int main()
{
  test_imu_seeding();
  test_imu_bias_random_walk();
  test_imu_scale_and_misalignment();
  test_bmp390_seeding();
  test_mmc5983ma_seeding();
  test_radalt_seeding();
  test_radalt_beam_geometry();
  test_sensor_clock_skew_and_jitter();

  std::puts("test_sensor_seeding: all checks passed");
  return 0;
}
