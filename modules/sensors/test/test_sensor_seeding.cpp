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
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

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

namespace
{

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
    assert(ra.accel_x == rb.accel_x && ra.accel_z == rb.accel_z && ra.gyro_y == rb.gyro_y);
    any_difference = any_difference || (ra.accel_x != rc.accel_x);
  }
  assert(any_difference);  // a different seed is a different part

  // The turn-on bias is drawn once: the mean of a long run sits off truth by
  // that draw and does not wander, while the samples themselves scatter by the
  // white sigma. Both are read off the same stream, in counts converted back
  // to SI by the configured scale.
  ImuNoiseConfig config;
  ImuNoiseModel model(config, /*seed=*/9);
  std::vector<double> gyro_x;
  gyro_x.reserve(kDraws);
  const double gyro_lsb_per_rad_s =
      static_cast<double>(config.scale.gyro_lsb_per_dps) * (180.0 / 3.14159265358979323846);
  for (int k = 0; k < kDraws; ++k)
  {
    gyro_x.push_back(static_cast<std::int16_t>(model.apply(truth).gyro_x) / gyro_lsb_per_rad_s);
  }
  // White sigma recovered (quantisation adds a little; 15 % is several
  // standard errors of a 4000-sample estimate either way).
  assert(near(stddev(gyro_x), config.gyro_noise_rad_s, 0.15 * config.gyro_noise_rad_s));
  // And the mean is the turn-on bias, not zero: within a few sigma of the
  // configured 1-sigma, and far outside the standard error of the mean.
  assert(std::fabs(mean(gyro_x)) < 4.0 * config.gyro_bias_sigma_rad_s);

  // A zero-sigma configuration draws nothing at all and is exactly truth.
  ImuNoiseConfig quiet;
  quiet.accel_noise_mps2 = 0.0F;
  quiet.gyro_noise_rad_s = 0.0F;
  quiet.accel_bias_sigma_mps2 = 0.0F;
  quiet.gyro_bias_sigma_rad_s = 0.0F;
  ImuNoiseModel noiseless(quiet, /*seed=*/1);
  const auto first = noiseless.apply(truth);
  const auto second = noiseless.apply(truth);
  assert(first.accel_x == second.accel_x && first.gyro_z == second.gyro_z);
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
    assert(ra.uncomp_press == rb.uncomp_press && ra.uncomp_temp == rb.uncomp_temp);
    any_difference = any_difference || (ra.uncomp_press != rc.uncomp_press);
  }
  assert(any_difference);

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
  assert(near(stddev(bias_pa), config.pressure_bias_sigma_pa, 0.2 * config.pressure_bias_sigma_pa));
  assert(std::fabs(mean(bias_pa)) < 0.3 * config.pressure_bias_sigma_pa);
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
    assert(ra.x == rb.x && ra.y == rb.y && ra.z == rb.z);
    any_difference = any_difference || (ra.x != rc.x);
  }
  assert(any_difference);

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
  assert(near(stddev(hard_iron_x), config.hard_iron_sigma_ut, 0.2 * config.hard_iron_sigma_ut));
  assert(std::fabs(mean(hard_iron_x)) < 0.3 * config.hard_iron_sigma_ut);
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
    assert(ra.range == rb.range && ra.status == rb.status);
    any_difference = any_difference || (ra.range != rc.range);
  }
  assert(any_difference);

  RadAltNoiseConfig config;
  RadAltNoiseModel model(config, /*seed=*/3);
  std::vector<double> range_m;
  range_m.reserve(kDraws);
  for (int k = 0; k < kDraws; ++k)
  {
    range_m.push_back(model.apply(truth).range / static_cast<double>(config.scale.range_lsb_per_m));
  }
  assert(near(stddev(range_m), config.range_noise_m, 0.15 * config.range_noise_m));
  assert(std::fabs(mean(range_m) - truth.height_agl_m) < 4.0 * config.range_bias_sigma_m);

  // Past the tracking range the part reports no ground, whatever the seed.
  truth.height_agl_m = config.max_range_m + 1000.0;
  assert(model.apply(truth).status == hemerion::sensors::radalt::kRadAltStatusNoReturn);
}

}  // namespace

int main()
{
  test_imu_seeding();
  test_bmp390_seeding();
  test_mmc5983ma_seeding();
  test_radalt_seeding();

  std::puts("test_sensor_seeding: all checks passed");
  return 0;
}
