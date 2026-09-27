// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_magnetic_calibration.cpp
//
// The ellipsoid fit that recovers hard and soft iron
// (Hemerion/mag/magnetic_calibration.h).
//
// The loop closes against the *simulated part*, not against a re-derivation of
// the distortion in this file. Samples go through Mmc5983maMeasurementModel --
// its own drawn hard iron, its own drawn soft iron, its noise, its 18-bit
// quantiser -- come back as register counts, get decoded the way the driver
// decodes them, and only then reach the calibration. That is the same trick
// the BMP390 stack uses: a test that inverts its own arithmetic proves the
// arithmetic is self-consistent and nothing else, whereas this one fails if
// the two models ever disagree about what the distortion *is*.
//
// Plain asserts + exit code, matching the other sensors tests.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <numbers>

#include "Hemerion/mag/magnetic_calibration.h"
#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_registers.h"

using hemerion::sensors::mag::MagCalibrationError;
using hemerion::sensors::mag::MagCorrection;
using hemerion::sensors::mag::MagneticCalibration;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maLsbPerMicrotesla;
using hemerion::sensors::mag::mmc5983ma::kMmc5983maNullFieldOutput;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maSensingState;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementConfig;
using hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maMeasurementModel;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG, and these files are compiled (though
// not run) by the Release SWIL job. See test_sensor_seeding.cpp.
[[noreturn]] void check_failed(const char* expression, const char* file, int line)
{
  std::cerr << file << ":" << line << ": check failed: " << expression << "\n";
  std::abort();
}

void check(bool condition, const char* expression, const char* file, int line)
{
  if (!condition)
  {
    check_failed(expression, file, line);
  }
}

bool near(double value, double expected, double tolerance) { return std::fabs(value - expected) <= tolerance; }

constexpr double kFieldUt = 48.0;  // a mid-latitude total intensity

/// A truth field of constant magnitude, pointing in a direction that sweeps
/// the whole sphere as `index` runs over `count`.
///
/// A Fibonacci lattice rather than nested Euler loops: it covers the sphere
/// evenly with no clustering at the poles, which is the coverage an ellipsoid
/// fit needs and the thing a naive lat/lon grid is worst at.
std::array<double, 3> truth_field(std::size_t index, std::size_t count)
{
  const double golden = std::numbers::pi * (3.0 - std::sqrt(5.0));
  const double z = 1.0 - (2.0 * (static_cast<double>(index) + 0.5) / static_cast<double>(count));
  const double radius = std::sqrt(std::max(0.0, 1.0 - (z * z)));
  const double theta = golden * static_cast<double>(index);
  return { kFieldUt * radius * std::cos(theta), kFieldUt * radius * std::sin(theta), kFieldUt * z };
}

/// One measurement through the simulated part and back out as microtesla,
/// exactly as the driver would recover it.
std::array<double, 3> through_the_part(Mmc5983maMeasurementModel& model,
                                       const std::array<double, 3>& truth,
                                       const Mmc5983maSensingState& sensing)
{
  const auto counts = model.measure(truth[0], truth[1], truth[2], sensing);
  const auto to_ut = [](std::uint32_t count) {
    return (static_cast<double>(count) - static_cast<double>(kMmc5983maNullFieldOutput)) /
           static_cast<double>(kMmc5983maLsbPerMicrotesla);
  };
  return { to_ut(counts.x), to_ut(counts.y), to_ut(counts.z) };
}

/// Fills a calibration from `count` attitudes through the part.
void sweep(MagneticCalibration& calibration,
           Mmc5983maMeasurementModel& model,
           const Mmc5983maSensingState& sensing,
           std::size_t count)
{
  for (std::size_t k = 0; k < count; ++k)
  {
    const auto measured = through_the_part(model, truth_field(k, count), sensing);
    calibration.add_sample(measured[0], measured[1], measured[2]);
  }
}

// --------------------------------------------------------------------------

/// With no distortion configured, the calibration must find none.
///
/// This is the case that catches a fit which "works" by absorbing the truth
/// into its own parameters: a clean part must come back as no offset and the
/// identity, not as a small correction that happens to fit.
void test_clean_part_needs_no_correction()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 0.0F;
  config.soft_iron_sigma = 0.0F;

  Mmc5983maSensingState sensing;
  sensing.automatic_set_reset = true;  // bridge offset already differenced out
  Mmc5983maMeasurementModel model(config, /*seed=*/3);

  MagneticCalibration calibration;
  sweep(calibration, model, sensing, 400);

  MagCorrection correction;
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kNone);

  for (std::size_t i = 0; i < 3; ++i)
  {
    CHECK(near(correction.hard_iron_ut[i], 0.0, 0.05));
    for (std::size_t j = 0; j < 3; ++j)
    {
      CHECK(near(correction.soft_iron_inverse[i][j], (i == j) ? 1.0 : 0.0, 0.02));
    }
  }
}

/// The real case: recover what the part actually drew.
///
/// The measurement model applies `m = S B + h`, so a correct calibration
/// returns `h` and a `W` that undoes `S`. Both are checked -- the offset
/// against the model's own `hard_iron_ut()`, and the matrix by the only test
/// that matters operationally, which is whether a corrected sample lands back
/// on the truth field it came from.
void test_recovers_the_parts_own_distortion()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;  // geometry first; noise has its own test below
  config.hard_iron_sigma_ut = 2.5F;
  config.soft_iron_sigma = 0.04F;

  Mmc5983maSensingState sensing;
  sensing.automatic_set_reset = true;
  Mmc5983maMeasurementModel model(config, /*seed=*/11);

  // The distortion is real, or the test proves nothing.
  const float* drawn = model.hard_iron_ut();
  const double offset_magnitude = std::sqrt((drawn[0] * drawn[0]) + (drawn[1] * drawn[1]) + (drawn[2] * drawn[2]));
  CHECK(offset_magnitude > 0.5);
  const auto& soft = model.soft_iron();
  CHECK(std::fabs(soft[0][0] - 1.0F) > 1e-4F || std::fabs(soft[0][1]) > 1e-4F);

  MagneticCalibration calibration;
  sweep(calibration, model, sensing, 600);

  MagCorrection correction;
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kNone);

  // Hard iron, against what the part drew.
  for (std::size_t i = 0; i < 3; ++i)
  {
    CHECK(near(correction.hard_iron_ut[i], static_cast<double>(drawn[i]), 0.05));
  }

  // And the whole correction, against truth: every corrected sample must land
  // back on the field that produced it. The measurement model's soft iron is
  // symmetric, which is what makes this recoverable at all -- see the header
  // on the rotation an ellipsoid fit cannot see.
  double worst_ut = 0.0;
  for (std::size_t k = 0; k < 600; ++k)
  {
    const auto truth = truth_field(k, 600);
    const auto measured = through_the_part(model, truth, sensing);
    const auto corrected = correction.apply(measured[0], measured[1], measured[2]);
    for (std::size_t i = 0; i < 3; ++i)
    {
      worst_ut = std::max(worst_ut, std::fabs(corrected[i] - truth[i]));
    }
  }
  // One quantiser step is 1/163.84 uT; this is a few of them.
  CHECK(worst_ut < 0.05);
}

/// Noise must not bias the answer, only loosen it.
///
/// A least-squares fit over enough samples averages zero-mean noise down; if
/// this ever fails by a *lot* rather than a little, the fit has a bias rather
/// than a variance problem.
void test_noise_widens_but_does_not_bias()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.4F;
  config.hard_iron_sigma_ut = 2.5F;
  config.soft_iron_sigma = 0.04F;

  Mmc5983maSensingState sensing;
  sensing.automatic_set_reset = true;
  Mmc5983maMeasurementModel model(config, /*seed=*/11);

  MagneticCalibration calibration;
  sweep(calibration, model, sensing, 4000);

  MagCorrection correction;
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kNone);

  const float* drawn = model.hard_iron_ut();
  for (std::size_t i = 0; i < 3; ++i)
  {
    CHECK(near(correction.hard_iron_ut[i], static_cast<double>(drawn[i]), 0.35));
  }
}

/// Coverage the fit cannot work with has to be refused, not guessed at.
///
/// Spinning about one axis traces a circle, and infinitely many ellipsoids
/// contain a given circle. A calibration that returned *something* here would
/// be the worst failure mode available: a plausible-looking correction with no
/// support in the data.
void test_insufficient_attitude_coverage_is_refused()
{
  MagneticCalibration circle;
  for (int k = 0; k < 720; ++k)
  {
    const double angle = static_cast<double>(k) * std::numbers::pi / 360.0;
    circle.add_sample(kFieldUt * std::cos(angle), kFieldUt * std::sin(angle), 0.0);
  }
  MagCorrection correction;
  const MagCalibrationError error = circle.solve(kFieldUt, correction);
  CHECK(error == MagCalibrationError::kDegenerate || error == MagCalibrationError::kNotEllipsoid);

  // The correction is left untouched on failure, so a caller that ignores the
  // return value gets the identity rather than rubbish.
  CHECK(correction.hard_iron_ut[0] == 0.0);
  CHECK(correction.soft_iron_inverse[0][0] == 1.0);
}

/// The other refusals, and that a reset really resets.
void test_guards()
{
  MagneticCalibration calibration;
  MagCorrection correction;
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kInsufficientData);

  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  Mmc5983maSensingState sensing;
  sensing.automatic_set_reset = true;
  Mmc5983maMeasurementModel model(config, /*seed=*/7);

  sweep(calibration, model, sensing, 200);
  CHECK(calibration.sample_count() == 200);
  CHECK(calibration.solve(0.0, correction) == MagCalibrationError::kInvalidField);
  CHECK(calibration.solve(-1.0, correction) == MagCalibrationError::kInvalidField);
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kNone);

  calibration.reset();
  CHECK(calibration.sample_count() == 0);
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kInsufficientData);
}

/// The heading error an uncalibrated installation actually costs, and that
/// calibrating removes it.
///
/// This is the number the whole exercise is for: the F-16 page reports a
/// 5.33 degree magnetic heading bias from hard iron alone, and until now
/// nothing in the tree could take it out.
void test_heading_error_is_removed()
{
  Mmc5983maMeasurementConfig config;
  config.noise_ut = 0.0F;
  config.hard_iron_sigma_ut = 2.5F;
  config.soft_iron_sigma = 0.04F;

  Mmc5983maSensingState sensing;
  sensing.automatic_set_reset = true;
  Mmc5983maMeasurementModel model(config, /*seed=*/11);

  MagneticCalibration calibration;
  sweep(calibration, model, sensing, 600);
  MagCorrection correction;
  CHECK(calibration.solve(kFieldUt, correction) == MagCalibrationError::kNone);

  // A level aircraft turning through 360 degrees under a field with a
  // northward and a downward component: heading comes from the horizontal
  // pair, which is exactly what hard iron corrupts.
  const double north_ut = 20.0;
  const double down_ut = std::sqrt((kFieldUt * kFieldUt) - (north_ut * north_ut));
  double worst_raw_deg = 0.0;
  double worst_corrected_deg = 0.0;
  for (int k = 0; k < 360; ++k)
  {
    const double heading = static_cast<double>(k) * std::numbers::pi / 180.0;
    const std::array<double, 3> truth = { north_ut * std::cos(heading), -north_ut * std::sin(heading), down_ut };
    const auto measured = through_the_part(model, truth, sensing);
    const auto corrected = correction.apply(measured[0], measured[1], measured[2]);

    const auto heading_of = [](double x, double y) { return std::atan2(-y, x); };
    const auto wrap_deg = [](double radians) {
      double degrees = radians * 180.0 / std::numbers::pi;
      while (degrees > 180.0)
      {
        degrees -= 360.0;
      }
      while (degrees < -180.0)
      {
        degrees += 360.0;
      }
      return degrees;
    };
    worst_raw_deg = std::max(worst_raw_deg, std::fabs(wrap_deg(heading_of(measured[0], measured[1]) - heading)));
    worst_corrected_deg =
        std::max(worst_corrected_deg, std::fabs(wrap_deg(heading_of(corrected[0], corrected[1]) - heading)));
  }

  // The uncalibrated error is degrees; the calibrated one is quantiser noise.
  CHECK(worst_raw_deg > 2.0);
  CHECK(worst_corrected_deg < 0.2);
  CHECK(worst_corrected_deg < worst_raw_deg / 10.0);
}

}  // namespace

int main()
{
  test_clean_part_needs_no_correction();
  test_recovers_the_parts_own_distortion();
  test_noise_widens_but_does_not_bias();
  test_insufficient_attitude_coverage_is_refused();
  test_guards();
  test_heading_error_is_removed();

  std::puts("test_magnetic_calibration: all checks passed");
  return 0;
}
