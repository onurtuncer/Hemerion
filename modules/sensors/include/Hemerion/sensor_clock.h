// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file sensor_clock.h
/// @brief The clock a simulated part keeps, as against the one the
/// co-simulation keeps.
///
/// Every sensor FMU stamps its samples with the master's own time, so the
/// whole instrument complement shares one perfect clock: a 100 Hz part
/// produces samples exactly 10 000 us apart, forever, in step with a 25 Hz
/// part and a 10 Hz one. No real system is like that. Each part runs on its
/// own oscillator, a few tens of parts per million off nominal and off every
/// other part, and each sample lands a little early or late of its own
/// nominal instant.
///
/// The two errors are different in kind, which is why they are separate
/// parameters:
///
/// * **Skew** is a *rate* error, drawn once per run and constant thereafter.
///   Its effect accumulates: at 50 ppm a part's clock is 50 us adrift after a
///   second and 180 ms after an hour. This is what makes two sensors' streams
///   slide relative to each other over a long flight, and what a filter fusing
///   them has to either estimate or be robust to.
/// * **Jitter** is a per-sample error with no memory. It does not accumulate;
///   it just means a sample's timestamp is not exactly when the measurement
///   was taken.
///
/// Both default to zero, which is the perfect clock the FMUs have always had.
///
/// Host-only, like the rest of the fmu/ subtree: `<random>` is fine here.

#pragma once

#include <cstdint>
#include <random>

/// @namespace hemerion::sensors
/// @brief Sensor stacks shared across the individual parts.
namespace hemerion::sensors
{

/// How far a part's own clock departs from the simulation's.
struct SensorClockConfig
{
  /// Oscillator rate error, 1-sigma, drawn once per run [parts per million].
  /// A cheap MEMS part is tens of ppm; a TCXO is single digits. The error is
  /// a *rate*, so its effect on a timestamp grows with elapsed time.
  float skew_sigma_ppm = 0.0F;

  /// Per-sample timestamp jitter, 1-sigma [seconds]. No memory, so it does
  /// not accumulate -- it is the difference between when a sample was taken
  /// and when the part says it was.
  float jitter_sigma_s = 0.0F;
};

/// @brief Turns a nominal sample instant into the one the part reports.
class SensorClock
{
public:
  /// @param config Skew and jitter magnitudes.
  /// @param rng    The model's RNG, shared so that one seed determines the
  ///               whole part -- its errors and its clock alike.
  template <typename Rng>
  SensorClock(const SensorClockConfig& config, Rng& rng) : config_(config)
  {
    // A zero sigma must not reach std::normal_distribution, which requires a
    // strictly positive stddev, and must draw nothing at all so that a
    // perfect clock leaves the rest of the model's stream untouched.
    if (config_.skew_sigma_ppm > 0.0F)
    {
      skew_ppm_ = std::normal_distribution<float>(0.0F, config_.skew_sigma_ppm)(rng);
    }
  }

  /// @brief The timestamp the part puts on a sample taken at `nominal_s`.
  ///
  /// @param nominal_s Simulation time the measurement was actually taken [s].
  /// @param rng       The model's RNG, for the jitter draw.
  /// @return Microseconds on the part's own clock, floored at zero -- a
  ///         negative timestamp would be a different kind of bug downstream
  ///         than an early one.
  template <typename Rng>
  [[nodiscard]] std::uint64_t stamp(double nominal_s, Rng& rng)
  {
    double reported_s = nominal_s * (1.0 + static_cast<double>(skew_ppm_) * 1e-6);
    if (config_.jitter_sigma_s > 0.0F)
    {
      reported_s += static_cast<double>(std::normal_distribution<float>(0.0F, config_.jitter_sigma_s)(rng));
    }
    return (reported_s <= 0.0) ? 0U : static_cast<std::uint64_t>(reported_s * 1e6);
  }

  /// @brief The rate error this instance drew [ppm], for a test to account
  /// for rather than mistake for a bug.
  [[nodiscard]] float skew_ppm() const { return skew_ppm_; }

private:
  SensorClockConfig config_;
  float skew_ppm_ = 0.0F;
};

}  // namespace hemerion::sensors
