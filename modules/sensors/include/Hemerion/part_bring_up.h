// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file part_bring_up.h
/// @brief When a simulated bus part starts sampling, in simulation time: the
/// bring-up gate that holds simulation time at t = 0 until the controller has
/// configured the part, and the free-running conversion schedule that then
/// runs from that instant.
///
/// **The problem these solve.** The IMU, BMP390 and MMC5983MA FMUs are
/// peripherals on shared-memory buses; the controller that brings them up is
/// the flight computer, a separate process on the wall clock. A part only
/// starts converting when the controller tells it to (BMP390 normal mode,
/// MMC5983MA continuous mode), and the IMU's bring-up empties its FIFO. If the
/// co-simulation is already stepping while that happens, *which step* the
/// command lands in is decided by the wall clock, and so is everything the
/// part does afterwards: the conversion phase, which plant state each
/// conversion samples, how many noise draws precede a given conversion, and
/// whether the IMU's first samples survive the FIFO reset. Measured on the
/// Linux CI runners (case 11, seeds 1-20, 2026-11-20): two runs of one seed
/// differed in the BMP390's first conversion by one 0.01 s step either way,
/// in one magnetometer row, and in the IMU's first sample, on 16 of 20 seeds.
///
/// **The rule.** A real board brings its sensors up before it flies, and
/// simulation time is flight time. So the first step of each such FMU holds
/// (PartBringUpGate::hold) until its controller has brought the part up, or a
/// wall-clock timeout passes; only then does simulation time start. Every
/// command of the bring-up therefore takes effect at t = 0, the part's
/// conversions run from there on a grid in simulation time
/// (FreeRunningSchedule), and its sample stream is a function of simulation
/// time and the seed alone. Nothing about the register-level protocol
/// changes: a BMP390 still starts converting when PWR_CTRL selects normal
/// mode, one period after that command -- the command is simply never late.
///
/// A part whose bring-up blocks on its own measurements (the MMC5983MA's
/// SET/RESET pair) needs them served while simulation time is held, so hold()
/// takes a service callback the FMU uses to answer triggered measurements
/// from the t = 0 inputs.
///
/// A timeout of 0 (every FMU's default) does not hold at all: the part
/// behaves exactly as it did before this gate existed. The co-simulation
/// hosts in examples/ set one.
///
/// Host-only, like the rest of the fmu/ subtrees: never cross-compiled.

#pragma once

#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>

namespace hemerion::sensors
{

/// @brief Holds a peripheral FMU's first step until its controller has
/// brought the part up.
class PartBringUpGate
{
public:
  /// How the first step was released.
  enum class Outcome : std::uint8_t
  {
    kPending,   ///< hold() has not run yet.
    kReady,     ///< The part was up when simulation time started.
    kNotReady,  ///< No timeout was set and the part was not up: simulation time started anyway.
    kTimedOut,  ///< The timeout passed before the part was up.
  };

  /// Poll interval while holding. The controller's bring-up is a few dozen
  /// bus transactions; 1 ms keeps the hold well under a step of slack.
  static constexpr std::chrono::milliseconds kPollInterval{ 1 };

  /// @brief Re-arms the gate for a new run (initialisation, or a reset).
  /// @param timeout_s Wall-clock time the first step may hold [s]; 0 or
  ///                  less does not hold.
  void arm(double timeout_s)
  {
    timeout_s_ = timeout_s;
    outcome_ = Outcome::kPending;
  }

  /// @brief Called at the top of every step; holds only the first.
  ///
  /// Repeats `service()` then `is_up()` until `is_up()` is true or the
  /// timeout passes. Without a timeout, calls each once and returns.
  ///
  /// @param is_up   True once the controller has brought the part up.
  /// @param service Called before each check, from the stepping thread, so a
  ///                part can answer the bring-up's own triggered
  ///                measurements while simulation time is held.
  template <class IsUp, class Service>
  void hold(IsUp&& is_up, Service&& service)
  {
    if (outcome_ != Outcome::kPending)
    {
      return;
    }
    service();
    if (is_up())
    {
      outcome_ = Outcome::kReady;
      return;
    }
    if (timeout_s_ <= 0.0)
    {
      outcome_ = Outcome::kNotReady;
      return;
    }
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_s_));
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(kPollInterval);
      service();
      if (is_up())
      {
        outcome_ = Outcome::kReady;
        return;
      }
    }
    outcome_ = Outcome::kTimedOut;
  }

  /// @brief hold() with nothing to service.
  template <class IsUp>
  void hold(IsUp&& is_up)
  {
    hold(std::forward<IsUp>(is_up), [] {});
  }

  [[nodiscard]] Outcome outcome() const { return outcome_; }

  /// True when the part was up as simulation time started, so its sample
  /// stream does not depend on when the controller ran.
  [[nodiscard]] bool ready_at_start() const { return outcome_ == Outcome::kReady; }

private:
  double timeout_s_ = 0.0;
  Outcome outcome_ = Outcome::kPending;
};

/// @brief A part's free-running measurement engine, in simulation time.
///
/// Whole periods elapsed within a step each produce one conversion, stamped
/// at its own instant within the step; the remainder carries into the next
/// step, so a 50 Hz part stepped at 10 Hz really converts at 50 Hz. While the
/// part is idle (period 0) the phase is held at zero, so a part put into a
/// free-running mode converts one period after the step that saw the command.
/// Shared by the BMP390 (normal mode) and MMC5983MA (continuous mode) FMUs.
class FreeRunningSchedule
{
public:
  /// @brief Advances the engine across one step.
  /// @param dt_s      Step length [s].
  /// @param period_us Conversion period the part's registers select now
  ///                  [microseconds]; 0 while it is not free-running.
  /// @param convert   Called once per conversion with its offset into the
  ///                  step [s] (in (0, dt_s]).
  template <class Convert>
  void advance(double dt_s, std::uint64_t period_us, Convert&& convert)
  {
    if (period_us == 0)
    {
      time_into_period_s_ = 0.0;  // idle: the measurement engine is not running
      return;
    }
    const double period_s = static_cast<double>(period_us) * 1e-6;
    double remaining_s = dt_s;
    while (time_into_period_s_ + remaining_s >= period_s)
    {
      remaining_s -= period_s - time_into_period_s_;
      time_into_period_s_ = 0.0;
      convert(dt_s - remaining_s);
    }
    time_into_period_s_ += remaining_s;
  }

  /// Power-on: no conversion in progress.
  void reset() { time_into_period_s_ = 0.0; }

private:
  double time_into_period_s_ = 0.0;
};

}  // namespace hemerion::sensors
