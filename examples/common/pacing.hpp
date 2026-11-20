// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file pacing.hpp
/// @brief Wall-clock pacing for the co-simulation hosts, the honest report of
/// how well it held, whether the bus parts were brought up before simulation
/// time started, and the Windows timer resolution the hosts and flight
/// computers run at.
///
/// **Why the report.** `--rtf 1` asks for one simulated second per wall
/// second, and the flight computer's arrival stamps (`host_time_s`) are only a
/// clock a filter can use if that holds. It does not always: the pacing below
/// sleeps to absolute targets, so a host whose plant step costs more than the
/// step itself simply falls behind, for good, by however much the step costs
/// vary. On the Linux CI runners that was 11 of 20 seeds of the case-11 batch
/// keeping real time (stamps fit to 1-4 ms rms) and 9 falling behind (scale
/// 1.03-1.36, 231-471 ms rms) -- the plant FMU's step cost, which is
/// Aetherion's, not anything on this side (TODO-sensor-fusion.md, "Host-clock
/// residual"). The host cannot catch up, but it can say so: PacingMonitor
/// records the lag at every step and the hosts print it and append it to the
/// run's sidecar.
///
/// **Why the timer.** Windows sleeps in whole scheduler ticks, 15.6 ms by
/// default, and the shared-memory buses back off with a 50 us sleep between
/// polls. Each SPI or I2C transaction therefore waited about a tick for the
/// other side to wake, the flight computer's loop took ~400 ms, its IMU, GNSS
/// and magnetometer stamps lagged by up to seconds, and the BMP390's single
/// data register was overwritten between polls -- the "starved" Windows
/// barometer. ScopedTimerResolution asks for 1 ms for the process's lifetime;
/// on Windows 10 2004 and later the request is per process, so every process
/// on a bus (the host, whose sensor FMUs run the peripheral side, and the
/// flight computer, which runs the controller side) makes its own. It is
/// compiled out everywhere else: on Linux a 50 us sleep is already ~50 us.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <thread>

#ifdef _WIN32
// Declared rather than taken from <windows.h>, which would bring its macros
// (min, max, near, far, ERROR...) into every host translation unit. These are
// winmm's exports exactly (UINT/MMRESULT are unsigned int); the targets link
// winmm on Windows.
extern "C" __declspec(dllimport) unsigned int __stdcall timeBeginPeriod(unsigned int uPeriod);
extern "C" __declspec(dllimport) unsigned int __stdcall timeEndPeriod(unsigned int uPeriod);
#endif

namespace hemerion::examples
{

/// @brief 1 ms scheduler resolution for as long as the object lives, on
/// Windows; nothing anywhere else. Construct one at the top of main().
class ScopedTimerResolution
{
public:
  ScopedTimerResolution()
  {
#ifdef _WIN32
    active_ = (timeBeginPeriod(kPeriodMs) == 0U);  // TIMERR_NOERROR
#endif
  }
  ~ScopedTimerResolution()
  {
#ifdef _WIN32
    if (active_)
    {
      timeEndPeriod(kPeriodMs);
    }
#endif
  }
  ScopedTimerResolution(const ScopedTimerResolution&) = delete;
  ScopedTimerResolution& operator=(const ScopedTimerResolution&) = delete;
  ScopedTimerResolution(ScopedTimerResolution&&) = delete;
  ScopedTimerResolution& operator=(ScopedTimerResolution&&) = delete;

  /// @brief Whether a finer resolution is in force (always false off Windows).
  [[nodiscard]] bool active() const { return active_; }

private:
  static constexpr unsigned int kPeriodMs = 1U;
  bool active_ = false;
};

/// @brief How far the paced run fell behind its wall-clock targets.
struct PacingReport
{
  double realtime_factor = 0.0;  ///< The --rtf the run asked for; 0 = unpaced, nothing below is meaningful.
  long steps = 0;                ///< Steps paced.
  double max_lag_s = 0.0;        ///< Largest lag behind a step's target, before sleeping [s].
  double final_lag_s = 0.0;      ///< Lag at the last step [s]: how much later than asked the run ended.
  long late_steps = 0;           ///< Steps that finished more than the tolerance behind their target.
  double tolerance_s = 0.0;      ///< The lag a step may have and still count as on time [s].

  /// @brief Whether --rtf was honoured: no step fell more than the tolerance behind.
  [[nodiscard]] bool honoured() const { return realtime_factor > 0.0 && max_lag_s <= tolerance_s; }
};

/// @brief Paces a fixed-step loop to wall-clock time and records the lag.
///
/// pace() sleeps until `start + sim_time / rtf`, exactly as the hosts always
/// have -- absolute targets, so an early wake-up or a late one does not
/// accumulate -- and before sleeping notes how far past that target the step
/// already is. Unpaced (rtf <= 0) it does nothing and reports nothing.
class PacingMonitor
{
public:
  /// Within this, a step is on time: two 0.01 s steps, the resolution a filter
  /// fitting the stamps can ask of them, and well above a scheduler's wake-up
  /// jitter at 1 ms resolution.
  static constexpr double kDefaultToleranceS = 0.02;

  PacingMonitor(double realtime_factor,
                std::chrono::steady_clock::time_point start,
                double tolerance_s = kDefaultToleranceS)
    : start_(start)
  {
    report_.realtime_factor = realtime_factor;
    report_.tolerance_s = tolerance_s;
  }

  /// @brief Records the lag of the step that just reached `sim_time_s`, then
  /// sleeps until its target. `now` is a parameter so the lag arithmetic can
  /// be tested without a clock; the hosts pass steady_clock::now().
  void pace(double sim_time_s, std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    if (report_.realtime_factor <= 0.0)
    {
      return;
    }
    const auto target = start_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(sim_time_s / report_.realtime_factor));
    record(std::chrono::duration<double>(now - target).count());
    std::this_thread::sleep_until(target);
  }

  /// @brief Re-anchors the targets so the step that just reached
  /// `sim_time_s` is exactly on time now. The hosts call it after the first
  /// step, which holds on the wall clock until the flight computer has brought
  /// the bus parts up (part_bring_up.h): that hold is not simulation time, and
  /// pacing from before it would leave every later step late by its length.
  void restart(double sim_time_s, std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    if (report_.realtime_factor <= 0.0)
    {
      return;
    }
    start_ = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(sim_time_s / report_.realtime_factor));
  }

  /// @brief Records one step's lag behind its target [s]; negative is early.
  void record(double lag_s)
  {
    const double late_s = std::max(0.0, lag_s);
    ++report_.steps;
    report_.max_lag_s = std::max(report_.max_lag_s, late_s);
    report_.final_lag_s = late_s;
    if (late_s > report_.tolerance_s)
    {
      ++report_.late_steps;
    }
  }

  [[nodiscard]] const PacingReport& report() const { return report_; }

private:
  std::chrono::steady_clock::time_point start_;
  PacingReport report_;
};

/// @brief The report as one console line, `[cosim] pacing: ...`.
inline void print_pacing(std::ostream& out, const PacingReport& report)
{
  if (report.realtime_factor <= 0.0)
  {
    out << "[cosim] pacing: unpaced (--rtf 0); arrival stamps are not a clock\n";
    return;
  }
  out << "[cosim] pacing: --rtf " << report.realtime_factor << (report.honoured() ? " honoured" : " NOT honoured")
      << ": max lag " << report.max_lag_s << " s, final lag " << report.final_lag_s << " s, " << report.late_steps
      << " of " << report.steps << " steps more than " << report.tolerance_s << " s late\n";
  if (!report.honoured())
  {
    out << "[cosim] warning: this host could not keep real time, so the flight computer's host_time_s is not a "
           "clock for this run (see scripts/fit_arrival_clock.py)\n";
  }
}

/// @brief Appends the report to the run's sidecar (`<csv stem>.config`) as
/// optional keys. Appended after the run because it is only known then; the
/// rest of the sidecar is written before the first step. Unpaced runs add
/// nothing.
///
/// Keys: `pacing_rtf_honoured` (1/0), `pacing_max_lag_s`,
/// `pacing_final_lag_s`, `pacing_late_steps`, `pacing_steps`,
/// `pacing_tolerance_s`. None is part of the log contract with Aetherion's
/// replay (log_contract.hpp names only the keys it requires).
inline void append_pacing_to_sidecar(const std::filesystem::path& csv_path, const PacingReport& report)
{
  if (report.realtime_factor <= 0.0)
  {
    return;
  }
  std::filesystem::path config_path = csv_path;
  config_path.replace_extension(".config");
  std::ofstream out(config_path, std::ios::app);
  if (!out)
  {
    return;  // the sidecar itself already warned when it could not be written
  }
  out << "pacing_rtf_honoured=" << (report.honoured() ? 1 : 0) << "\n"
      << "pacing_max_lag_s=" << report.max_lag_s << "\n"
      << "pacing_final_lag_s=" << report.final_lag_s << "\n"
      << "pacing_late_steps=" << report.late_steps << "\n"
      << "pacing_steps=" << report.steps << "\n"
      << "pacing_tolerance_s=" << report.tolerance_s << "\n";
}

/// @brief Whether the flight computer had brought each bus part up before
/// simulation time started -- the condition for its sensor logs to be a
/// function of the seed alone (part_bring_up.h). Read from the FMUs'
/// `ready_at_start` outputs after the run.
struct BringUpReport
{
  double timeout_s = 0.0;  ///< The parts' bring_up_timeout_s; 0 = the first step did not hold.
  int imu_ready = 0;       ///< 1 if the IMU was up at t = 0.
  int baro_ready = 0;      ///< 1 if the BMP390 was measuring at t = 0.
  int mag_ready = 0;       ///< 1 if the MMC5983MA was in continuous mode at t = 0.

  [[nodiscard]] bool all_ready() const { return imu_ready == 1 && baro_ready == 1 && mag_ready == 1; }
};

/// @brief The report as one console line, `[cosim] bring-up: ...`.
inline void print_bring_up(std::ostream& out, const BringUpReport& report)
{
  out << "[cosim] bring-up: IMU " << (report.imu_ready == 1 ? "ready" : "NOT ready") << ", BMP390 "
      << (report.baro_ready == 1 ? "ready" : "NOT ready") << ", MMC5983MA "
      << (report.mag_ready == 1 ? "ready" : "NOT ready") << " at t = 0 (hold up to " << report.timeout_s << " s)\n";
  if (!report.all_ready())
  {
    out << "[cosim] warning: a part was brought up after simulation time started, so this run's sensor logs "
           "depend on when the flight computer ran, not only on the seed\n";
  }
}

/// @brief Appends the report to the run's sidecar, after the pacing keys.
///
/// Keys: `bring_up_timeout_s`, `bring_up_imu_ready`, `bring_up_baro_ready`,
/// `bring_up_mag_ready` (1/0). Not log-contract keys.
inline void append_bring_up_to_sidecar(const std::filesystem::path& csv_path, const BringUpReport& report)
{
  std::filesystem::path config_path = csv_path;
  config_path.replace_extension(".config");
  std::ofstream out(config_path, std::ios::app);
  if (!out)
  {
    return;  // the sidecar itself already warned when it could not be written
  }
  out << "bring_up_timeout_s=" << report.timeout_s << "\n"
      << "bring_up_imu_ready=" << report.imu_ready << "\n"
      << "bring_up_baro_ready=" << report.baro_ready << "\n"
      << "bring_up_mag_ready=" << report.mag_ready << "\n";
}

}  // namespace hemerion::examples
