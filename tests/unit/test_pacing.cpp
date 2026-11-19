// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_pacing.cpp
//
// examples/common/pacing.hpp: the co-simulation hosts' pacing report (how far
// a paced run fell behind its wall-clock targets, whether --rtf held), its
// console line and sidecar keys, and the Windows timer-resolution scope.
// ------------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "pacing.hpp"

using hemerion::examples::append_pacing_to_sidecar;
using hemerion::examples::PacingMonitor;
using hemerion::examples::print_pacing;
using hemerion::examples::ScopedTimerResolution;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// As in test_example_environment.cpp: a check that survives NDEBUG.
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

std::map<std::string, std::string> read_keys(const std::filesystem::path& path)
{
  std::map<std::string, std::string> keys;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line))
  {
    const auto equals = line.find('=');
    if (equals != std::string::npos)
    {
      keys[line.substr(0, equals)] = line.substr(equals + 1);
    }
  }
  return keys;
}

// A run that keeps up: lags at or under the tolerance (an early step counts as
// zero, not as negative lag), --rtf honoured, no late steps.
void test_a_run_that_keeps_up()
{
  PacingMonitor monitor(1.0, std::chrono::steady_clock::now());
  for (const double lag_s : { -0.004, 0.001, 0.0, 0.019, -0.5, 0.02 })
  {
    monitor.record(lag_s);
  }
  const auto& report = monitor.report();
  CHECK(report.steps == 6);
  CHECK(near(report.max_lag_s, 0.02, 1e-12));
  CHECK(report.final_lag_s == 0.02);
  CHECK(report.late_steps == 0);
  CHECK(report.honoured());
}

// A run that falls behind: the worst lag, where it ended, and how many steps
// were later than the tolerance -- and --rtf reported as not honoured.
void test_a_run_that_falls_behind()
{
  PacingMonitor monitor(1.0, std::chrono::steady_clock::now(), 0.05);
  for (const double lag_s : { 0.0, 0.03, 0.2, 0.9, 0.6 })
  {
    monitor.record(lag_s);
  }
  const auto& report = monitor.report();
  CHECK(near(report.max_lag_s, 0.9, 1e-12));
  CHECK(near(report.final_lag_s, 0.6, 1e-12));
  CHECK(report.late_steps == 3);
  CHECK(!report.honoured());

  std::ostringstream line;
  print_pacing(line, report);
  CHECK(line.str().find("NOT honoured: max lag 0.9 s, final lag 0.6 s, 3 of 5 steps") != std::string::npos);
  CHECK(line.str().find("warning: this host could not keep real time") != std::string::npos);
}

// pace() measures the lag against the absolute target start + t / rtf. With a
// start far enough in the past every target is already due, so nothing sleeps
// and the lags are exact: at rtf 2, sim time 4 s is due 2 s after the start.
void test_pace_measures_against_the_absolute_target()
{
  const auto now = std::chrono::steady_clock::now();
  const auto start = now - std::chrono::seconds(10);
  PacingMonitor monitor(2.0, start);
  monitor.pace(4.0, now);   // due at start + 2 s, now is start + 10 s: 8 s late
  monitor.pace(20.0, now);  // due at start + 10 s: on time
  CHECK(near(monitor.report().max_lag_s, 8.0, 1e-6));
  CHECK(near(monitor.report().final_lag_s, 0.0, 1e-6));
  CHECK(monitor.report().late_steps == 1);
}

// Unpaced: pace() records nothing, the report is "unpaced", and the sidecar
// gains no keys.
void test_unpaced_reports_nothing()
{
  PacingMonitor monitor(0.0, std::chrono::steady_clock::now());
  monitor.pace(1.0);
  CHECK(monitor.report().steps == 0);
  CHECK(!monitor.report().honoured());
  std::ostringstream line;
  print_pacing(line, monitor.report());
  CHECK(line.str().find("unpaced") != std::string::npos);

  const auto dir = std::filesystem::temp_directory_path() / "hemerion_test_pacing_unpaced";
  std::filesystem::create_directories(dir);
  {
    std::ofstream(dir / "truth.config") << "seed=5\n";
  }
  append_pacing_to_sidecar(dir / "truth.csv", monitor.report());
  CHECK(read_keys(dir / "truth.config").size() == 1);
  std::filesystem::remove_all(dir);
}

// Paced: the keys are appended after what the host wrote before the run,
// which stays exactly as it was.
void test_sidecar_keys_are_appended()
{
  PacingMonitor monitor(1.0, std::chrono::steady_clock::now());
  monitor.record(0.25);
  monitor.record(0.125);
  const auto dir = std::filesystem::temp_directory_path() / "hemerion_test_pacing_paced";
  std::filesystem::create_directories(dir);
  {
    std::ofstream(dir / "f16_truth.config") << "seed=5\nrealtime_factor=1\n";
  }
  append_pacing_to_sidecar(dir / "f16_truth.csv", monitor.report());
  const auto keys = read_keys(dir / "f16_truth.config");
  CHECK(keys.at("seed") == "5");
  CHECK(keys.at("realtime_factor") == "1");
  CHECK(keys.at("pacing_rtf_honoured") == "0");
  CHECK(keys.at("pacing_max_lag_s") == "0.25");
  CHECK(keys.at("pacing_final_lag_s") == "0.125");
  CHECK(keys.at("pacing_late_steps") == "2");
  CHECK(keys.at("pacing_steps") == "2");
  CHECK(keys.at("pacing_tolerance_s") == "0.02");
  std::filesystem::remove_all(dir);
}

// The timer scope is active on Windows and inert elsewhere; either way it
// constructs and destructs cleanly, nested too.
void test_timer_resolution_scope()
{
  const ScopedTimerResolution outer;
  {
    const ScopedTimerResolution inner;
#ifdef _WIN32
    CHECK(inner.active());
#else
    CHECK(!inner.active());
#endif
  }
#ifdef _WIN32
  CHECK(outer.active());
#else
  CHECK(!outer.active());
#endif
}

}  // namespace

int main()
{
  test_a_run_that_keeps_up();
  test_a_run_that_falls_behind();
  test_pace_measures_against_the_absolute_target();
  test_unpaced_reports_nothing();
  test_sidecar_keys_are_appended();
  test_timer_resolution_scope();
  std::cout << "test_pacing: all checks passed\n";
  return 0;
}
