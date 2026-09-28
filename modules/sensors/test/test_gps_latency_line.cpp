// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_gps_latency_line.cpp
//
// The receiver's processing latency (Hemerion/gps/fmu/gpsLatencyLine.hpp).
//
// The logic lives in a header rather than in fmu_main.cpp for the same reason
// SensorClock does: an FMU entry point cannot be unit-tested, and "the frame
// for epoch k is delivered during epoch k+n" is exactly the kind of
// off-by-one-step claim that wants an assertion rather than a co-simulation.
//
// Plain asserts + exit code, matching the other sensors tests.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "Hemerion/gps/fmu/gpsLatencyLine.hpp"

using hemerion::sensors::gps::fmu::GpsLatencyLine;

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

/// A stand-in for an encoded frame: the epoch it describes, so a test can say
/// which solution came out rather than merely how many did.
using Epoch = int;

/// Runs `steps` communication steps of `dt`, pushing one frame per step with
/// `latency` of delay, and records which epoch was delivered at which step.
///
/// Returns (step index, epoch delivered) pairs, in delivery order.
std::vector<std::pair<int, Epoch>> run(double dt, double latency, int steps)
{
  GpsLatencyLine<Epoch> line;
  std::vector<std::pair<int, Epoch>> delivered;
  for (int k = 1; k <= steps; ++k)
  {
    const double epoch_time = static_cast<double>(k) * dt;
    line.push(epoch_time + latency, k);
    Epoch out = 0;
    while (line.pop_due(epoch_time, out))
    {
      delivered.emplace_back(k, out);
    }
  }
  return delivered;
}

/// Zero latency must be the identity: every epoch delivered in its own step.
///
/// This is what makes the parameter safe to default to 0 -- the whole point of
/// a default is that runs predating it are unchanged, and "statistically
/// similar" would not do.
void test_zero_latency_is_the_identity()
{
  const auto delivered = run(0.1, 0.0, 20);
  CHECK(delivered.size() == 20);
  for (std::size_t i = 0; i < delivered.size(); ++i)
  {
    const int step = delivered[i].first;
    const Epoch epoch = delivered[i].second;
    CHECK(step == static_cast<int>(i) + 1);
    CHECK(epoch == step);
  }
}

/// The frame for epoch k arrives during epoch k+n, where n = latency / step.
///
/// The claim the file exists to make, asserted rather than described.
void test_a_solution_arrives_n_steps_late()
{
  // 100 ms steps, 250 ms of latency: an epoch's frame is due 2.5 steps later,
  // so it is delivered on the third.
  const auto delivered = run(0.1, 0.25, 12);
  CHECK(!delivered.empty());
  for (const auto& [step, epoch] : delivered)
  {
    CHECK(step - epoch == 3);
  }

  // Nothing at all comes out before the first frame is due.
  CHECK(delivered.front().first == 4);
  CHECK(delivered.front().second == 1);
}

/// A step longer than the latency makes several frames due at once, and all of
/// them must come out.
///
/// The failure this guards is a receiver that *discards* solutions rather than
/// delaying them, which is what an `if` in place of the caller's `while` would
/// model -- and which would show up as a fix rate quietly below the configured
/// one rather than as an error.
void test_a_long_step_delivers_every_due_frame()
{
  GpsLatencyLine<Epoch> line;
  for (int k = 1; k <= 5; ++k)
  {
    line.push(static_cast<double>(k) * 0.1, k);
  }
  CHECK(line.pending() == 5);

  int count = 0;
  Epoch out = 0;
  Epoch last = 0;
  while (line.pop_due(1.0, out))
  {
    ++count;
    CHECK(out > last);  // first in, first out
    last = out;
  }
  CHECK(count == 5);
  CHECK(line.pending() == 0);
}

/// At steady state the queue holds ceil(latency / step) frames and stops
/// growing. A queue that grew without bound would be a leak in a part meant to
/// run for a 200-second flight at 10 Hz.
void test_the_queue_reaches_a_steady_depth()
{
  GpsLatencyLine<Epoch> line;
  const double dt = 0.1;
  const double latency = 0.25;
  std::size_t worst = 0;
  for (int k = 1; k <= 2000; ++k)
  {
    const double epoch_time = static_cast<double>(k) * dt;
    line.push(epoch_time + latency, k);
    Epoch out = 0;
    while (line.pop_due(epoch_time, out))
    {
    }
    worst = std::max(worst, line.pending());
  }
  CHECK(worst == 3);  // ceil(0.25 / 0.1)
  CHECK(line.pending() == 3);
}

/// Reset drops what is in flight, so an FMI reset does not deliver the
/// previous run's solutions into the next one.
void test_reset_drops_frames_in_flight()
{
  GpsLatencyLine<Epoch> line;
  for (int k = 1; k <= 4; ++k)
  {
    line.push(static_cast<double>(k), k);
  }
  CHECK(line.pending() == 4);
  line.reset();
  CHECK(line.pending() == 0);

  Epoch out = 0;
  CHECK(!line.pop_due(1e9, out));
}

/// What the latency costs, in the unit that matters.
///
/// Not a property of the queue but of what it models, and the reason the
/// parameter exists: at the F-16 examples' trim speed a receiver's ordinary
/// processing delay is metres of systematic, unaveragable position error
/// along the velocity vector.
void test_the_position_error_it_represents()
{
  const double speed_mps = 172.0;  // the F-16 check-cases' trim speed
  for (const auto& [latency_s, expected_m] : { std::pair<double, double>{ 0.05, 8.6 }, { 0.1, 17.2 }, { 0.2, 34.4 } })
  {
    CHECK(std::fabs((speed_mps * latency_s) - expected_m) < 0.05);
  }
}

}  // namespace

int main()
{
  test_zero_latency_is_the_identity();
  test_a_solution_arrives_n_steps_late();
  test_a_long_step_delivers_every_due_frame();
  test_the_queue_reaches_a_steady_depth();
  test_reset_drops_frames_in_flight();
  test_the_position_error_it_represents();

  std::puts("test_gps_latency_line: all checks passed");
  return 0;
}
