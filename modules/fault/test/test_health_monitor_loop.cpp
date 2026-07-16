// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_health_monitor_loop.cpp
//
// Integration-style test for FaultRegistry + WatchdogSupervisor used
// together the way a health-monitor task would: each scheduler tick, step()
// the watchdog and raise/clear a fault code based on the result. This is the
// intended usage pattern for both pieces (see their header comments), but
// the health monitor task itself, and the rtos_core queues that would feed
// it fault reports from other tasks, are not yet built -- so this lives
// here, exercising only modules/fault, rather than in tests/integration/
// (reserved for flows that actually span multiple modules; see
// tests/README.md). Run by CTest under the test-native preset.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/fault/fault_registry.h"
#include "hemerion/fault/watchdog.h"

using hemerion::fault::FaultRegistry;
using hemerion::fault::FaultRegistryError;
using hemerion::fault::FaultSeverity;
using hemerion::fault::WatchdogError;
using hemerion::fault::WatchdogSupervisor;

namespace
{

constexpr std::uint16_t kFaultGncTaskStale = 1;
constexpr std::uint16_t kFaultImuTaskStale = 2;

constexpr std::uint8_t kChannelGncTask = 0;
constexpr std::uint8_t kChannelImuTask = 1;

struct HealthMonitor
{
  FaultRegistry faults;
  WatchdogSupervisor watchdog;

  void init()
  {
    assert(faults.register_code(kFaultGncTaskStale, FaultSeverity::kCritical) == FaultRegistryError::kNone);
    assert(faults.register_code(kFaultImuTaskStale, FaultSeverity::kWarning) == FaultRegistryError::kNone);
    assert(watchdog.register_channel(kChannelGncTask, 50) == WatchdogError::kNone);
    assert(watchdog.register_channel(kChannelImuTask, 20) == WatchdogError::kNone);
  }

  // Mirrors the per-tick body a real health-monitor task would run: advance
  // the watchdog, then reflect each channel's expiry into the matching fault
  // code so the rest of the system only has to consult the fault registry.
  void step(std::uint32_t elapsed_ms)
  {
    watchdog.step(elapsed_ms);

    if (watchdog.is_expired(kChannelGncTask))
    {
      assert(faults.raise(kFaultGncTaskStale) == FaultRegistryError::kNone);
    }
    else
    {
      assert(faults.clear(kFaultGncTaskStale) == FaultRegistryError::kNone);
    }

    if (watchdog.is_expired(kChannelImuTask))
    {
      assert(faults.raise(kFaultImuTaskStale) == FaultRegistryError::kNone);
    }
    else
    {
      assert(faults.clear(kFaultImuTaskStale) == FaultRegistryError::kNone);
    }
  }
};

void test_nominal_ticks_with_regular_kicks_stay_clear()
{
  HealthMonitor monitor;
  monitor.init();

  for (int tick = 0; tick < 5; ++tick)
  {
    assert(monitor.watchdog.kick(kChannelGncTask) == WatchdogError::kNone);
    assert(monitor.watchdog.kick(kChannelImuTask) == WatchdogError::kNone);
    monitor.step(10);
  }

  assert(monitor.faults.active_count() == 0);
  assert(monitor.faults.highest_active_severity() == FaultSeverity::kInfo);
}

void test_stalled_imu_task_raises_warning_without_tripping_critical()
{
  HealthMonitor monitor;
  monitor.init();

  // GNC task keeps kicking; IMU task stops.
  for (int tick = 0; tick < 3; ++tick)
  {
    assert(monitor.watchdog.kick(kChannelGncTask) == WatchdogError::kNone);
    monitor.step(10);
  }

  assert(monitor.faults.is_active(kFaultImuTaskStale));
  assert(!monitor.faults.is_active(kFaultGncTaskStale));
  assert(monitor.faults.highest_active_severity() == FaultSeverity::kWarning);
}

void test_stalled_gnc_task_escalates_to_critical()
{
  HealthMonitor monitor;
  monitor.init();

  // Neither task kicks; advance past both timeouts.
  monitor.step(60);

  assert(monitor.faults.is_active(kFaultGncTaskStale));
  assert(monitor.faults.is_active(kFaultImuTaskStale));
  assert(monitor.faults.highest_active_severity() == FaultSeverity::kCritical);
}

void test_resumed_kicks_clear_the_fault()
{
  HealthMonitor monitor;
  monitor.init();

  monitor.step(60);
  assert(monitor.faults.is_active(kFaultGncTaskStale));

  assert(monitor.watchdog.kick(kChannelGncTask) == WatchdogError::kNone);
  assert(monitor.watchdog.kick(kChannelImuTask) == WatchdogError::kNone);
  monitor.step(10);

  assert(monitor.faults.active_count() == 0);
  assert(monitor.faults.highest_active_severity() == FaultSeverity::kInfo);
}

}  // namespace

int main()
{
  test_nominal_ticks_with_regular_kicks_stay_clear();
  test_stalled_imu_task_raises_warning_without_tripping_critical();
  test_stalled_gnc_task_escalates_to_critical();
  test_resumed_kicks_clear_the_fault();

  std::puts("test_health_monitor_loop: all checks passed");
  return 0;
}
