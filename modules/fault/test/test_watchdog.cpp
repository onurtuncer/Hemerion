// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_watchdog.cpp
//
// Native unit test for the software watchdog supervisor. Run by CTest under
// the test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/fault/watchdog.h"

using hemerion::fault::kMaxWatchdogChannels;
using hemerion::fault::WatchdogError;
using hemerion::fault::WatchdogStatus;
using hemerion::fault::WatchdogSupervisor;

namespace
{

void test_no_channels_never_expires()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.step(1'000'000) == WatchdogStatus::kOk);
}

void test_channel_within_timeout_stays_ok()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.step(50) == WatchdogStatus::kOk);
  assert(!watchdog.is_expired(1));
}

void test_channel_past_timeout_expires()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.step(60) == WatchdogStatus::kOk);
  assert(watchdog.step(60) == WatchdogStatus::kExpired);
  assert(watchdog.is_expired(1));
}

void test_kick_resets_elapsed_time()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.step(90) == WatchdogStatus::kOk);
  assert(watchdog.kick(1) == WatchdogError::kNone);
  assert(watchdog.step(90) == WatchdogStatus::kOk);
  assert(!watchdog.is_expired(1));
}

void test_kick_clears_expired_state()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.step(150) == WatchdogStatus::kExpired);
  assert(watchdog.kick(1) == WatchdogError::kNone);
  assert(!watchdog.is_expired(1));
  assert(watchdog.step(10) == WatchdogStatus::kOk);
}

void test_one_expired_channel_reports_expired_overall()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.register_channel(2, 1000) == WatchdogError::kNone);
  assert(watchdog.step(150) == WatchdogStatus::kExpired);
  assert(watchdog.is_expired(1));
  assert(!watchdog.is_expired(2));
}

void test_duplicate_registration_rejected()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.register_channel(1, 100) == WatchdogError::kNone);
  assert(watchdog.register_channel(1, 200) == WatchdogError::kDuplicateId);
}

void test_unknown_id_operations_rejected()
{
  WatchdogSupervisor watchdog;
  assert(watchdog.kick(42) == WatchdogError::kUnknownId);
  assert(!watchdog.is_expired(42));
}

void test_registry_full_rejects_further_registration()
{
  WatchdogSupervisor watchdog;
  for (std::uint8_t id = 0; id < static_cast<std::uint8_t>(kMaxWatchdogChannels); ++id)
  {
    assert(watchdog.register_channel(id, 100) == WatchdogError::kNone);
  }
  assert(watchdog.register_channel(static_cast<std::uint8_t>(kMaxWatchdogChannels), 100) ==
         WatchdogError::kRegistryFull);
}

}  // namespace

int main()
{
  test_no_channels_never_expires();
  test_channel_within_timeout_stays_ok();
  test_channel_past_timeout_expires();
  test_kick_resets_elapsed_time();
  test_kick_clears_expired_state();
  test_one_expired_channel_reports_expired_overall();
  test_duplicate_registration_rejected();
  test_unknown_id_operations_rejected();
  test_registry_full_rejects_further_registration();

  std::puts("test_watchdog: all checks passed");
  return 0;
}
