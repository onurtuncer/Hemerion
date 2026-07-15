// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_task_registry.cpp
//
// Native unit test for task priority bands, tick conversion, and the task
// registry. Run by CTest under the test-native preset. Unity (referenced in
// modules/README.md as the project's test framework) is not yet vendored, so
// this uses plain asserts and an exit code; switch to Unity test cases once
// vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdio>

#include "hemerion/rtos_core/task_priority.h"
#include "hemerion/rtos_core/task_registry.h"
#include "hemerion/rtos_core/tick.h"

using hemerion::rtos_core::band_of;
using hemerion::rtos_core::is_valid_priority;
using hemerion::rtos_core::kPriorityCritical;
using hemerion::rtos_core::kPriorityHighMin;
using hemerion::rtos_core::kPriorityIdle;
using hemerion::rtos_core::kPriorityLowMin;
using hemerion::rtos_core::kPriorityMidMin;
using hemerion::rtos_core::ms_to_ticks;
using hemerion::rtos_core::TaskDescriptor;
using hemerion::rtos_core::TaskPriorityBand;
using hemerion::rtos_core::TaskRegistry;
using hemerion::rtos_core::TaskRegistryError;
using hemerion::rtos_core::ticks_to_ms;

namespace {

void noop_entry_point(void*) {}

// Priority bands, validity, and tick conversion are constexpr and verified at
// compile time.
static_assert(band_of(kPriorityIdle) == TaskPriorityBand::kIdle);
static_assert(band_of(kPriorityLowMin) == TaskPriorityBand::kLow);
static_assert(band_of(kPriorityMidMin) == TaskPriorityBand::kMid);
static_assert(band_of(kPriorityHighMin) == TaskPriorityBand::kHigh);
static_assert(band_of(kPriorityCritical) == TaskPriorityBand::kCritical);

static_assert(is_valid_priority(kPriorityCritical));
static_assert(!is_valid_priority(kPriorityCritical + 1));

static_assert(ms_to_ticks(1000) == 1000);
static_assert(ticks_to_ms(1000) == 1000);
static_assert(ms_to_ticks(0) == 0);

static_assert(ms_to_ticks(10, 100) == 1);  // 10ms @ 100Hz tick rate -> 1 tick
static_assert(ticks_to_ms(1, 100) == 10);  // 1 tick @ 100Hz tick rate -> 10ms

void test_register_task_accepts_valid_descriptor() {
  TaskRegistry registry;
  TaskDescriptor descriptor;
  descriptor.name = "control_loop";
  descriptor.entry_point = &noop_entry_point;
  descriptor.priority = kPriorityHighMin;
  descriptor.stack_words = 256;

  assert(registry.register_task(descriptor) == TaskRegistryError::kNone);
  assert(registry.count() == 1);
  assert(registry.find_by_name("control_loop") != nullptr);
  assert(registry.find_by_name("missing") == nullptr);
}

void test_register_task_rejects_invalid_descriptors() {
  TaskRegistry registry;
  TaskDescriptor descriptor;
  descriptor.name = "bad";
  descriptor.entry_point = nullptr;
  assert(registry.register_task(descriptor) == TaskRegistryError::kNullEntryPoint);

  descriptor.entry_point = &noop_entry_point;
  descriptor.priority = kPriorityCritical + 1;
  assert(registry.register_task(descriptor) == TaskRegistryError::kInvalidPriority);

  descriptor.priority = kPriorityIdle;
  descriptor.stack_words = 1;
  assert(registry.register_task(descriptor) == TaskRegistryError::kStackTooSmall);
}

void test_register_task_rejects_duplicate_name() {
  TaskRegistry registry;
  TaskDescriptor descriptor;
  descriptor.name = "telemetry";
  descriptor.entry_point = &noop_entry_point;
  descriptor.stack_words = 256;

  assert(registry.register_task(descriptor) == TaskRegistryError::kNone);
  assert(registry.register_task(descriptor) == TaskRegistryError::kDuplicateName);
}

void test_register_task_rejects_when_full() {
  TaskRegistry registry;
  TaskDescriptor descriptor;
  descriptor.entry_point = &noop_entry_point;
  descriptor.stack_words = 256;

  std::array<std::array<char, 2>, hemerion::rtos_core::kMaxTasks + 1> names{};
  for (std::size_t i = 0; i < hemerion::rtos_core::kMaxTasks; ++i) {
    names[i][0] = static_cast<char>('a' + i);
    descriptor.name = names[i].data();
    assert(registry.register_task(descriptor) == TaskRegistryError::kNone);
  }

  names[hemerion::rtos_core::kMaxTasks][0] = 'z';
  descriptor.name = names[hemerion::rtos_core::kMaxTasks].data();
  assert(registry.register_task(descriptor) == TaskRegistryError::kRegistryFull);
}

}  // namespace

int main() {
  test_register_task_accepts_valid_descriptor();
  test_register_task_rejects_invalid_descriptors();
  test_register_task_rejects_duplicate_name();
  test_register_task_rejects_when_full();

  std::puts("test_task_registry: all checks passed");
  return 0;
}
