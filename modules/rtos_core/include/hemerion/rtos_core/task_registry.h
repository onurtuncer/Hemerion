// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/rtos_core/task_registry.h
//
// Central static registry of every FreeRTOS task an app intends to create.
// This is deliberately decoupled from xTaskCreate(): bsp/stm32h743_nucleo now
// supplies FreeRTOSConfig.h and the freertos_config/FREERTOS_PORT wiring (see
// vendor/FreeRTOS-Kernel/CMakeLists.txt), but this module itself still only
// links ETL, not FreeRTOS::Kernel -- see modules/rtos_core/CMakeLists.txt.
// Apps register descriptors here during startup so the set of tasks can be
// validated (no duplicate names, in-range priorities, sane stack sizes) on
// host; the loop that calls xTaskCreate() once per descriptor is the next step.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "hemerion/rtos_core/task_priority.h"

namespace hemerion::rtos_core {

inline constexpr std::size_t kMaxTasks = 16;
inline constexpr std::uint32_t kMinStackWords = 128;

// Matches FreeRTOS's TaskFunction_t signature so a descriptor's entry_point
// can be passed to xTaskCreate() unchanged once that step is wired up.
using TaskEntryPoint = void (*)(void*);

enum class TaskRegistryError : std::uint8_t {
  kNone,
  kRegistryFull,
  kDuplicateName,
  kInvalidPriority,
  kStackTooSmall,
  kNullEntryPoint,
};

struct TaskDescriptor {
  const char* name = nullptr;
  TaskEntryPoint entry_point = nullptr;
  void* argument = nullptr;
  std::uint8_t priority = kPriorityIdle;
  std::uint32_t stack_words = kMinStackWords;
  std::uint32_t period_ms = 0;  // 0 = event-driven / not periodic
};

class TaskRegistry {
 public:
  // Rejects the descriptor (without modifying the registry) if its name is
  // null/already registered, its entry point is null, its priority is out of
  // range, its stack is below kMinStackWords, or the registry is full.
  [[nodiscard]] TaskRegistryError register_task(const TaskDescriptor& descriptor);

  [[nodiscard]] std::size_t count() const { return count_; }
  [[nodiscard]] const TaskDescriptor& at(std::size_t index) const { return tasks_[index]; }
  [[nodiscard]] const TaskDescriptor* find_by_name(const char* name) const;

  void clear() { count_ = 0; }

 private:
  std::array<TaskDescriptor, kMaxTasks> tasks_{};
  std::size_t count_ = 0;
};

}  // namespace hemerion::rtos_core
