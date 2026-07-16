// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file task_registry.h
/// @brief Central static registry of every FreeRTOS task an app intends to
/// create.
///
/// This is deliberately decoupled from xTaskCreate(): bsp/stm32h743_nucleo
/// now supplies FreeRTOSConfig.h and the freertos_config/FREERTOS_PORT
/// wiring (see vendor/FreeRTOS-Kernel/CMakeLists.txt), but this module
/// itself still only links ETL, not FreeRTOS::Kernel -- see
/// modules/rtos_core/CMakeLists.txt. Apps register descriptors here during
/// startup so the set of tasks can be validated (no duplicate names,
/// in-range priorities, sane stack sizes) on host; the loop that calls
/// xTaskCreate() once per descriptor is the next step.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "hemerion/rtos_core/task_priority.h"

namespace hemerion::rtos_core
{

/// Maximum number of tasks a TaskRegistry can hold.
inline constexpr std::size_t kMaxTasks = 16;
/// Smallest stack a descriptor may request, in 32-bit words.
inline constexpr std::uint32_t kMinStackWords = 128;

/// Task entry point. Matches FreeRTOS's TaskFunction_t signature so a
/// descriptor's entry_point can be passed to xTaskCreate() unchanged once
/// that step is wired up.
using TaskEntryPoint = void (*)(void*);

/// Result of TaskRegistry::register_task().
enum class TaskRegistryError : std::uint8_t
{
  kNone,             ///< Descriptor accepted.
  kRegistryFull,     ///< Registry already holds kMaxTasks descriptors.
  kDuplicateName,    ///< A descriptor with the same name is already registered.
  kInvalidPriority,  ///< Priority fails is_valid_priority().
  kStackTooSmall,    ///< stack_words is below kMinStackWords.
  kNullEntryPoint,   ///< entry_point (or name) is null.
};

/// Everything needed to later create one FreeRTOS task.
struct TaskDescriptor
{
  const char* name = nullptr;                  ///< Unique, non-null task name; not copied -- must outlive the registry.
  TaskEntryPoint entry_point = nullptr;        ///< Function the task runs; passed to xTaskCreate() unchanged.
  void* argument = nullptr;                    ///< Opaque argument forwarded to entry_point.
  std::uint8_t priority = kPriorityIdle;       ///< Task priority; must satisfy is_valid_priority().
  std::uint32_t stack_words = kMinStackWords;  ///< Stack size in 32-bit words, >= kMinStackWords.
  std::uint32_t period_ms = 0;                 ///< Nominal period [ms]; 0 = event-driven / not periodic.
};

/// @brief Fixed-capacity, statically-allocated table of TaskDescriptor
/// entries, validated as they are registered.
class TaskRegistry
{
public:
  /// @brief Validates and stores `descriptor`.
  ///
  /// Rejects the descriptor (without modifying the registry) if its name is
  /// null/already registered, its entry point is null, its priority is out
  /// of range, its stack is below kMinStackWords, or the registry is full.
  ///
  /// @param descriptor Descriptor to register (copied).
  /// @return TaskRegistryError::kNone on success.
  [[nodiscard]] TaskRegistryError register_task(const TaskDescriptor& descriptor);

  /// Number of descriptors registered so far.
  [[nodiscard]] std::size_t count() const { return count_; }
  /// @brief Descriptor at `index`.
  /// @param index Must be < count(); not bounds-checked.
  [[nodiscard]] const TaskDescriptor& at(std::size_t index) const { return tasks_[index]; }
  /// @brief Looks a descriptor up by task name.
  /// @param name Name to search for (null returns nullptr).
  /// @return The matching descriptor, or nullptr if not registered.
  [[nodiscard]] const TaskDescriptor* find_by_name(const char* name) const;

  /// Removes every registered descriptor.
  void clear() { count_ = 0; }

private:
  std::array<TaskDescriptor, kMaxTasks> tasks_{};
  std::size_t count_ = 0;
};

}  // namespace hemerion::rtos_core
