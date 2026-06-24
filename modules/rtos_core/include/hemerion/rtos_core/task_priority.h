// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/rtos_core/task_priority.h
//
// Canonical priority bands, mirrored from the "Task priority scheme" table in
// apps/README.md so every app's task_config.hpp draws from one definition
// instead of restating the numbers. configMAX_PRIORITIES in the BSP's
// FreeRTOSConfig.h must be at least kPriorityCount.
// ------------------------------------------------------------------------------
#pragma once

#include <cstdint>

namespace hemerion::rtos_core {

enum class TaskPriorityBand : std::uint8_t {
  kIdle,
  kLow,
  kMid,
  kHigh,
  kCritical,
};

inline constexpr std::uint8_t kPriorityIdle = 0;
inline constexpr std::uint8_t kPriorityLowMin = 1;
inline constexpr std::uint8_t kPriorityLowMax = 3;
inline constexpr std::uint8_t kPriorityMidMin = 4;
inline constexpr std::uint8_t kPriorityMidMax = 6;
inline constexpr std::uint8_t kPriorityHighMin = 7;
inline constexpr std::uint8_t kPriorityHighMax = 9;
inline constexpr std::uint8_t kPriorityCritical = 10;
inline constexpr std::uint8_t kPriorityCount = kPriorityCritical + 1;

[[nodiscard]] constexpr bool is_valid_priority(std::uint8_t priority) {
  return priority <= kPriorityCritical;
}

// Out-of-range priorities (priority > kPriorityCritical) classify as
// kCritical; call is_valid_priority() first if rejecting them matters.
[[nodiscard]] constexpr TaskPriorityBand band_of(std::uint8_t priority) {
  if (priority == kPriorityIdle) {
    return TaskPriorityBand::kIdle;
  }
  if (priority <= kPriorityLowMax) {
    return TaskPriorityBand::kLow;
  }
  if (priority <= kPriorityMidMax) {
    return TaskPriorityBand::kMid;
  }
  if (priority <= kPriorityHighMax) {
    return TaskPriorityBand::kHigh;
  }
  return TaskPriorityBand::kCritical;
}

}  // namespace hemerion::rtos_core
