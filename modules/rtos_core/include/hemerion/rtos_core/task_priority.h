// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file task_priority.h
/// @brief Canonical task priority bands.
///
/// Mirrored from the "Task priority scheme" table in apps/README.md so every
/// app's task_config.hpp draws from one definition instead of restating the
/// numbers. configMAX_PRIORITIES in the BSP's FreeRTOSConfig.h must be at
/// least kPriorityCount.

#pragma once

#include <cstdint>

/// @namespace hemerion::rtos_core
/// @brief RTOS-facing primitives: task/queue registries, priority bands,
/// tick conversion, and static memory pools.
namespace hemerion::rtos_core
{

/// Named priority band a numeric priority falls into; see band_of().
enum class TaskPriorityBand : std::uint8_t
{
  kIdle,      ///< Priority kPriorityIdle (0).
  kLow,       ///< Priorities kPriorityLowMin..kPriorityLowMax.
  kMid,       ///< Priorities kPriorityMidMin..kPriorityMidMax.
  kHigh,      ///< Priorities kPriorityHighMin..kPriorityHighMax.
  kCritical,  ///< Priority kPriorityCritical (and any out-of-range value).
};

inline constexpr std::uint8_t kPriorityIdle = 0;       ///< Idle band (FreeRTOS idle task level).
inline constexpr std::uint8_t kPriorityLowMin = 1;     ///< Lowest priority in the low band.
inline constexpr std::uint8_t kPriorityLowMax = 3;     ///< Highest priority in the low band.
inline constexpr std::uint8_t kPriorityMidMin = 4;     ///< Lowest priority in the mid band.
inline constexpr std::uint8_t kPriorityMidMax = 6;     ///< Highest priority in the mid band.
inline constexpr std::uint8_t kPriorityHighMin = 7;    ///< Lowest priority in the high band.
inline constexpr std::uint8_t kPriorityHighMax = 9;    ///< Highest priority in the high band.
inline constexpr std::uint8_t kPriorityCritical = 10;  ///< Single critical (highest) priority.
/// Number of distinct priorities; lower bound for configMAX_PRIORITIES.
inline constexpr std::uint8_t kPriorityCount = kPriorityCritical + 1;

/// @brief True if `priority` lies within the defined scheme
/// (0..kPriorityCritical).
[[nodiscard]] constexpr bool is_valid_priority(std::uint8_t priority) { return priority <= kPriorityCritical; }

/// @brief Maps a numeric priority to its band.
///
/// Out-of-range priorities (priority > kPriorityCritical) classify as
/// kCritical; call is_valid_priority() first if rejecting them matters.
///
/// @param priority Numeric task priority.
/// @return The band `priority` falls into.
[[nodiscard]] constexpr TaskPriorityBand band_of(std::uint8_t priority)
{
  if (priority == kPriorityIdle)
  {
    return TaskPriorityBand::kIdle;
  }
  if (priority <= kPriorityLowMax)
  {
    return TaskPriorityBand::kLow;
  }
  if (priority <= kPriorityMidMax)
  {
    return TaskPriorityBand::kMid;
  }
  if (priority <= kPriorityHighMax)
  {
    return TaskPriorityBand::kHigh;
  }
  return TaskPriorityBand::kCritical;
}

}  // namespace hemerion::rtos_core
