// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file tick.h
/// @brief Millisecond/tick conversion with no dependency on FreeRTOSConfig.h.
///
/// No BSP supplies a FreeRTOSConfig.h to this module yet -- see
/// task_registry.h. ms_to_ticks() truncates exactly like FreeRTOS's
/// pdMS_TO_TICKS() macro, so call sites see identical values once a BSP
/// exists and they switch to the real macro.

#pragma once

#include <cstdint>

namespace hemerion::rtos_core
{

/// Default tick rate [Hz]. The STM32H743 Nucleo BSP (see modules/README.md
/// target hardware) is documented to run configTICK_RATE_HZ = 1000; used
/// until a BSP-specific FreeRTOSConfig.h exists to read it from.
inline constexpr std::uint32_t kDefaultTickRateHz = 1000;

/// @brief Converts milliseconds to scheduler ticks, truncating like
/// FreeRTOS's pdMS_TO_TICKS().
/// @param milliseconds Duration to convert [ms].
/// @param tick_rate_hz Scheduler tick rate [Hz].
/// @return Equivalent tick count, truncated toward zero.
[[nodiscard]] constexpr std::uint32_t ms_to_ticks(std::uint32_t milliseconds,
                                                  std::uint32_t tick_rate_hz = kDefaultTickRateHz)
{
  return (milliseconds * tick_rate_hz) / 1000U;
}

/// @brief Converts scheduler ticks to milliseconds.
/// @param ticks        Tick count to convert.
/// @param tick_rate_hz Scheduler tick rate [Hz].
/// @return Equivalent duration [ms], truncated toward zero.
[[nodiscard]] constexpr std::uint32_t ticks_to_ms(std::uint32_t ticks, std::uint32_t tick_rate_hz = kDefaultTickRateHz)
{
  return (ticks * 1000U) / tick_rate_hz;
}

}  // namespace hemerion::rtos_core
