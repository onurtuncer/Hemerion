// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/rtos_core/tick.h
//
// Millisecond/tick conversion with no dependency on FreeRTOSConfig.h (no BSP
// supplies one yet -- see task_registry.h). ms_to_ticks() truncates exactly
// like FreeRTOS's pdMS_TO_TICKS() macro, so call sites see identical values
// once a BSP exists and they switch to the real macro.
// ------------------------------------------------------------------------------
#pragma once

#include <cstdint>

namespace hemerion::rtos_core {

// STM32H743 Nucleo BSP (see modules/README.md target hardware) is documented
// to run configTICK_RATE_HZ = 1000; used as the default tick rate until a
// BSP-specific FreeRTOSConfig.h exists to read it from.
inline constexpr std::uint32_t kDefaultTickRateHz = 1000;

[[nodiscard]] constexpr std::uint32_t ms_to_ticks(std::uint32_t milliseconds,
                                                   std::uint32_t tick_rate_hz = kDefaultTickRateHz) {
  return (milliseconds * tick_rate_hz) / 1000U;
}

[[nodiscard]] constexpr std::uint32_t ticks_to_ms(std::uint32_t ticks,
                                                   std::uint32_t tick_rate_hz = kDefaultTickRateHz) {
  return (ticks * 1000U) / tick_rate_hz;
}

}  // namespace hemerion::rtos_core
