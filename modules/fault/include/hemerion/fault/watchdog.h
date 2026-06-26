// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/fault/watchdog.h
//
// Software deadline supervisor: tracks a fixed set of caller-defined
// channels (e.g. one per RTOS task), each with its own timeout, and reports
// whether any channel has gone without a kick() for longer than its
// timeout. This is a pure state machine, driven by step() once per
// scheduler tick -- it has no dependency on a hardware watchdog peripheral
// or the HAL abstraction (cmake/hemerion_hal/ has no fault.h yet). A later
// HAL-backed driver feeds the actual IWDG only while step() reports
// WatchdogStatus::kOk across all channels.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::fault {

inline constexpr std::size_t kMaxWatchdogChannels = 8;

enum class WatchdogError : std::uint8_t {
  kNone,
  kRegistryFull,
  kDuplicateId,
  kUnknownId,
};

enum class WatchdogStatus : std::uint8_t {
  kOk,
  kExpired,
};

// Supervises a fixed set of channels, each expected to call kick() at least
// once per `timeout_ms`. Call step() once per scheduler tick with the
// elapsed time since the previous call.
class WatchdogSupervisor {
 public:
  // Adds a channel identified by `id` with the given timeout. Rejected if
  // `id` is already registered or the supervisor is at
  // kMaxWatchdogChannels capacity. A newly registered channel starts with a
  // full timeout budget, as if just kicked.
  [[nodiscard]] WatchdogError register_channel(std::uint8_t id, std::uint32_t timeout_ms);

  // Resets the elapsed time for `id` back to zero.
  [[nodiscard]] WatchdogError kick(std::uint8_t id);

  // Advances every registered channel's elapsed time by `elapsed_ms`.
  // Returns WatchdogStatus::kExpired if any channel's elapsed time has
  // reached or exceeded its timeout (after this step), else
  // WatchdogStatus::kOk. A channel that has expired stays expired until
  // kicked, even across further step() calls.
  WatchdogStatus step(std::uint32_t elapsed_ms);

  // Returns false for both unexpired and never-registered channels.
  [[nodiscard]] bool is_expired(std::uint8_t id) const;

 private:
  struct Channel {
    std::uint8_t id = 0;
    std::uint32_t timeout_ms = 0;
    std::uint32_t elapsed_ms = 0;
    bool registered = false;
    bool expired = false;
  };

  [[nodiscard]] Channel* find(std::uint8_t id);
  [[nodiscard]] const Channel* find(std::uint8_t id) const;

  std::array<Channel, kMaxWatchdogChannels> channels_{};
  std::uint8_t registered_count_ = 0;
};

}  // namespace hemerion::fault
