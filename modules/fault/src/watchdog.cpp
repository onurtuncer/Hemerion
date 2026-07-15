// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file watchdog.cpp
/// @brief Implements the WatchdogSupervisor deadline tracking declared in
/// watchdog.h.

#include "hemerion/fault/watchdog.h"

namespace hemerion::fault {

WatchdogSupervisor::Channel* WatchdogSupervisor::find(std::uint8_t id) {
  for (auto& channel : channels_) {
    if (channel.registered && channel.id == id) {
      return &channel;
    }
  }
  return nullptr;
}

const WatchdogSupervisor::Channel* WatchdogSupervisor::find(std::uint8_t id) const {
  for (const auto& channel : channels_) {
    if (channel.registered && channel.id == id) {
      return &channel;
    }
  }
  return nullptr;
}

WatchdogError WatchdogSupervisor::register_channel(std::uint8_t id, std::uint32_t timeout_ms) {
  if (find(id) != nullptr) {
    return WatchdogError::kDuplicateId;
  }
  if (registered_count_ >= kMaxWatchdogChannels) {
    return WatchdogError::kRegistryFull;
  }
  Channel& channel = channels_[registered_count_];
  channel.id = id;
  channel.timeout_ms = timeout_ms;
  channel.elapsed_ms = 0;
  channel.registered = true;
  channel.expired = false;
  ++registered_count_;
  return WatchdogError::kNone;
}

WatchdogError WatchdogSupervisor::kick(std::uint8_t id) {
  Channel* channel = find(id);
  if (channel == nullptr) {
    return WatchdogError::kUnknownId;
  }
  channel->elapsed_ms = 0;
  channel->expired = false;
  return WatchdogError::kNone;
}

WatchdogStatus WatchdogSupervisor::step(std::uint32_t elapsed_ms) {
  WatchdogStatus status = WatchdogStatus::kOk;
  for (auto& channel : channels_) {
    if (!channel.registered) {
      continue;
    }
    channel.elapsed_ms += elapsed_ms;
    if (channel.elapsed_ms >= channel.timeout_ms) {
      channel.expired = true;
    }
    if (channel.expired) {
      status = WatchdogStatus::kExpired;
    }
  }
  return status;
}

bool WatchdogSupervisor::is_expired(std::uint8_t id) const {
  const Channel* channel = find(id);
  return channel != nullptr && channel->expired;
}

}  // namespace hemerion::fault
