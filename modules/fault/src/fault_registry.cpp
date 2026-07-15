// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fault_registry.cpp
/// @brief Implements the FaultRegistry bookkeeping declared in
/// fault_registry.h.

#include "hemerion/fault/fault_registry.h"

namespace hemerion::fault {

FaultRegistry::Entry* FaultRegistry::find(std::uint16_t code) {
  for (auto& entry : entries_) {
    if (entry.registered && entry.code == code) {
      return &entry;
    }
  }
  return nullptr;
}

const FaultRegistry::Entry* FaultRegistry::find(std::uint16_t code) const {
  for (const auto& entry : entries_) {
    if (entry.registered && entry.code == code) {
      return &entry;
    }
  }
  return nullptr;
}

FaultRegistryError FaultRegistry::register_code(std::uint16_t code, FaultSeverity severity) {
  if (find(code) != nullptr) {
    return FaultRegistryError::kDuplicateCode;
  }
  if (registered_count_ >= kMaxFaultCodes) {
    return FaultRegistryError::kRegistryFull;
  }
  Entry& entry = entries_[registered_count_];
  entry.code = code;
  entry.severity = severity;
  entry.registered = true;
  entry.active = false;
  ++registered_count_;
  return FaultRegistryError::kNone;
}

FaultRegistryError FaultRegistry::raise(std::uint16_t code) {
  Entry* entry = find(code);
  if (entry == nullptr) {
    return FaultRegistryError::kUnknownCode;
  }
  entry->active = true;
  return FaultRegistryError::kNone;
}

FaultRegistryError FaultRegistry::clear(std::uint16_t code) {
  Entry* entry = find(code);
  if (entry == nullptr) {
    return FaultRegistryError::kUnknownCode;
  }
  entry->active = false;
  return FaultRegistryError::kNone;
}

bool FaultRegistry::is_active(std::uint16_t code) const {
  const Entry* entry = find(code);
  return entry != nullptr && entry->active;
}

std::uint8_t FaultRegistry::active_count() const {
  std::uint8_t count = 0;
  for (const auto& entry : entries_) {
    if (entry.registered && entry.active) {
      ++count;
    }
  }
  return count;
}

FaultSeverity FaultRegistry::highest_active_severity() const {
  FaultSeverity highest = FaultSeverity::kInfo;
  for (const auto& entry : entries_) {
    if (entry.registered && entry.active && entry.severity > highest) {
      highest = entry.severity;
    }
  }
  return highest;
}

}  // namespace hemerion::fault
