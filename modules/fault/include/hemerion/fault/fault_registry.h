// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/fault/fault_registry.h
//
// Static fault code registry: codes must be registered up front (no dynamic
// allocation), then raised/cleared by other modules as conditions are
// detected. This is a pure bookkeeping layer -- it has no opinion on how a
// fault is detected (that lives in the owning module, e.g. power's
// evaluate_faults()) or on what happens when one is active (that is the
// health monitor's job, not yet built). A later health monitor polls
// highest_active_severity() and decides whether to trip the watchdog-backed
// safe state.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::fault {

inline constexpr std::size_t kMaxFaultCodes = 64;

enum class FaultSeverity : std::uint8_t {
  kInfo,
  kWarning,
  kCritical,
};

enum class FaultRegistryError : std::uint8_t {
  kNone,
  kRegistryFull,
  kDuplicateCode,
  kUnknownCode,
};

// Fixed-capacity table mapping fault codes to severity and active/cleared
// state. Codes are arbitrary caller-defined std::uint16_t values; this class
// does not assign them.
class FaultRegistry {
 public:
  // Adds `code` to the registry with the given severity. Rejected if `code`
  // is already registered or the registry is at kMaxFaultCodes capacity.
  [[nodiscard]] FaultRegistryError register_code(std::uint16_t code, FaultSeverity severity);

  // Marks a previously registered `code` as active. Idempotent: raising an
  // already-active code is not an error.
  [[nodiscard]] FaultRegistryError raise(std::uint16_t code);

  // Marks a previously registered `code` as inactive. Idempotent: clearing
  // an already-inactive code is not an error.
  [[nodiscard]] FaultRegistryError clear(std::uint16_t code);

  // Returns false for both inactive and never-registered codes.
  [[nodiscard]] bool is_active(std::uint16_t code) const;

  [[nodiscard]] std::uint8_t active_count() const;

  // Returns the highest severity among currently active codes, or
  // FaultSeverity::kInfo if none are active.
  [[nodiscard]] FaultSeverity highest_active_severity() const;

 private:
  struct Entry {
    std::uint16_t code = 0;
    FaultSeverity severity = FaultSeverity::kInfo;
    bool registered = false;
    bool active = false;
  };

  [[nodiscard]] Entry* find(std::uint16_t code);
  [[nodiscard]] const Entry* find(std::uint16_t code) const;

  std::array<Entry, kMaxFaultCodes> entries_{};
  std::uint8_t registered_count_ = 0;
};

}  // namespace hemerion::fault
