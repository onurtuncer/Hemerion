// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fault_registry.h
/// @brief Static fault code registry.
///
/// Codes must be registered up front (no dynamic allocation), then
/// raised/cleared by other modules as conditions are detected. This is a
/// pure bookkeeping layer -- it has no opinion on how a fault is detected
/// (that lives in the owning module, e.g. power's evaluate_faults()) or on
/// what happens when one is active (that is the health monitor's job, not
/// yet built). A later health monitor polls highest_active_severity() and
/// decides whether to trip the watchdog-backed safe state.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// @namespace hemerion::fault
/// @brief Fault bookkeeping and software watchdog supervision.
namespace hemerion::fault {

/// Maximum number of fault codes a FaultRegistry can hold.
inline constexpr std::size_t kMaxFaultCodes = 64;

/// Severity assigned to a fault code at registration time.
enum class FaultSeverity : std::uint8_t {
  kInfo,      ///< Informational; no action expected.
  kWarning,   ///< Degraded but operable.
  kCritical,  ///< Requires the health monitor to act (e.g. safe state).
};

/// Result of FaultRegistry operations.
enum class FaultRegistryError : std::uint8_t {
  kNone,           ///< Accepted.
  kRegistryFull,   ///< Registry already holds kMaxFaultCodes codes.
  kDuplicateCode,  ///< register_code() called with an already-registered code.
  kUnknownCode,    ///< raise()/clear() called with a never-registered code.
};

/// @brief Fixed-capacity table mapping fault codes to severity and
/// active/cleared state.
///
/// Codes are arbitrary caller-defined std::uint16_t values; this class does
/// not assign them.
class FaultRegistry {
 public:
  /// @brief Adds `code` to the registry with the given severity.
  /// @param code     Caller-defined fault code.
  /// @param severity Severity reported while `code` is active.
  /// @return kDuplicateCode if `code` is already registered, kRegistryFull
  ///         at kMaxFaultCodes capacity, else FaultRegistryError::kNone.
  [[nodiscard]] FaultRegistryError register_code(std::uint16_t code, FaultSeverity severity);

  /// @brief Marks a previously registered `code` as active.
  ///
  /// Idempotent: raising an already-active code is not an error.
  ///
  /// @param code Registered fault code.
  /// @return kUnknownCode if `code` was never registered, else kNone.
  [[nodiscard]] FaultRegistryError raise(std::uint16_t code);

  /// @brief Marks a previously registered `code` as inactive.
  ///
  /// Idempotent: clearing an already-inactive code is not an error.
  ///
  /// @param code Registered fault code.
  /// @return kUnknownCode if `code` was never registered, else kNone.
  [[nodiscard]] FaultRegistryError clear(std::uint16_t code);

  /// @brief Whether `code` is currently active.
  /// @return False for both inactive and never-registered codes.
  [[nodiscard]] bool is_active(std::uint16_t code) const;

  /// Number of currently active codes.
  [[nodiscard]] std::uint8_t active_count() const;

  /// @brief Highest severity among currently active codes.
  /// @return FaultSeverity::kInfo if none are active.
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
