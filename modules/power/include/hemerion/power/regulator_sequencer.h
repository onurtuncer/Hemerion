// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file regulator_sequencer.h
/// @brief Ordered regulator enable sequencing.
///
/// This is a pure state machine: it has no dependency on a GPIO/regulator
/// peripheral or the HAL abstraction (cmake/hemerion_hal/ has no power.h
/// yet), so the caller drives it by calling step() once per scheduler tick
/// and reporting whether the rail currently being enabled has signalled
/// power-good (e.g. from a HAL-backed power-good pin read). A later
/// HAL-backed driver owns that read and the actual rail enable GPIO writes.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::power {

/// Maximum number of rails a RegulatorSequencer can be configured with.
inline constexpr std::size_t kMaxRails = 8;

/// Result of RegulatorSequencer::configure() / start().
enum class RegulatorSequencerError : std::uint8_t {
  kNone,            ///< Accepted.
  kNoRails,         ///< configure() called with rail_count == 0 or null rail_ids.
  kTooManyRails,    ///< configure() called with rail_count > kMaxRails.
  kAlreadyRunning,  ///< Rejected because a sequence is currently in progress.
};

/// Lifecycle state of a RegulatorSequencer, as returned by state() / step().
enum class RegulatorSequencerState : std::uint8_t {
  kIdle,      ///< Configured (or freshly constructed) but not started.
  kEnabling,  ///< A rail is being enabled; awaiting its power-good confirmation.
  kComplete,  ///< Every configured rail confirmed power-good in order.
  kFaulted,   ///< A rail failed to confirm power-good within the timeout.
};

/// @brief Sequences a fixed, ordered list of regulator rail IDs, enabling one
/// rail at a time and waiting for its power-good confirmation (within a
/// configured timeout) before moving on to the next.
class RegulatorSequencer {
 public:
  /// @brief Configures the ordered rail IDs to sequence and the per-rail
  /// power-good confirmation timeout.
  ///
  /// Rejected while a sequence is in progress.
  ///
  /// @param rail_ids           Ordered rail identifiers to enable; copied internally.
  /// @param rail_count         Number of entries in `rail_ids`, 1..kMaxRails.
  /// @param confirm_timeout_ms Per-rail power-good confirmation timeout [ms].
  /// @return RegulatorSequencerError::kNone on success.
  [[nodiscard]] RegulatorSequencerError configure(const std::uint8_t* rail_ids, std::uint8_t rail_count,
                                                   std::uint32_t confirm_timeout_ms);

  /// @brief Starts (or restarts, once idle/complete/faulted) sequencing from
  /// the first configured rail.
  /// @return RegulatorSequencerError::kNone on success,
  ///         kAlreadyRunning while state() == kEnabling,
  ///         kNoRails if configure() has not succeeded yet.
  [[nodiscard]] RegulatorSequencerError start();

  /// @brief Advances the sequencer by `elapsed_ms` and reports whether the
  /// rail currently being enabled has signalled power-good.
  ///
  /// No-op (returns the current state unchanged) unless
  /// state() == kEnabling. Call current_rail_id() afterwards while the
  /// result is kEnabling to learn which rail to (re)check next.
  ///
  /// @param elapsed_ms      Time elapsed since the previous step() call [ms].
  /// @param rail_power_good True if the rail currently being enabled reports power-good.
  /// @return The sequencer state after this step.
  RegulatorSequencerState step(std::uint32_t elapsed_ms, bool rail_power_good);

  /// Current lifecycle state.
  [[nodiscard]] RegulatorSequencerState state() const { return state_; }

  /// @brief ID of the rail currently being enabled.
  /// @return Valid only while state() == kEnabling; returns rail_ids[0] otherwise.
  [[nodiscard]] std::uint8_t current_rail_id() const {
    const std::uint8_t safe_index = index_ < rail_count_ ? index_ : static_cast<std::uint8_t>(0);
    return rail_ids_[safe_index];
  }

 private:
  std::array<std::uint8_t, kMaxRails> rail_ids_{};
  std::uint8_t rail_count_ = 0;
  std::uint8_t index_ = 0;
  std::uint32_t confirm_timeout_ms_ = 0;
  std::uint32_t elapsed_ms_ = 0;
  RegulatorSequencerState state_ = RegulatorSequencerState::kIdle;
};

}  // namespace hemerion::power
