// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file regulator_sequencer.cpp
/// @brief Implements the RegulatorSequencer state machine declared in
/// regulator_sequencer.h.

#include "hemerion/power/regulator_sequencer.h"

namespace hemerion::power {

RegulatorSequencerError RegulatorSequencer::configure(const std::uint8_t* rail_ids, std::uint8_t rail_count,
                                                       std::uint32_t confirm_timeout_ms) {
  if (state_ == RegulatorSequencerState::kEnabling) {
    return RegulatorSequencerError::kAlreadyRunning;
  }
  if (rail_count == 0) {
    return RegulatorSequencerError::kNoRails;
  }
  if (rail_count > kMaxRails) {
    return RegulatorSequencerError::kTooManyRails;
  }

  for (std::uint8_t i = 0; i < rail_count; ++i) {
    rail_ids_[i] = rail_ids[i];
  }
  rail_count_ = rail_count;
  confirm_timeout_ms_ = confirm_timeout_ms;
  index_ = 0;
  elapsed_ms_ = 0;
  state_ = RegulatorSequencerState::kIdle;
  return RegulatorSequencerError::kNone;
}

RegulatorSequencerError RegulatorSequencer::start() {
  if (rail_count_ == 0) {
    return RegulatorSequencerError::kNoRails;
  }
  if (state_ == RegulatorSequencerState::kEnabling) {
    return RegulatorSequencerError::kAlreadyRunning;
  }

  index_ = 0;
  elapsed_ms_ = 0;
  state_ = RegulatorSequencerState::kEnabling;
  return RegulatorSequencerError::kNone;
}

RegulatorSequencerState RegulatorSequencer::step(std::uint32_t elapsed_ms, bool rail_power_good) {
  if (state_ != RegulatorSequencerState::kEnabling) {
    return state_;
  }

  if (rail_power_good) {
    ++index_;
    elapsed_ms_ = 0;
    if (index_ >= rail_count_) {
      state_ = RegulatorSequencerState::kComplete;
    }
    return state_;
  }

  elapsed_ms_ += elapsed_ms;
  if (elapsed_ms_ > confirm_timeout_ms_) {
    state_ = RegulatorSequencerState::kFaulted;
  }
  return state_;
}

}  // namespace hemerion::power
