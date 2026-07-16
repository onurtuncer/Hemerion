// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_regulator_sequencer.cpp
//
// Native unit test for the regulator enable sequencing state machine. Run by
// CTest under the test-native preset. Unity (referenced in
// modules/README.md as the project's test framework) is not yet vendored,
// so this uses plain asserts and an exit code; switch to Unity test cases
// once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/power/regulator_sequencer.h"

using hemerion::power::kMaxRails;
using hemerion::power::RegulatorSequencer;
using hemerion::power::RegulatorSequencerError;
using hemerion::power::RegulatorSequencerState;

namespace
{

void test_sequences_rails_in_order_on_confirmation()
{
  RegulatorSequencer sequencer;
  const std::array<std::uint8_t, 3> rails = { 5, 1, 7 };
  assert(sequencer.configure(rails.data(), 3, 100) == RegulatorSequencerError::kNone);
  assert(sequencer.start() == RegulatorSequencerError::kNone);
  assert(sequencer.state() == RegulatorSequencerState::kEnabling);
  assert(sequencer.current_rail_id() == 5);

  assert(sequencer.step(10, true) == RegulatorSequencerState::kEnabling);
  assert(sequencer.current_rail_id() == 1);

  assert(sequencer.step(10, true) == RegulatorSequencerState::kEnabling);
  assert(sequencer.current_rail_id() == 7);

  assert(sequencer.step(10, true) == RegulatorSequencerState::kComplete);
}

void test_times_out_when_rail_never_confirms()
{
  RegulatorSequencer sequencer;
  const std::array<std::uint8_t, 1> rails = { 2 };
  assert(sequencer.configure(rails.data(), 1, 50) == RegulatorSequencerError::kNone);
  assert(sequencer.start() == RegulatorSequencerError::kNone);

  assert(sequencer.step(30, false) == RegulatorSequencerState::kEnabling);
  assert(sequencer.step(30, false) == RegulatorSequencerState::kFaulted);
}

void test_configure_rejects_zero_rails()
{
  RegulatorSequencer sequencer;
  assert(sequencer.configure(nullptr, 0, 100) == RegulatorSequencerError::kNoRails);
}

void test_configure_rejects_too_many_rails()
{
  RegulatorSequencer sequencer;
  std::array<std::uint8_t, kMaxRails + 1> rails{};
  assert(sequencer.configure(rails.data(), static_cast<std::uint8_t>(kMaxRails + 1), 100) ==
         RegulatorSequencerError::kTooManyRails);
}

void test_start_rejects_reentry_while_running()
{
  RegulatorSequencer sequencer;
  const std::array<std::uint8_t, 1> rails = { 3 };
  assert(sequencer.configure(rails.data(), 1, 100) == RegulatorSequencerError::kNone);
  assert(sequencer.start() == RegulatorSequencerError::kNone);
  assert(sequencer.start() == RegulatorSequencerError::kAlreadyRunning);
}

}  // namespace

int main()
{
  test_sequences_rails_in_order_on_confirmation();
  test_times_out_when_rail_never_confirms();
  test_configure_rejects_zero_rails();
  test_configure_rejects_too_many_rails();
  test_start_rejects_reentry_while_running();

  std::puts("test_regulator_sequencer: all checks passed");
  return 0;
}
