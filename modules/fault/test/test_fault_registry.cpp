// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_fault_registry.cpp
//
// Native unit test for the fault code registry. Run by CTest under the
// test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/fault/fault_registry.h"

using hemerion::fault::FaultRegistry;
using hemerion::fault::FaultRegistryError;
using hemerion::fault::FaultSeverity;
using hemerion::fault::kMaxFaultCodes;

namespace {

void test_unregistered_code_is_not_active() {
  FaultRegistry registry;
  assert(!registry.is_active(1));
  assert(registry.active_count() == 0);
}

void test_register_then_raise_makes_code_active() {
  FaultRegistry registry;
  assert(registry.register_code(1, FaultSeverity::kWarning) == FaultRegistryError::kNone);
  assert(!registry.is_active(1));
  assert(registry.raise(1) == FaultRegistryError::kNone);
  assert(registry.is_active(1));
  assert(registry.active_count() == 1);
}

void test_clear_makes_code_inactive() {
  FaultRegistry registry;
  assert(registry.register_code(1, FaultSeverity::kCritical) == FaultRegistryError::kNone);
  assert(registry.raise(1) == FaultRegistryError::kNone);
  assert(registry.clear(1) == FaultRegistryError::kNone);
  assert(!registry.is_active(1));
  assert(registry.active_count() == 0);
}

void test_raise_and_clear_are_idempotent() {
  FaultRegistry registry;
  assert(registry.register_code(1, FaultSeverity::kInfo) == FaultRegistryError::kNone);
  assert(registry.clear(1) == FaultRegistryError::kNone);
  assert(registry.raise(1) == FaultRegistryError::kNone);
  assert(registry.raise(1) == FaultRegistryError::kNone);
  assert(registry.active_count() == 1);
}

void test_duplicate_registration_rejected() {
  FaultRegistry registry;
  assert(registry.register_code(1, FaultSeverity::kInfo) == FaultRegistryError::kNone);
  assert(registry.register_code(1, FaultSeverity::kWarning) == FaultRegistryError::kDuplicateCode);
}

void test_unknown_code_operations_rejected() {
  FaultRegistry registry;
  assert(registry.raise(42) == FaultRegistryError::kUnknownCode);
  assert(registry.clear(42) == FaultRegistryError::kUnknownCode);
}

void test_registry_full_rejects_further_registration() {
  FaultRegistry registry;
  for (std::uint16_t code = 0; code < static_cast<std::uint16_t>(kMaxFaultCodes); ++code) {
    assert(registry.register_code(code, FaultSeverity::kInfo) == FaultRegistryError::kNone);
  }
  assert(registry.register_code(static_cast<std::uint16_t>(kMaxFaultCodes), FaultSeverity::kInfo) ==
         FaultRegistryError::kRegistryFull);
}

void test_highest_active_severity_tracks_active_set() {
  FaultRegistry registry;
  assert(registry.register_code(1, FaultSeverity::kWarning) == FaultRegistryError::kNone);
  assert(registry.register_code(2, FaultSeverity::kCritical) == FaultRegistryError::kNone);
  assert(registry.highest_active_severity() == FaultSeverity::kInfo);

  assert(registry.raise(1) == FaultRegistryError::kNone);
  assert(registry.highest_active_severity() == FaultSeverity::kWarning);

  assert(registry.raise(2) == FaultRegistryError::kNone);
  assert(registry.highest_active_severity() == FaultSeverity::kCritical);

  assert(registry.clear(2) == FaultRegistryError::kNone);
  assert(registry.highest_active_severity() == FaultSeverity::kWarning);
}

}  // namespace

int main() {
  test_unregistered_code_is_not_active();
  test_register_then_raise_makes_code_active();
  test_clear_makes_code_inactive();
  test_raise_and_clear_are_idempotent();
  test_duplicate_registration_rejected();
  test_unknown_code_operations_rejected();
  test_registry_full_rejects_further_registration();
  test_highest_active_severity_tracks_active_set();

  std::puts("test_fault_registry: all checks passed");
  return 0;
}
