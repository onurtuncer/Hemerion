// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_shm_segment.cpp
//
// Native unit test for the cross-platform named-segment RAII wrapper. Run by
// CTest under the test-native preset on both Linux and Windows (see
// .github/workflows/linux.yml and windows.yml). Unity is not yet vendored
// (see modules/power/test/test_battery_monitor.cpp), so this uses plain
// asserts and an exit code.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <utility>

#include "hemerion/sim/shm_bridge/shm_segment.h"

using hemerion::sim::shm_bridge::ShmSegment;

namespace {

constexpr std::size_t kSize = 256;

void test_create_then_open_existing_share_bytes() {
  std::optional<ShmSegment> owner = ShmSegment::create("hemerion_test_shm_segment_a", kSize);
  assert(owner.has_value());
  assert(owner->size() == kSize);
  assert(owner->owns_segment());

  std::memset(owner->data(), 0x42, kSize);

  std::optional<ShmSegment> peer = ShmSegment::open_existing("hemerion_test_shm_segment_a", kSize);
  assert(peer.has_value());
  assert(!peer->owns_segment());
  assert(std::memcmp(owner->data(), peer->data(), kSize) == 0);

  // Writes through the peer's view must be visible through the owner's --
  // they map the same physical pages.
  static_cast<std::uint8_t*>(peer->data())[0] = 0x7;
  assert(static_cast<std::uint8_t*>(owner->data())[0] == 0x7);
}

void test_create_fails_if_name_already_exists() {
  std::optional<ShmSegment> first = ShmSegment::create("hemerion_test_shm_segment_b", kSize);
  assert(first.has_value());

  std::optional<ShmSegment> second = ShmSegment::create("hemerion_test_shm_segment_b", kSize);
  assert(!second.has_value());
}

void test_open_existing_fails_if_never_created() {
  std::optional<ShmSegment> peer = ShmSegment::open_existing("hemerion_test_shm_segment_never_created", kSize);
  assert(!peer.has_value());
}

void test_move_construct_transfers_ownership() {
  std::optional<ShmSegment> owner = ShmSegment::create("hemerion_test_shm_segment_c", kSize);
  assert(owner.has_value());
  void* original_data = owner->data();

  ShmSegment moved(std::move(*owner));
  assert(moved.data() == original_data);
  assert(moved.owns_segment());

  // A peer can still attach while the moved-to handle is alive.
  std::optional<ShmSegment> peer = ShmSegment::open_existing("hemerion_test_shm_segment_c", kSize);
  assert(peer.has_value());
}

}  // namespace

int main() {
  test_create_then_open_existing_share_bytes();
  test_create_fails_if_name_already_exists();
  test_open_existing_fails_if_never_created();
  test_move_construct_transfers_ownership();

  std::puts("test_shm_segment: all checks passed");
  return 0;
}
