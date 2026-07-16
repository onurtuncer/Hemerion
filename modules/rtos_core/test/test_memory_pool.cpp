// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_memory_pool.cpp
//
// Native unit test for the static memory pool. Run by CTest under the
// test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/rtos_core/memory_pool.h"

using hemerion::rtos_core::MemoryPool;

namespace
{

struct Payload
{
  std::uint32_t value = 0;
};

void test_acquire_release_round_trip()
{
  MemoryPool<Payload, 2> pool;
  assert(pool.capacity() == 2);
  assert(pool.size() == 0);
  assert(!pool.full());

  Payload* a = pool.acquire();
  assert(a != nullptr);
  a->value = 42;
  assert(pool.size() == 1);

  Payload* b = pool.acquire();
  assert(b != nullptr);
  assert(pool.full());

  Payload* c = pool.acquire();
  assert(c == nullptr);  // Capacity 2, already full.

  pool.release(a);
  assert(!pool.full());
  assert(pool.size() == 1);

  Payload* d = pool.acquire();
  assert(d != nullptr);
  assert(pool.full());

  pool.release(b);
  pool.release(d);
  assert(pool.size() == 0);
}

}  // namespace

int main()
{
  test_acquire_release_round_trip();

  std::puts("test_memory_pool: all checks passed");
  return 0;
}
