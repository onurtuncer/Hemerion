// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/rtos_core/memory_pool.h
//
// Fixed-capacity, statically-allocated object pool for RTOS message payloads
// that must outlive a single queue send (a queue itself only moves fixed-size
// handles/pointers around). Wraps etl::pool so callers never see ETL's
// no-pool-available assertion: acquire() reports failure by returning
// nullptr instead, matching the [[nodiscard]] error-code convention used
// across this module.
// ------------------------------------------------------------------------------
#pragma once

#include <cstddef>

#include "etl/pool.h"

namespace hemerion::rtos_core {

template <typename T, std::size_t Capacity>
class MemoryPool {
 public:
  // Returns nullptr without allocating if the pool is already full.
  [[nodiscard]] T* acquire() {
    if (pool_.full()) {
      return nullptr;
    }
    return pool_.allocate();
  }

  // `item` must have come from a prior acquire() on this pool.
  void release(T* item) { pool_.release(item); }

  [[nodiscard]] std::size_t size() const { return pool_.size(); }
  [[nodiscard]] bool full() const { return pool_.full(); }
  [[nodiscard]] static constexpr std::size_t capacity() { return Capacity; }

 private:
  etl::pool<T, Capacity> pool_;
};

}  // namespace hemerion::rtos_core
