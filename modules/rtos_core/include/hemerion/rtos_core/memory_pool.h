// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file memory_pool.h
/// @brief Fixed-capacity, statically-allocated object pool for RTOS message
/// payloads that must outlive a single queue send.
///
/// A queue itself only moves fixed-size handles/pointers around. Wraps
/// etl::pool so callers never see ETL's no-pool-available assertion:
/// acquire() reports failure by returning nullptr instead, matching the
/// [[nodiscard]] error-code convention used across this module.

#pragma once

#include <cstddef>

#include "etl/pool.h"

namespace hemerion::rtos_core {

/// @brief Statically-allocated pool of default-constructed T objects.
///
/// @tparam T        Object type stored in the pool.
/// @tparam Capacity Maximum number of simultaneously acquired objects.
template <typename T, std::size_t Capacity>
class MemoryPool {
 public:
  /// @brief Acquires one object from the pool.
  /// @return Pointer to a pool-owned object, or nullptr without allocating
  ///         if the pool is already full.
  [[nodiscard]] T* acquire() {
    if (pool_.full()) {
      return nullptr;
    }
    return pool_.allocate();
  }

  /// @brief Returns an object to the pool.
  /// @param item Must have come from a prior acquire() on this pool.
  void release(T* item) { pool_.release(item); }

  /// Number of objects currently acquired.
  [[nodiscard]] std::size_t size() const { return pool_.size(); }
  /// True if every slot is acquired.
  [[nodiscard]] bool full() const { return pool_.full(); }
  /// Compile-time maximum number of objects.
  [[nodiscard]] static constexpr std::size_t capacity() { return Capacity; }

 private:
  etl::pool<T, Capacity> pool_;
};

}  // namespace hemerion::rtos_core
