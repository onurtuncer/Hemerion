// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/rtos_core/queue_registry.h
//
// Central static registry of the inter-module RTOS queues described in
// modules/README.md ("inter-module communication goes through RTOS queues
// defined in rtos_core/"). Producer and consumer modules each register their
// expected element size and depth under a shared queue name so a mismatch
// (wrong element size, or two modules picking the same name for different
// queues) is caught on host; an app wires the matching xQueueCreate() calls
// once a BSP exists (see task_registry.h for why that step isn't here yet).
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::rtos_core {

inline constexpr std::size_t kMaxQueues = 16;

enum class QueueRegistryError : std::uint8_t {
  kNone,
  kRegistryFull,
  kDuplicateName,
  kZeroElementSize,
  kZeroDepth,
  kNullName,
};

struct QueueDescriptor {
  const char* name = nullptr;
  std::size_t element_size_bytes = 0;
  std::size_t depth = 0;
};

class QueueRegistry {
 public:
  // Rejects the descriptor (without modifying the registry) if its name is
  // null/already registered, its element size or depth is zero, or the
  // registry is full.
  [[nodiscard]] QueueRegistryError register_queue(const QueueDescriptor& descriptor);

  [[nodiscard]] std::size_t count() const { return count_; }
  [[nodiscard]] const QueueDescriptor& at(std::size_t index) const { return queues_[index]; }
  [[nodiscard]] const QueueDescriptor* find_by_name(const char* name) const;

  void clear() { count_ = 0; }

 private:
  std::array<QueueDescriptor, kMaxQueues> queues_{};
  std::size_t count_ = 0;
};

}  // namespace hemerion::rtos_core
