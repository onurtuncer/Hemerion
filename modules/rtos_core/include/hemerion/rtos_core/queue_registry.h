// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file queue_registry.h
/// @brief Central static registry of the inter-module RTOS queues described
/// in modules/README.md.
///
/// "Inter-module communication goes through RTOS queues defined in
/// rtos_core/": producer and consumer modules each register their expected
/// element size and depth under a shared queue name so a mismatch (wrong
/// element size, or two modules picking the same name for different queues)
/// is caught on host; an app wires the matching xQueueCreate() calls once a
/// BSP exists (see task_registry.h for why that step isn't here yet).

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::rtos_core {

/// Maximum number of queues a QueueRegistry can hold.
inline constexpr std::size_t kMaxQueues = 16;

/// Result of QueueRegistry::register_queue().
enum class QueueRegistryError : std::uint8_t {
  kNone,             ///< Descriptor accepted.
  kRegistryFull,     ///< Registry already holds kMaxQueues descriptors.
  kDuplicateName,    ///< A descriptor with the same name is already registered.
  kZeroElementSize,  ///< element_size_bytes is 0.
  kZeroDepth,        ///< depth is 0.
  kNullName,         ///< name is null.
};

/// Everything needed to later create one FreeRTOS queue.
struct QueueDescriptor {
  const char* name = nullptr;          ///< Unique, non-null queue name; not copied -- must outlive the registry.
  std::size_t element_size_bytes = 0;  ///< Size of one queued element [bytes].
  std::size_t depth = 0;               ///< Maximum number of elements the queue holds.
};

/// @brief Fixed-capacity, statically-allocated table of QueueDescriptor
/// entries, validated as they are registered.
class QueueRegistry {
 public:
  /// @brief Validates and stores `descriptor`.
  ///
  /// Rejects the descriptor (without modifying the registry) if its name is
  /// null/already registered, its element size or depth is zero, or the
  /// registry is full.
  ///
  /// @param descriptor Descriptor to register (copied).
  /// @return QueueRegistryError::kNone on success.
  [[nodiscard]] QueueRegistryError register_queue(const QueueDescriptor& descriptor);

  /// Number of descriptors registered so far.
  [[nodiscard]] std::size_t count() const { return count_; }
  /// @brief Descriptor at `index`.
  /// @param index Must be < count(); not bounds-checked.
  [[nodiscard]] const QueueDescriptor& at(std::size_t index) const { return queues_[index]; }
  /// @brief Looks a descriptor up by queue name.
  /// @param name Name to search for (null returns nullptr).
  /// @return The matching descriptor, or nullptr if not registered.
  [[nodiscard]] const QueueDescriptor* find_by_name(const char* name) const;

  /// Removes every registered descriptor.
  void clear() { count_ = 0; }

 private:
  std::array<QueueDescriptor, kMaxQueues> queues_{};
  std::size_t count_ = 0;
};

}  // namespace hemerion::rtos_core
