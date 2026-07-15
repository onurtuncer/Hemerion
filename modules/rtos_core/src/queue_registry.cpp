// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file queue_registry.cpp
/// @brief Implements the QueueRegistry validation logic declared in
/// queue_registry.h.

#include "hemerion/rtos_core/queue_registry.h"

#include <cstring>

namespace hemerion::rtos_core {

QueueRegistryError QueueRegistry::register_queue(const QueueDescriptor& descriptor) {
  if (descriptor.name == nullptr) {
    return QueueRegistryError::kNullName;
  }
  if (descriptor.element_size_bytes == 0) {
    return QueueRegistryError::kZeroElementSize;
  }
  if (descriptor.depth == 0) {
    return QueueRegistryError::kZeroDepth;
  }
  if (find_by_name(descriptor.name) != nullptr) {
    return QueueRegistryError::kDuplicateName;
  }
  if (count_ >= kMaxQueues) {
    return QueueRegistryError::kRegistryFull;
  }

  queues_[count_] = descriptor;
  ++count_;
  return QueueRegistryError::kNone;
}

const QueueDescriptor* QueueRegistry::find_by_name(const char* name) const {
  for (std::size_t i = 0; i < count_; ++i) {
    if (std::strcmp(queues_[i].name, name) == 0) {
      return &queues_[i];
    }
  }
  return nullptr;
}

}  // namespace hemerion::rtos_core
