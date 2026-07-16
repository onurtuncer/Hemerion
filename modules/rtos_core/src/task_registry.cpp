// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file task_registry.cpp
/// @brief Implements the TaskRegistry validation logic declared in
/// task_registry.h.

#include "hemerion/rtos_core/task_registry.h"

#include <cstring>

namespace hemerion::rtos_core
{

TaskRegistryError TaskRegistry::register_task(const TaskDescriptor& descriptor)
{
  if (descriptor.name == nullptr || descriptor.entry_point == nullptr)
  {
    return TaskRegistryError::kNullEntryPoint;
  }
  if (!is_valid_priority(descriptor.priority))
  {
    return TaskRegistryError::kInvalidPriority;
  }
  if (descriptor.stack_words < kMinStackWords)
  {
    return TaskRegistryError::kStackTooSmall;
  }
  if (find_by_name(descriptor.name) != nullptr)
  {
    return TaskRegistryError::kDuplicateName;
  }
  if (count_ >= kMaxTasks)
  {
    return TaskRegistryError::kRegistryFull;
  }

  tasks_[count_] = descriptor;
  ++count_;
  return TaskRegistryError::kNone;
}

const TaskDescriptor* TaskRegistry::find_by_name(const char* name) const
{
  for (std::size_t i = 0; i < count_; ++i)
  {
    if (std::strcmp(tasks_[i].name, name) == 0)
    {
      return &tasks_[i];
    }
  }
  return nullptr;
}

}  // namespace hemerion::rtos_core
