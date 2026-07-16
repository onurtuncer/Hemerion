// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_queue_registry.cpp
//
// Native unit test for the inter-module queue registry. Run by CTest under
// the test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdio>

#include "hemerion/rtos_core/queue_registry.h"

using hemerion::rtos_core::kMaxQueues;
using hemerion::rtos_core::QueueDescriptor;
using hemerion::rtos_core::QueueRegistry;
using hemerion::rtos_core::QueueRegistryError;

namespace
{

void test_register_queue_accepts_valid_descriptor()
{
  QueueRegistry registry;
  QueueDescriptor descriptor;
  descriptor.name = "imu_samples";
  descriptor.element_size_bytes = 32;
  descriptor.depth = 8;

  assert(registry.register_queue(descriptor) == QueueRegistryError::kNone);
  assert(registry.count() == 1);
  assert(registry.find_by_name("imu_samples") != nullptr);
  assert(registry.find_by_name("imu_samples")->depth == 8);
  assert(registry.find_by_name("missing") == nullptr);
}

void test_register_queue_rejects_invalid_descriptors()
{
  QueueRegistry registry;
  QueueDescriptor descriptor;
  descriptor.name = nullptr;
  assert(registry.register_queue(descriptor) == QueueRegistryError::kNullName);

  descriptor.name = "zero_size";
  descriptor.element_size_bytes = 0;
  descriptor.depth = 8;
  assert(registry.register_queue(descriptor) == QueueRegistryError::kZeroElementSize);

  descriptor.element_size_bytes = 32;
  descriptor.depth = 0;
  assert(registry.register_queue(descriptor) == QueueRegistryError::kZeroDepth);
}

void test_register_queue_rejects_duplicate_name()
{
  QueueRegistry registry;
  QueueDescriptor descriptor;
  descriptor.name = "fault_codes";
  descriptor.element_size_bytes = 4;
  descriptor.depth = 4;

  assert(registry.register_queue(descriptor) == QueueRegistryError::kNone);
  assert(registry.register_queue(descriptor) == QueueRegistryError::kDuplicateName);
}

void test_register_queue_rejects_when_full()
{
  QueueRegistry registry;
  QueueDescriptor descriptor;
  descriptor.element_size_bytes = 4;
  descriptor.depth = 4;

  std::array<std::array<char, 2>, kMaxQueues + 1> names{};
  for (std::size_t i = 0; i < kMaxQueues; ++i)
  {
    names[i][0] = static_cast<char>('a' + i);
    descriptor.name = names[i].data();
    assert(registry.register_queue(descriptor) == QueueRegistryError::kNone);
  }

  names[kMaxQueues][0] = 'z';
  descriptor.name = names[kMaxQueues].data();
  assert(registry.register_queue(descriptor) == QueueRegistryError::kRegistryFull);
}

}  // namespace

int main()
{
  test_register_queue_accepts_valid_descriptor();
  test_register_queue_rejects_invalid_descriptors();
  test_register_queue_rejects_duplicate_name();
  test_register_queue_rejects_when_full();

  std::puts("test_queue_registry: all checks passed");
  return 0;
}
