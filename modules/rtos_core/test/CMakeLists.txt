# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/rtos_core. Only added when HEMERION_BUILD_TESTS
# is ON (test-native preset) -- see modules/rtos_core/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_rtos_core_task_registry test_task_registry.cpp)
target_link_libraries(test_hemerion_rtos_core_task_registry PRIVATE hemerion_rtos_core)

add_executable(test_hemerion_rtos_core_queue_registry test_queue_registry.cpp)
target_link_libraries(test_hemerion_rtos_core_queue_registry PRIVATE hemerion_rtos_core)

add_executable(test_hemerion_rtos_core_memory_pool test_memory_pool.cpp)
target_link_libraries(test_hemerion_rtos_core_memory_pool PRIVATE hemerion_rtos_core)

add_test(NAME rtos_core.task_registry COMMAND test_hemerion_rtos_core_task_registry)
set_tests_properties(rtos_core.task_registry PROPERTIES LABELS "unit")

add_test(NAME rtos_core.queue_registry COMMAND test_hemerion_rtos_core_queue_registry)
set_tests_properties(rtos_core.queue_registry PROPERTIES LABELS "unit")

add_test(NAME rtos_core.memory_pool COMMAND test_hemerion_rtos_core_memory_pool)
set_tests_properties(rtos_core.memory_pool PROPERTIES LABELS "unit")
