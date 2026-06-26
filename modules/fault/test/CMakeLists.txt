# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/fault. Only added when HEMERION_BUILD_TESTS is
# ON (test-native preset) -- see modules/fault/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_fault_registry test_fault_registry.cpp)
target_link_libraries(test_hemerion_fault_registry PRIVATE hemerion_fault)

add_executable(test_hemerion_fault_watchdog test_watchdog.cpp)
target_link_libraries(test_hemerion_fault_watchdog PRIVATE hemerion_fault)

add_test(NAME fault.registry COMMAND test_hemerion_fault_registry)
set_tests_properties(fault.registry PROPERTIES LABELS "unit")

add_test(NAME fault.watchdog COMMAND test_hemerion_fault_watchdog)
set_tests_properties(fault.watchdog PROPERTIES LABELS "unit")
