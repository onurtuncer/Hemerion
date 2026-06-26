# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for sim/shm_bridge. Only added when HEMERION_BUILD_TESTS is ON (test-native preset) -- see
# sim/shm_bridge/CMakeLists.txt. Both binaries run single-process: they open two handles onto the same named segment to
# exercise the cross-process protocol without spawning a second process.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_shm_bridge_segment test_shm_segment.cpp)
target_link_libraries(test_hemerion_shm_bridge_segment PRIVATE hemerion_shm_bridge)

add_executable(test_hemerion_shm_bridge_bridge test_shm_bridge.cpp)
target_link_libraries(test_hemerion_shm_bridge_bridge PRIVATE hemerion_shm_bridge)

add_test(NAME shm_bridge.segment COMMAND test_hemerion_shm_bridge_segment)
set_tests_properties(shm_bridge.segment PROPERTIES LABELS "unit")

add_test(NAME shm_bridge.bridge COMMAND test_hemerion_shm_bridge_bridge)
set_tests_properties(shm_bridge.bridge PROPERTIES LABELS "unit")
