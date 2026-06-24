# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/comms. Only added when HEMERION_BUILD_TESTS is
# ON (test-native preset) -- see modules/comms/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_comms_can_frame test_can_frame.cpp)
target_link_libraries(test_hemerion_comms_can_frame PRIVATE hemerion_comms)

add_test(NAME comms.can_frame COMMAND test_hemerion_comms_can_frame)
set_tests_properties(comms.can_frame PROPERTIES LABELS "unit")
