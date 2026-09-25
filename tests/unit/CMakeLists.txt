# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Unit tests for host-side code that does not belong to a module.
#
# examples/ cannot be tested where it lives: every example needs an Aetherion plant FMU, and the preset CI runs ctest
# under (test-native) builds no examples at all. So the examples' pure logic -- a formula out of MIL-F-8785C, an option
# parser, and the two GPS receiver presets the documentation quotes -- lives in a header with no FMI, no Ecos and no
# sockets, and is covered here instead. The co-simulation smoke tests stay in examples/, registered only where a plant
# FMU was found, so their absence in CI is visible in the test count rather than scored as a pass.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_example_environment test_example_environment.cpp)
target_include_directories(test_hemerion_example_environment PRIVATE "${CMAKE_SOURCE_DIR}/examples/common")
target_link_libraries(test_hemerion_example_environment PRIVATE hemerion_test_support)

add_test(NAME examples.environment COMMAND test_hemerion_example_environment)
set_tests_properties(examples.environment PROPERTIES LABELS "unit")
