# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/power. Only added when HEMERION_BUILD_TESTS is
# ON (test-native preset) -- see modules/power/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_power_battery_monitor test_battery_monitor.cpp)
target_link_libraries(test_hemerion_power_battery_monitor PRIVATE hemerion_power)

add_executable(test_hemerion_power_regulator_sequencer test_regulator_sequencer.cpp)
target_link_libraries(test_hemerion_power_regulator_sequencer PRIVATE hemerion_power)

add_test(NAME power.battery_monitor COMMAND test_hemerion_power_battery_monitor)
set_tests_properties(power.battery_monitor PROPERTIES LABELS "unit")

add_test(NAME power.regulator_sequencer COMMAND test_hemerion_power_regulator_sequencer)
set_tests_properties(power.regulator_sequencer PROPERTIES LABELS "unit")
