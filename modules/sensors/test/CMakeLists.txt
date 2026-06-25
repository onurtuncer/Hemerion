# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/sensors. Only added when HEMERION_BUILD_TESTS
# is ON (test-native preset) -- see modules/sensors/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_sensors_imu_conversion test_imu_conversion.cpp)
target_link_libraries(test_hemerion_sensors_imu_conversion PRIVATE hemerion_sensors)

add_test(NAME sensors.imu_conversion COMMAND test_hemerion_sensors_imu_conversion)
set_tests_properties(sensors.imu_conversion PROPERTIES LABELS "unit")
