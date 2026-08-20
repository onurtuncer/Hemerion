# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for sim/i2c_shm. Only added when HEMERION_BUILD_TESTS is ON (test-native preset) -- see
# sim/i2c_shm/CMakeLists.txt. The binary runs single-process: it opens both ends of the same named bus to exercise the
# handshake and the peripheral service thread without spawning a second process.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_i2c_shm_link test_i2c_shm_link.cpp)
target_link_libraries(test_hemerion_i2c_shm_link PRIVATE hemerion_i2c_shm)

add_test(NAME i2c_shm.link COMMAND test_hemerion_i2c_shm_link)
set_tests_properties(i2c_shm.link PROPERTIES LABELS "unit")

# The full BMP390 SWIL chain minus only the FMI wrapper: the real on-target driver polling the simulated part across
# this bus. The BMP390 sources are compiled in directly rather than linked from hemerion_sensors, so this test does not
# order sim/ against modules/ -- it depends only on this directory's target and the sensors *sources*, shared with
# tools/ as HEMERION_I2C_SHM_BMP390_SOURCES. The on-target driver is named here rather than there because this is the
# only target that needs it.
add_executable(test_hemerion_i2c_shm_bmp390 test_i2c_shm_bmp390.cpp ${HEMERION_I2C_SHM_BMP390_SOURCES}
                                            "${HEMERION_I2C_SHM_SENSORS_DIR}/src/baro/bmp390/bmp390_driver.cpp")
target_include_directories(test_hemerion_i2c_shm_bmp390 PRIVATE "${HEMERION_I2C_SHM_SENSORS_DIR}/include")
target_link_libraries(test_hemerion_i2c_shm_bmp390 PRIVATE hemerion_i2c_shm)

add_test(NAME i2c_shm.bmp390 COMMAND test_hemerion_i2c_shm_bmp390)
set_tests_properties(i2c_shm.bmp390 PROPERTIES LABELS "unit")
