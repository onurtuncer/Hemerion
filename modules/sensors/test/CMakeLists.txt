# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for modules/sensors. Only added when HEMERION_BUILD_TESTS is ON (test-native preset) -- see
# modules/sensors/CMakeLists.txt.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_sensors_imu_conversion test_imu_conversion.cpp)
target_link_libraries(test_hemerion_sensors_imu_conversion PRIVATE hemerion_sensors)

add_test(NAME sensors.imu_conversion COMMAND test_hemerion_sensors_imu_conversion)
set_tests_properties(sensors.imu_conversion PROPERTIES LABELS "unit")

add_executable(test_hemerion_sensors_nmea_parser test_nmea_parser.cpp)
target_link_libraries(test_hemerion_sensors_nmea_parser PRIVATE hemerion_sensors)

add_test(NAME sensors.nmea_parser COMMAND test_hemerion_sensors_nmea_parser)
set_tests_properties(sensors.nmea_parser PROPERTIES LABELS "unit")

add_executable(test_hemerion_sensors_ubx_parser test_ubx_parser.cpp)
target_link_libraries(test_hemerion_sensors_ubx_parser PRIVATE hemerion_sensors)

add_test(NAME sensors.ubx_parser COMMAND test_hemerion_sensors_ubx_parser)
set_tests_properties(sensors.ubx_parser PROPERTIES LABELS "unit")

add_executable(test_hemerion_sensors_gps_driver test_gps_driver.cpp)
target_link_libraries(test_hemerion_sensors_gps_driver PRIVATE hemerion_sensors)

add_test(NAME sensors.gps_driver COMMAND test_hemerion_sensors_gps_driver)
set_tests_properties(sensors.gps_driver PROPERTIES LABELS "unit")

# Round-trip test for the GPS FMU simulator (gps/fmu/). ubxEmitter.cpp is compiled directly into this test binary rather
# than via hemerion_sensors, since the emitter is sim-only code gated behind HEMERION_BUILD_FMU in
# gps/fmu/CMakeLists.txt and must not be linked into cross-compiled firmware.
add_executable(test_hemerion_sensors_ubx_emitter test_ubx_emitter.cpp
                                                 "${CMAKE_CURRENT_SOURCE_DIR}/../src/gps/fmu/ubxEmitter.cpp")
target_link_libraries(test_hemerion_sensors_ubx_emitter PRIVATE hemerion_sensors)

add_test(NAME sensors.ubx_emitter COMMAND test_hemerion_sensors_ubx_emitter)
set_tests_properties(sensors.ubx_emitter PROPERTIES LABELS "unit")

# Round-trip test for the IMU FMU simulator (imu/fmu/). imu_packet_emitter.cpp is compiled directly into this test
# binary rather than via hemerion_sensors, since the emitter is sim-only code gated behind HEMERION_BUILD_FMU in
# imu/fmu/CMakeLists.txt and must not be linked into cross-compiled firmware.
add_executable(test_hemerion_sensors_imu_packet test_imu_packet.cpp
                                                "${CMAKE_CURRENT_SOURCE_DIR}/../src/imu/fmu/imu_packet_emitter.cpp")
target_link_libraries(test_hemerion_sensors_imu_packet PRIVATE hemerion_sensors)

add_test(NAME sensors.imu_packet COMMAND test_hemerion_sensors_imu_packet)
set_tests_properties(sensors.imu_packet PROPERTIES LABELS "unit")

# Round-trip tests for the barometer, radar altimeter and magnetometer FMU simulators (baro/fmu/, radalt/fmu/,
# mag/fmu/). Like ubxEmitter.cpp and imu_packet_emitter.cpp above, each emitter is compiled directly into its test
# binary rather than via hemerion_sensors, since the emitters are sim-only code gated behind HEMERION_BUILD_FMU and
# must not be linked into cross-compiled firmware.
add_executable(test_hemerion_sensors_baro_packet test_baro_packet.cpp
                                                 "${CMAKE_CURRENT_SOURCE_DIR}/../src/baro/fmu/baro_packet_emitter.cpp")
target_link_libraries(test_hemerion_sensors_baro_packet PRIVATE hemerion_sensors)

add_test(NAME sensors.baro_packet COMMAND test_hemerion_sensors_baro_packet)
set_tests_properties(sensors.baro_packet PROPERTIES LABELS "unit")

add_executable(
  test_hemerion_sensors_radalt_packet test_radalt_packet.cpp
                                      "${CMAKE_CURRENT_SOURCE_DIR}/../src/radalt/fmu/radalt_packet_emitter.cpp")
target_link_libraries(test_hemerion_sensors_radalt_packet PRIVATE hemerion_sensors)

add_test(NAME sensors.radalt_packet COMMAND test_hemerion_sensors_radalt_packet)
set_tests_properties(sensors.radalt_packet PROPERTIES LABELS "unit")

add_executable(test_hemerion_sensors_mag_packet test_mag_packet.cpp
                                                "${CMAKE_CURRENT_SOURCE_DIR}/../src/mag/fmu/mag_packet_emitter.cpp")
target_link_libraries(test_hemerion_sensors_mag_packet PRIVATE hemerion_sensors)

add_test(NAME sensors.mag_packet COMMAND test_hemerion_sensors_mag_packet)
set_tests_properties(sensors.mag_packet PROPERTIES LABELS "unit")
