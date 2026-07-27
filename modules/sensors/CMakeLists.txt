# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/sensors -- IMU/baro/radalt/mag raw-to-SI unit conversion + Hemerion packet parsers, GPS NMEA/UBX parsing.
#
# The raw-count-to-SI-unit conversions and Hemerion wire-protocol packet parsers for the IMU, barometer, radar altimeter
# and magnetometer, plus the GPS NMEA/UBX parsers + driver, are implemented. include/Hemerion/<sensor>/fmu/ subtrees are
# per-sensor hardware simulator FMUs (truth state in, noisy raw sensor bytes over UDP out): gps/fmu/ emits UBX-NAV-PVT,
# and imu/fmu/, baro/fmu/, radalt/fmu/ and mag/fmu/ emit the matching Hemerion raw-sample frames -- see their
# CMakeLists.txt; none of them is the generic whole-module numeric-variable FMU export package.xml's <artifact
# type="fmu"> describes, which remains unbuilt. None of this has a hardware dependency, so this target needs no
# peripheral/bus libraries -- cmake/hemerion_module.cmake (the documented hemerion_add_module() helper) does not exist
# yet, so this file uses plain CMake rather than the not-yet-built helper. Migrate to hemerion_add_module() once it
# lands.
# ------------------------------------------------------------------------------

add_library(
  hemerion_sensors STATIC
  src/imu/imu_conversion.cpp
  src/imu/imu_packet.cpp
  src/baro/baro_conversion.cpp
  src/baro/baro_packet.cpp
  src/radalt/radalt_conversion.cpp
  src/radalt/radalt_packet.cpp
  src/mag/mag_conversion.cpp
  src/mag/mag_packet.cpp
  src/gps/nmeaParser.cpp
  src/gps/ubxParser.cpp)

target_include_directories(hemerion_sensors PUBLIC include)
target_compile_features(hemerion_sensors PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()

# ------------------------------------------------------------------------------
# Per-sensor hardware simulator FMUs. Each subdirectory is one generateFMU() call; the socket libraries are collected
# here because all five emit their raw sensor bytes over the same UdpSender, and only Windows needs an explicit link.
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_FMU)
  if(WIN32)
    set(HEMERION_FMU_SOCKET_LIBRARIES ws2_32)
  else()
    set(HEMERION_FMU_SOCKET_LIBRARIES "")
  endif()

  foreach(
    _hemerion_sensor_fmu
    gps
    imu
    baro
    radalt
    mag)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/Hemerion/${_hemerion_sensor_fmu}/fmu/CMakeLists.txt")
      add_subdirectory("include/Hemerion/${_hemerion_sensor_fmu}/fmu")
    endif()
  endforeach()
  unset(_hemerion_sensor_fmu)
endif()
