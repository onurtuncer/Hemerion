# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/sensors -- IMU raw-to-SI unit conversion, GPS NMEA/UBX parsing.
#
# The IMU raw-count-to-SI-unit conversion and the GPS NMEA/UBX parsers + driver are implemented; the barometer
# driver is not yet built. include/Hemerion/gps/fmu/ is a GPS-specific UBX hardware simulator FMU (truth state in,
# noisy UBX-NAV-PVT bytes over UDP out) -- see its CMakeLists.txt; this is not the generic whole-module
# numeric-variable FMU export package.xml's <artifact type="fmu"> describes, which remains unbuilt. None of this
# has a hardware dependency, so this target needs no peripheral/bus libraries -- cmake/hemerion_module.cmake (the
# documented hemerion_add_module() helper) does not exist yet, so this file uses plain CMake rather than the
# not-yet-built helper. Migrate to hemerion_add_module() once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_sensors STATIC src/imu/imu_conversion.cpp src/gps/nmeaParser.cpp src/gps/ubxParser.cpp)

target_include_directories(hemerion_sensors PUBLIC include)
target_compile_features(hemerion_sensors PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()

if(HEMERION_BUILD_FMU AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/Hemerion/gps/fmu/CMakeLists.txt")
  add_subdirectory(include/Hemerion/gps/fmu)
endif()
