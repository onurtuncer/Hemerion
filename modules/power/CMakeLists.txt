# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/power -- BMS telemetry fault evaluation and regulator enable sequencing.
#
# Only those two pure-logic pieces are implemented so far (see package.xml); the actual BMS/regulator peripheral driver
# is not yet built. Both pieces have no hardware dependency, so this target only needs the standard library --
# cmake/hemerion_module.cmake (the documented hemerion_add_module() helper) and cmake/hemerion_hal/power.h do not exist
# yet, so this file uses plain CMake rather than the not-yet-built helper. Migrate to hemerion_add_module() once it
# lands.
# ------------------------------------------------------------------------------

add_library(hemerion_power STATIC src/battery_monitor.cpp src/regulator_sequencer.cpp)

target_include_directories(hemerion_power PUBLIC include)
target_compile_features(hemerion_power PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
