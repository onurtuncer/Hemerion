# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/fault -- fault code registry and software watchdog supervisor.
#
# Only those two pure-logic pieces are implemented so far (see package.xml); the health monitor's HAL-backed inputs
# (BSP-fed sensor/peripheral faults) and the hardware IWDG feed are not yet built. Both pieces have no hardware or RTOS
# dependency, so this target only needs the standard library -- cmake/hemerion_module.cmake (the documented
# hemerion_add_module() helper) and cmake/hemerion_hal/fault.h do not exist yet, so this file uses plain CMake rather
# than the not-yet-built helper. Migrate to hemerion_add_module() once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_fault STATIC src/fault_registry.cpp src/watchdog.cpp)

target_include_directories(hemerion_fault PUBLIC include)
target_compile_features(hemerion_fault PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
