# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/comms -- CAN framing.
#
# Only the CAN framing layer is implemented so far (see package.xml); the
# EtherCAT slave stack and MAVLink codec described there are not yet built.
# CAN framing has no hardware dependency, so this target only needs ETL --
# cmake/hemerion_module.cmake (the documented hemerion_add_module() helper)
# and cmake/hemerion_hal/can.h do not exist yet, so this file uses plain
# CMake rather than the not-yet-built helper. Migrate to hemerion_add_module()
# once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_comms STATIC src/can_frame.cpp)

target_include_directories(hemerion_comms PUBLIC include)
target_link_libraries(hemerion_comms PUBLIC etl::etl)
target_compile_features(hemerion_comms PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
