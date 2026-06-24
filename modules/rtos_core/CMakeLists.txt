# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# modules/rtos_core -- task registry, queue registry, tick conversion, and a
# static memory pool.
#
# Only those host-testable, FreeRTOS-API-free pieces are implemented so far
# (see package.xml); the xTaskCreate()/xQueueCreate() wiring described there
# is not yet built. Linking FreeRTOS::Kernel needs a BSP-supplied
# freertos_config target and port (see vendor/FreeRTOS-Kernel/CMakeLists.txt),
# and no BSP provides FreeRTOSConfig.h yet, so this target only needs ETL --
# cmake/hemerion_module.cmake (the documented hemerion_add_module() helper)
# does not exist yet, so this file uses plain CMake rather than the
# not-yet-built helper. Migrate to hemerion_add_module() once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_rtos_core STATIC src/task_registry.cpp src/queue_registry.cpp)

target_include_directories(hemerion_rtos_core PUBLIC include)
target_link_libraries(hemerion_rtos_core PUBLIC etl::etl)
target_compile_features(hemerion_rtos_core PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
