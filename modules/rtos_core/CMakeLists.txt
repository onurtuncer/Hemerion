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
# is not yet built. bsp/stm32h743_nucleo now supplies the freertos_config/
# FREERTOS_PORT wiring FreeRTOS::Kernel needs (see
# vendor/FreeRTOS-Kernel/CMakeLists.txt), but this target still only needs
# ETL until that wiring lands here too. cmake/hemerion_module.cmake (the
# documented hemerion_add_module() helper) does not exist yet, so this file
# uses plain CMake rather than the not-yet-built helper. Migrate to
# hemerion_add_module() once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_rtos_core STATIC src/task_registry.cpp src/queue_registry.cpp)

target_include_directories(hemerion_rtos_core PUBLIC include)
target_link_libraries(hemerion_rtos_core PUBLIC etl::etl)
target_compile_features(hemerion_rtos_core PUBLIC cxx_std_23)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
