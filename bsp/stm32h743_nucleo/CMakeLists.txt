# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# bsp/stm32h743_nucleo -- NUCLEO-H743ZI2 board support package (see
# package.xml and bsp/README.md). An INTERFACE library: it contributes the
# MCU compile/link flags, linker script, include paths, and startup/clock/HAL
# sources to whatever app links it, but holds no compiled objects of its own.
#
# cmake/hemerion_hal/ (the shared, board-agnostic HAL contract declarations)
# does not exist yet -- see cmake/README.md -- so include/hemerion/hal/ here
# both declares and implements gpio.h/uart.h/board.h for now. Migrate to
# consuming the shared contract once it lands.
# ------------------------------------------------------------------------------

add_library(hemerion_bsp_stm32h743_nucleo INTERFACE)

target_compile_definitions(hemerion_bsp_stm32h743_nucleo INTERFACE STM32H743xx USE_HAL_DRIVER)

# Cortex-M7 with double-precision FPU; toolchains/arm-none-eabi.cmake covers
# only the baseline flags shared by every Cortex-M variant (see its README).
target_compile_options(
  hemerion_bsp_stm32h743_nucleo INTERFACE -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb)
target_link_options(
  hemerion_bsp_stm32h743_nucleo INTERFACE
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -T${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32h743zi_flash.ld)

target_include_directories(hemerion_bsp_stm32h743_nucleo
                            INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_sources(
  hemerion_bsp_stm32h743_nucleo
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/startup_stm32h743xx.s"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/system_stm32h7xx.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/board.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/gpio.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/uart.c")

target_link_libraries(hemerion_bsp_stm32h743_nucleo INTERFACE CMSIS::STM32H7 STM32H7xx::HAL)

# FREERTOS_PORT/freertos_config are set up by the root CMakeLists.txt (from
# this BSP's freertos_port.cmake) before vendor/FreeRTOS-Kernel is even added,
# so FreeRTOS::Kernel already exists by the time this file runs.
if(TARGET FreeRTOS::Kernel)
  target_link_libraries(hemerion_bsp_stm32h743_nucleo INTERFACE FreeRTOS::Kernel)
endif()
