# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Included by the root CMakeLists.txt before add_subdirectory(vendor), once per the freertos_config target it creates
# from this BSP's FreeRTOSConfig.h. Single Cortex-M7 core -> the GCC_ARM_CM7 port in vendor/FreeRTOS-Kernel.
#
# This is also the only hook that runs before vendor/ is processed, so it is where this board's MCU-wide compile
# settings get applied globally: vendor's static libraries (stm32h7xx-hal-driver, freertos_kernel) compile their own
# sources immediately when add_subdirectory(vendor) runs, before any consumer exists to link against -- the INTERFACE
# compile options/definitions/include dirs on hemerion_bsp_stm32h743_nucleo (bsp/stm32h743_nucleo/CMakeLists.txt) only
# reach targets that *link* that BSP target, which vendor's libraries never do. Without -mthumb specifically, GCC
# defaults to classic ARM mode and fails to assemble Thumb-2-only intrinsics (DSB/ISB/CPSID) that HAL_CORTEX and the
# FreeRTOS port use.
# ------------------------------------------------------------------------------

set(FREERTOS_PORT GCC_ARM_CM7 CACHE STRING "FreeRTOS port selected by bsp/stm32h743_nucleo" FORCE)

set(FREERTOS_HEAP 4 CACHE STRING "FreeRTOS heap implementation selected by bsp/stm32h743_nucleo" FORCE)

add_compile_definitions(STM32H743xx)
add_compile_options(
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb)
include_directories("${CMAKE_CURRENT_LIST_DIR}/include")
