# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# arm-none-eabi.cmake
#
# Toolchain file for Cortex-M targets (STM32H7, STM32F4) built with the GNU Arm Embedded ("arm-none-eabi")
# cross-compiler, version 12 or newer. Selected by the cross-base preset family in CMakePresets.json via
# "toolchainFile".
#
# Only baseline flags that apply to every Cortex-M variant live here. MCU- specific flags (-mcpu, -mfpu, -mfloat-abi)
# belong on the BSP's interface target -- see bsp/README.md -- so this single file covers all variants.
# ------------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

add_compile_options(
  -ffunction-sections
  -fdata-sections
  -fno-exceptions
  -fno-rtti
  -Wall
  -Wextra)
add_link_options(-Wl,--gc-sections -Wl,--print-memory-usage --specs=nosys.specs)
