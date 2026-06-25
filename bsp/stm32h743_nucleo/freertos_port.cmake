# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Included by the root CMakeLists.txt before add_subdirectory(vendor), once
# per the freertos_config target it creates from this BSP's FreeRTOSConfig.h.
# Single Cortex-M7 core -> the GCC_ARM_CM7 port in vendor/FreeRTOS-Kernel.
# ------------------------------------------------------------------------------

set(FREERTOS_PORT
    GCC_ARM_CM7
    CACHE STRING "FreeRTOS port selected by bsp/stm32h743_nucleo" FORCE)
