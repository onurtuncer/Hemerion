# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# vendor/ holds git submodules only -- no vendored source is copied into this
# tree. Each submodule that exposes its own CMakeLists.txt is added below;
# submodules not yet checked out are skipped so configure still succeeds.
# ------------------------------------------------------------------------------

foreach(_hemerion_vendor_dep FreeRTOS-Kernel etl fmi4c open_ecat)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_vendor_dep}/CMakeLists.txt")
    add_subdirectory(${_hemerion_vendor_dep})
  endif()
endforeach()
unset(_hemerion_vendor_dep)

# ------------------------------------------------------------------------------
# STM32 CMSIS device headers + HAL drivers. None of these ST submodules ship a
# CMakeLists.txt usable as-is: cmsis-core's is CMSIS-pack-generated and only
# populates its INTERFACE library when CMSIS_Dcore / CMSIS_ENTIRE_* pack
# variables are set, and the device/HAL repos ship no build files at all (they
# are meant to be dropped into STM32CubeIDE or vendored by hand). So this file
# is the one place that turns their raw headers/sources into proper targets.
# ------------------------------------------------------------------------------

# CMSIS::Core -- Cortex-M core headers shared by every STM32 family
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-core/Core/Include")
  add_library(cmsis_core INTERFACE)
  target_include_directories(cmsis_core
                              INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-core/Core/Include")
  add_library(CMSIS::Core ALIAS cmsis_core)
endif()

# CMSIS::STM32H7 / CMSIS::STM32F4 -- per-family device headers + system_*.c templates
foreach(_hemerion_cmsis_device IN ITEMS "cmsis-device-h7:STM32H7" "cmsis-device-f4:STM32F4")
  string(REPLACE ":" ";" _hemerion_cmsis_device ${_hemerion_cmsis_device})
  list(GET _hemerion_cmsis_device 0 _hemerion_cmsis_dir)
  list(GET _hemerion_cmsis_device 1 _hemerion_cmsis_family)
  if(TARGET CMSIS::Core AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_cmsis_dir}/Include")
    add_library(cmsis_device_${_hemerion_cmsis_family} INTERFACE)
    target_include_directories(
      cmsis_device_${_hemerion_cmsis_family}
      INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_cmsis_dir}/Include")
    target_link_libraries(cmsis_device_${_hemerion_cmsis_family} INTERFACE CMSIS::Core)
    add_library(CMSIS::${_hemerion_cmsis_family} ALIAS cmsis_device_${_hemerion_cmsis_family})
  endif()
endforeach()
unset(_hemerion_cmsis_device)
unset(_hemerion_cmsis_dir)
unset(_hemerion_cmsis_family)

# STM32H7xx::HAL / STM32F4xx::HAL -- HAL drivers, built from the vendored Src/.
# A consumer (the BSP) must put its own stm32<family>xx_hal_conf.h on the
# include path -- copied from this submodule's Inc/*_conf_template.h -- ahead
# of linking this target, the same way BSPs supply FreeRTOSConfig.h ahead of
# FreeRTOS::Kernel (see bsp/README.md). *_template.c files are excluded: they
# are scaffolding the BSP is expected to copy and implement, not link as-is.
foreach(_hemerion_hal_driver IN ITEMS "stm32h7xx-hal-driver:STM32H7xx" "stm32f4xx-hal-driver:STM32F4xx")
  string(REPLACE ":" ";" _hemerion_hal_driver ${_hemerion_hal_driver})
  list(GET _hemerion_hal_driver 0 _hemerion_hal_dir)
  list(GET _hemerion_hal_driver 1 _hemerion_hal_family)
  string(REPLACE "xx" "" _hemerion_cmsis_family ${_hemerion_hal_family})
  if(TARGET CMSIS::${_hemerion_cmsis_family} AND EXISTS
                                                 "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_hal_dir}/Src")
    file(GLOB _hemerion_hal_sources "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_hal_dir}/Src/*.c")
    list(FILTER _hemerion_hal_sources EXCLUDE REGEX "_template\\.c$")
    add_library(${_hemerion_hal_dir} STATIC ${_hemerion_hal_sources})
    target_include_directories(${_hemerion_hal_dir}
                                PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_hal_dir}/Inc")
    target_link_libraries(${_hemerion_hal_dir} PUBLIC CMSIS::${_hemerion_cmsis_family})
    target_compile_definitions(${_hemerion_hal_dir} PUBLIC USE_HAL_DRIVER)
    add_library(${_hemerion_hal_family}::HAL ALIAS ${_hemerion_hal_dir})
    unset(_hemerion_hal_sources)
  endif()
endforeach()
unset(_hemerion_hal_driver)
unset(_hemerion_hal_dir)
unset(_hemerion_hal_family)
unset(_hemerion_cmsis_family)
