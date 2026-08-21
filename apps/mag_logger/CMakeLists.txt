# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apps/mag_logger -- MMC5983MA magnetometer logger for bsp/stm32h743_nucleo: the first firmware consumer of the
# on-target Mmc5983maDriver (modules/sensors/mag/mmc5983ma/), over the BSP's I2C1 HAL and the header-only
# Mmc5983maHalI2cBus adapter. One FreeRTOS task probes the part, cancels its bridge offset with the datasheet's
# SET/RESET pair, and polls it, printing samples in nanotesla over the ST-LINK VCP.
#
# Only buildable when that BSP is active, same as apps/led_blink (whose CMakeLists.txt documents the guard).
# ------------------------------------------------------------------------------

if(TARGET hemerion_bsp_stm32h743_nucleo)
  add_executable(mag_logger main.cpp)

  target_link_libraries(mag_logger PRIVATE hemerion_bsp_stm32h743_nucleo hemerion_sensors)
  target_compile_features(mag_logger PRIVATE cxx_std_23)

  if(CMAKE_OBJCOPY)
    add_custom_command(
      TARGET mag_logger
      POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:mag_logger> $<TARGET_FILE_DIR:mag_logger>/mag_logger.hex
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:mag_logger> $<TARGET_FILE_DIR:mag_logger>/mag_logger.bin
      COMMENT "Generating mag_logger.hex / mag_logger.bin")
  endif()
endif()
