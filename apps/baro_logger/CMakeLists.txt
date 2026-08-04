# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apps/baro_logger -- BMP390 barometer logger for bsp/stm32h743_nucleo: the first firmware consumer of the on-target
# Bmp390Driver (modules/sensors/baro/bmp390/), over the BSP's I2C1 HAL and the header-only Bmp390HalI2cBus adapter.
# One FreeRTOS task probes, configures and polls the part, printing compensated samples over the ST-LINK VCP.
#
# Only buildable when that BSP is active, same as apps/led_blink (whose CMakeLists.txt documents the guard).
# ------------------------------------------------------------------------------

if(TARGET hemerion_bsp_stm32h743_nucleo)
  add_executable(baro_logger main.cpp)

  target_link_libraries(baro_logger PRIVATE hemerion_bsp_stm32h743_nucleo hemerion_sensors)
  target_compile_features(baro_logger PRIVATE cxx_std_23)

  if(CMAKE_OBJCOPY)
    add_custom_command(
      TARGET baro_logger
      POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:baro_logger> $<TARGET_FILE_DIR:baro_logger>/baro_logger.hex
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:baro_logger> $<TARGET_FILE_DIR:baro_logger>/baro_logger.bin
      COMMENT "Generating baro_logger.hex / baro_logger.bin")
  endif()
endif()
