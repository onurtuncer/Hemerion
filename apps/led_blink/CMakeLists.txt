# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apps/led_blink -- minimal FreeRTOS LED blink demo for bsp/stm32h743_nucleo.
# Doubles as the project's first SWIL example: sim/renode/boards/
# nucleo_h743zi2.repl and tests/swil/test_led_blink.py exercise this exact
# target.
#
# Only buildable when that BSP (and the FreeRTOS::Kernel it wires up) is the
# active one: apps/CMakeLists.txt adds every app/ subdirectory unconditionally,
# but main.cpp hard-includes FreeRTOS.h, so presets selecting a different or
# nonexistent BSP (e.g. HEMERION_BSP=native_linux on test-native/fmu-native)
# must skip this target rather than fail to compile it.
# ------------------------------------------------------------------------------

if(TARGET hemerion_bsp_stm32h743_nucleo)
  add_executable(led_blink main.cpp)

  target_link_libraries(led_blink PRIVATE hemerion_bsp_stm32h743_nucleo)
  target_compile_features(led_blink PRIVATE cxx_std_23)

  if(CMAKE_OBJCOPY)
    add_custom_command(
      TARGET led_blink
      POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:led_blink> $<TARGET_FILE_DIR:led_blink>/led_blink.hex
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:led_blink>
              $<TARGET_FILE_DIR:led_blink>/led_blink.bin
      COMMENT "Generating led_blink.hex / led_blink.bin")
  endif()
endif()
