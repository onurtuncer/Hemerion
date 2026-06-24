# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apps/can_actuator_link -- minimal consumer of modules/comms.
#
# Links only hemerion_comms for now: cmake/hemerion_hal/can.h and a BSP don't
# exist yet, so there is nothing else to link against. Once a BSP is
# available, this target should also link hemerion_bsp_<board> and
# FreeRTOS::Kernel, and main.cpp should gain task creation + scheduler start
# per apps/README.md's documented layout.
# ------------------------------------------------------------------------------

add_executable(can_actuator_link main.cpp)

target_link_libraries(can_actuator_link PRIVATE hemerion_comms)
target_compile_features(can_actuator_link PRIVATE cxx_std_23)
