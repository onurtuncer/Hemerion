# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Host-side simulation targets. Nothing under sim/ is ever cross-compiled -- this directory is only added when
# HEMERION_BUILD_SIM, HEMERION_BUILD_FMU or HEMERION_BUILD_EXAMPLES is ON, all three of which the root CMakeLists.txt
# already guards against CMAKE_CROSSCOMPILING. The FMU/examples cases need spi_shm (and the shm_bridge it builds on):
# the IMU hardware-simulator FMU is an SPI peripheral, not a UDP emitter.
# ------------------------------------------------------------------------------

foreach(
  _hemerion_sim_target
  renode
  fmi
  shm_bridge
  spi_shm
  udp_bridge)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_sim_target}/CMakeLists.txt")
    add_subdirectory(${_hemerion_sim_target})
  endif()
endforeach()
unset(_hemerion_sim_target)
