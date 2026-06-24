# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Host-side simulation targets. Nothing under sim/ is ever cross-compiled --
# this directory is only added when HEMERION_BUILD_SIM is ON, which the root
# CMakeLists.txt already guards against CMAKE_CROSSCOMPILING.
# ------------------------------------------------------------------------------

foreach(_hemerion_sim_target renode fmi shm_bridge udp_bridge)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_sim_target}/CMakeLists.txt")
    add_subdirectory(${_hemerion_sim_target})
  endif()
endforeach()
unset(_hemerion_sim_target)
