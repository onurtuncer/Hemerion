# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# examples/ -- host-only, end-to-end demonstration programs (co-simulation, tooling). Never cross-compiled: the root
# CMakeLists.txt only reaches this directory when HEMERION_BUILD_EXAMPLES is ON, which it guards against
# CMAKE_CROSSCOMPILING. Same EXISTS-guarded foreach pattern as apps/CMakeLists.txt.
# ------------------------------------------------------------------------------

foreach(_hemerion_example rocket_gps_ecos)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_example}/CMakeLists.txt")
    add_subdirectory("${_hemerion_example}")
  endif()
endforeach()
unset(_hemerion_example)
