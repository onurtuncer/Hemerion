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
