# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Cross-cutting integration and SWIL tests. Per-module unit tests live inside
# each module's own test/ subdirectory instead. Categories not yet
# implemented are skipped so configure still succeeds.
# ------------------------------------------------------------------------------

foreach(_hemerion_test_dir unit integration swil fmu)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_test_dir}/CMakeLists.txt")
    add_subdirectory(${_hemerion_test_dir})
  endif()
endforeach()
unset(_hemerion_test_dir)
