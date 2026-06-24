# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Top-level firmware executables. Each app links one or more modules with the
# active BSP. Listed explicitly per app/README.md; apps not yet implemented
# are skipped so configure still succeeds.
# ------------------------------------------------------------------------------

foreach(_hemerion_app gnc_flight ecs_gateway datalink)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_hemerion_app}/CMakeLists.txt")
    add_subdirectory(${_hemerion_app})
  endif()
endforeach()
unset(_hemerion_app)
