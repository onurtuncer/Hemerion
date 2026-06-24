# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# FindAetherion.cmake
#
# Usage: find_package(Aetherion REQUIRED) Exposes: Aetherion::Aetherion (imported target), Aetherion_FOUND,
# Aetherion_INCLUDE_DIR, Aetherion_LIBRARY
#
# Aetherion installs (via AETHERION_INSTALL=ON) its own AetherionConfig.cmake, which is the authoritative description of
# the package -- it wires up Eigen3::Eigen, CppAD::cppad and the generateFMU() helper alongside Aetherion::Aetherion.
# This module first defers to that Config package when it is discoverable on CMAKE_PREFIX_PATH/PATH. It only falls back
# to locating raw headers/libraries (e.g. a manually copied install, with no dependency wiring) when no Config package
# can be found, so that `find_package(Aetherion)` still works in that degraded case on both Windows and Linux.
# ------------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

# ── Prefer the real Config package, if one can be located ──────────────────────
find_package(
  Aetherion
  ${Aetherion_FIND_VERSION}
  QUIET
  CONFIG)

if(Aetherion_FOUND)
  find_package_handle_standard_args(Aetherion CONFIG_MODE)
  return()
endif()

# ── Fallback: locate a bare install with no exported Config package ────────────
if(WIN32)
  set(_aetherion_hints
      "$ENV{AETHERION_ROOT}"
      "$ENV{ProgramFiles}/Aetherion"
      "$ENV{ProgramFiles(x86)}/Aetherion"
      "C:/Program Files/Aetherion")
else()
  set(_aetherion_hints
      "$ENV{AETHERION_ROOT}"
      "/usr/local"
      "/usr"
      "/opt/aetherion")
endif()

find_path(
  Aetherion_INCLUDE_DIR
  NAMES Aetherion/Aetherion.h
  HINTS ${_aetherion_hints}
  PATH_SUFFIXES include)

find_library(
  Aetherion_LIBRARY
  NAMES Aetherion
  HINTS ${_aetherion_hints}
  PATH_SUFFIXES lib lib64)

unset(_aetherion_hints)

find_package_handle_standard_args(Aetherion REQUIRED_VARS Aetherion_LIBRARY Aetherion_INCLUDE_DIR)

if(Aetherion_FOUND AND NOT TARGET Aetherion::Aetherion)
  add_library(Aetherion::Aetherion UNKNOWN IMPORTED)
  set_target_properties(Aetherion::Aetherion PROPERTIES IMPORTED_LOCATION "${Aetherion_LIBRARY}"
                                                        INTERFACE_INCLUDE_DIRECTORIES "${Aetherion_INCLUDE_DIR}")

  message(STATUS "[FindAetherion] Located a bare Aetherion install with no exported "
                 "AetherionConfig.cmake -- Eigen3, CppAD and the generateFMU() helper are NOT wired up. "
                 "Re-install Aetherion with -DAETHERION_INSTALL=ON for full functionality.")
endif()

mark_as_advanced(Aetherion_INCLUDE_DIR Aetherion_LIBRARY)
