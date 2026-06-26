# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# FindRenode.cmake
#
# Locates the Renode SWIL simulation platform (https://renode.io), used by tests/swil and sim/renode to run firmware
# without hardware.
#
# Usage: find_package(Renode [1.15] [REQUIRED]) Exposes: Renode_FOUND, Renode_EXECUTABLE, Renode_VERSION, Renode::renode
# (imported EXECUTABLE target, usable as the COMMAND of add_test()/ add_custom_command() entries that drive a .resc
# script).
#
# Search order: RENODE_ROOT / RENODE_HOME environment variables, then PATH, then the well-known install locations for
# each OS -- the antmicro/renode Linux package, the Windows installer's default Program Files directory, and the macOS
# .app bundle.
# ------------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

# Renode runs on the build machine, not the cross-compilation target -- check CMAKE_HOST_<platform>, not
# WIN32/APPLE/UNIX. Those describe the *target* system (CMAKE_SYSTEM_NAME is "Generic" under
# toolchains/arm-none-eabi.cmake, so plain WIN32 is false even on a Windows host) and would silently fall through to the
# Linux path hints below on every cross-compiling preset.
if(CMAKE_HOST_WIN32)
  set(_renode_names renode.exe renode)
  set(_renode_hints
      "$ENV{RENODE_ROOT}"
      "$ENV{RENODE_HOME}"
      "$ENV{ProgramFiles}/Renode"
      "$ENV{ProgramFiles\(x86\)}/Renode")
elseif(CMAKE_HOST_APPLE)
  set(_renode_names renode)
  set(_renode_hints "$ENV{RENODE_ROOT}" "$ENV{RENODE_HOME}" "/Applications/Renode.app/Contents/MacOS")
else()
  set(_renode_names renode)
  set(_renode_hints
      "$ENV{RENODE_ROOT}"
      "$ENV{RENODE_HOME}"
      "/opt/renode"
      "/opt/renode/bin"
      "/usr/local/bin"
      "/usr/bin")
endif()

find_program(
  Renode_EXECUTABLE
  NAMES ${_renode_names}
  HINTS ${_renode_hints}
  PATH_SUFFIXES bin)

unset(_renode_names)
unset(_renode_hints)

if(Renode_EXECUTABLE)
  execute_process(
    COMMAND "${Renode_EXECUTABLE}" --version
    OUTPUT_VARIABLE _renode_version_output
    ERROR_VARIABLE _renode_version_output
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
  # Typical banner: "Renode v1.15.0.12345 (...)" -- pull out the dotted version
  if(_renode_version_output MATCHES "([0-9]+\\.[0-9]+\\.[0-9]+)")
    set(Renode_VERSION "${CMAKE_MATCH_1}")
  endif()
  unset(_renode_version_output)
endif()

find_package_handle_standard_args(Renode REQUIRED_VARS Renode_EXECUTABLE VERSION_VAR Renode_VERSION)

if(Renode_FOUND AND NOT TARGET Renode::renode)
  add_executable(Renode::renode IMPORTED)
  set_target_properties(Renode::renode PROPERTIES IMPORTED_LOCATION "${Renode_EXECUTABLE}")
endif()

mark_as_advanced(Renode_EXECUTABLE)
