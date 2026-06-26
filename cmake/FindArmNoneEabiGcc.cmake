# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# FindArmNoneEabiGcc.cmake
#
# Locates the GNU Arm Embedded ("arm-none-eabi") cross-compiler toolchain used to build firmware for the Cortex-M
# targets under bsp/.
#
# This module is a diagnostic step, not the mechanism that selects the compiler: toolchains/arm-none-eabi.cmake sets
# CMAKE_<LANG>_COMPILER directly, because CMAKE_MODULE_PATH (and therefore this module) is not yet available while
# CMAKE_TOOLCHAIN_FILE is being processed. Call find_package(ArmNoneEabiGcc 12) from the root CMakeLists.txt once
# CMAKE_CROSSCOMPILING is known, so a missing or too-old toolchain fails with a clear message instead of a raw
# compiler-test error.
#
# Usage: find_package(ArmNoneEabiGcc [12] [REQUIRED]) Exposes: ArmNoneEabiGcc_FOUND    - TRUE if both gcc and g++ were
# located ArmNoneEabiGcc_VERSION  - dotted gcc version string ArmNoneEabiGcc_GCC      - path to arm-none-eabi-gcc
# ArmNoneEabiGcc_GXX      - path to arm-none-eabi-g++ ArmNoneEabiGcc_OBJCOPY  - path to arm-none-eabi-objcopy (optional)
# ArmNoneEabiGcc_SIZE     - path to arm-none-eabi-size (optional) ArmNoneEabiGcc_GDB      - path to arm-none-eabi-gdb
# (optional) ArmNoneEabiGcc::gcc / ::gxx / ::objcopy / ::size / ::gdb - imported EXECUTABLE targets for whichever of the
# above were found
# ------------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

# This toolchain runs on the build machine, not the cross-compilation target -- check CMAKE_HOST_WIN32, not WIN32. WIN32
# describes the *target* system (CMAKE_SYSTEM_NAME is "Generic" here, so plain WIN32 is false even on a Windows host)
# and would silently fall through to the Linux path hints below.
if(CMAKE_HOST_WIN32)
  set(_arm_gcc_hints "$ENV{ARM_NONE_EABI_ROOT}" "$ENV{ProgramFiles}/Arm GNU Toolchain arm-none-eabi"
                     "$ENV{ProgramFiles\(x86\)}/GNU Arm Embedded Toolchain")
else()
  set(_arm_gcc_hints
      "$ENV{ARM_NONE_EABI_ROOT}"
      "/usr"
      "/usr/local"
      "/opt/gcc-arm-none-eabi")
endif()

find_program(
  ArmNoneEabiGcc_GCC
  NAMES arm-none-eabi-gcc
  HINTS ${_arm_gcc_hints}
  PATH_SUFFIXES bin)
find_program(
  ArmNoneEabiGcc_GXX
  NAMES arm-none-eabi-g++
  HINTS ${_arm_gcc_hints}
  PATH_SUFFIXES bin)
find_program(
  ArmNoneEabiGcc_OBJCOPY
  NAMES arm-none-eabi-objcopy
  HINTS ${_arm_gcc_hints}
  PATH_SUFFIXES bin)
find_program(
  ArmNoneEabiGcc_SIZE
  NAMES arm-none-eabi-size
  HINTS ${_arm_gcc_hints}
  PATH_SUFFIXES bin)
find_program(
  ArmNoneEabiGcc_GDB
  NAMES arm-none-eabi-gdb
  HINTS ${_arm_gcc_hints}
  PATH_SUFFIXES bin)

unset(_arm_gcc_hints)

if(ArmNoneEabiGcc_GCC)
  execute_process(COMMAND "${ArmNoneEabiGcc_GCC}" -dumpfullversion OUTPUT_VARIABLE ArmNoneEabiGcc_VERSION
                  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

find_package_handle_standard_args(ArmNoneEabiGcc REQUIRED_VARS ArmNoneEabiGcc_GCC ArmNoneEabiGcc_GXX
                                  VERSION_VAR ArmNoneEabiGcc_VERSION)

if(ArmNoneEabiGcc_GCC AND NOT TARGET ArmNoneEabiGcc::gcc)
  add_executable(ArmNoneEabiGcc::gcc IMPORTED)
  set_target_properties(ArmNoneEabiGcc::gcc PROPERTIES IMPORTED_LOCATION "${ArmNoneEabiGcc_GCC}")
endif()
if(ArmNoneEabiGcc_GXX AND NOT TARGET ArmNoneEabiGcc::gxx)
  add_executable(ArmNoneEabiGcc::gxx IMPORTED)
  set_target_properties(ArmNoneEabiGcc::gxx PROPERTIES IMPORTED_LOCATION "${ArmNoneEabiGcc_GXX}")
endif()
if(ArmNoneEabiGcc_OBJCOPY AND NOT TARGET ArmNoneEabiGcc::objcopy)
  add_executable(ArmNoneEabiGcc::objcopy IMPORTED)
  set_target_properties(ArmNoneEabiGcc::objcopy PROPERTIES IMPORTED_LOCATION "${ArmNoneEabiGcc_OBJCOPY}")
endif()
if(ArmNoneEabiGcc_SIZE AND NOT TARGET ArmNoneEabiGcc::size)
  add_executable(ArmNoneEabiGcc::size IMPORTED)
  set_target_properties(ArmNoneEabiGcc::size PROPERTIES IMPORTED_LOCATION "${ArmNoneEabiGcc_SIZE}")
endif()
if(ArmNoneEabiGcc_GDB AND NOT TARGET ArmNoneEabiGcc::gdb)
  add_executable(ArmNoneEabiGcc::gdb IMPORTED)
  set_target_properties(ArmNoneEabiGcc::gdb PROPERTIES IMPORTED_LOCATION "${ArmNoneEabiGcc_GDB}")
endif()

mark_as_advanced(
  ArmNoneEabiGcc_GCC
  ArmNoneEabiGcc_GXX
  ArmNoneEabiGcc_OBJCOPY
  ArmNoneEabiGcc_SIZE
  ArmNoneEabiGcc_GDB)
