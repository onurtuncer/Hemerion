# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# x86_64-native.cmake
#
# The toolchain file the native preset family (native-base, and so test-native / fmu-native / examples-native) names
# unconditionally. It carries no settings of its own -- it forwards to the host's own file below.
#
# Why a dispatcher exists at all. A preset binds exactly one toolchainFile, but those presets are built under the same
# names on two hosts: Linux with gcc-14 and Windows with MSVC (see .github/workflows/linux.yml and windows.yml). The
# previous answer was to name no toolchain file, which kept the preset names portable at the cost of making
# x86_64-linux.cmake and x86_64-windows-msvc.cmake apply to nothing at all -- toolchains/README.md's claim that presets
# reference these files was simply untrue of the native ones.
#
# What a native toolchain file must not do: set CMAKE_SYSTEM_NAME. CMake sets CMAKE_CROSSCOMPILING to TRUE whenever that
# variable is assigned from a toolchain file, *even when it is assigned the host's own name*, and the root
# CMakeLists.txt hard-errors on CMAKE_CROSSCOMPILING for HEMERION_BUILD_FMU, _SIM and _EXAMPLES. Both native files used
# to set it, which is why pointing a preset at them would have failed configure outright. They no longer do.
#
# What is and is not visible here. CMAKE_HOST_SYSTEM_NAME is already populated when a toolchain file is read, so the
# branch below is safe. CMAKE_CXX_COMPILER_ID and MSVC are not -- compiler detection has not run yet -- so per-compiler
# flags in the included files are written as generator expressions rather than if(MSVC).
# ------------------------------------------------------------------------------

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
  include("${CMAKE_CURRENT_LIST_DIR}/x86_64-windows-msvc.cmake")
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  include("${CMAKE_CURRENT_LIST_DIR}/x86_64-linux.cmake")
else()
  # Loud rather than fatal: an untested host still configures and builds, but nobody gets to believe a host-specific
  # file was applied when none exists. Add toolchains/x86_64-<host>.cmake and a branch here to fix it properly.
  message(WARNING "No native toolchain file for host '${CMAKE_HOST_SYSTEM_NAME}'; using the Linux one, which carries "
                  "no host-specific flags. Only Linux and Windows are covered by CI.")
  include("${CMAKE_CURRENT_LIST_DIR}/x86_64-linux.cmake")
endif()
