# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# x86_64-linux.cmake
#
# Native Linux host builds (system GCC or Clang; CI pins gcc-14 for <print>). Reached through
# toolchains/x86_64-native.cmake, which the native presets name -- see that file for why the indirection exists.
#
# Deliberately thin. It carries no warning flags: turning them on here would change how every native translation unit
# compiles, which is a policy change and not this file's business. Its job is to be the place those flags will go.
#
# It does NOT set CMAKE_SYSTEM_NAME or CMAKE_SYSTEM_PROCESSOR. Assigning CMAKE_SYSTEM_NAME from a toolchain file sets
# CMAKE_CROSSCOMPILING to TRUE even when the value matches the host, and the root CMakeLists.txt hard-errors on that for
# HEMERION_BUILD_FMU / _SIM / _EXAMPLES -- see x86_64-native.cmake. A native build is not cross-compiling and must not
# claim to be.
# ------------------------------------------------------------------------------

# Proof this file was processed, read by the stale-cache guard in the root CMakeLists.txt. Deliberately a normal
# variable and not a cache entry: it must vanish the moment the file stops running, which is exactly what a cache entry
# would not do.
set(HEMERION_TOOLCHAIN_APPLIED "x86_64-linux")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
