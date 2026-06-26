# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# x86_64-linux.cmake
#
# Toolchain file for native Linux host builds (system GCC or Clang). Mostly a marker so presets reference a file rather
# than leaving CMAKE_TOOLCHAIN_FILE empty; the native-base preset family does not force this file automatically so the
# same preset name keeps working on CI runners without it.
# ------------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
