# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# x86_64-windows-msvc.cmake
#
# Toolchain file for native Windows builds with MSVC 2022. Used for
# fmu-native and shm_bridge builds on Windows; adds the Windows 10 shared-
# memory APIs required by sim/shm_bridge.
# ------------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Windows)
add_compile_options(/std:c++latest /permissive- /W4)
add_compile_definitions(_WIN32_WINNT=0x0A00 NOMINMAX)
