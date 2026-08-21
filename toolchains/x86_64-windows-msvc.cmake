# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# x86_64-windows-msvc.cmake
#
# Native Windows host builds. Reached through toolchains/x86_64-native.cmake, which the native presets name -- see that
# file for why the indirection exists, and for why CMAKE_SYSTEM_NAME is absent here.
#
# The Windows 10 feature level is the substantive part: sim/shm_bridge's CreateFileMapping()/MapViewOfFile() path needs
# it, and NOMINMAX keeps <windows.h>'s min/max macros out of C++ headers that use std::min/std::max.
#
# The flags are guarded by compiler, not by host. This file is included from a toolchain file, where
# CMAKE_CXX_COMPILER_ID and MSVC are not yet set -- compiler detection has not run -- so the guard has to be a generator
# expression, evaluated at generate time when the answer is known. Without it, /W4 reaches any Windows build driven by a
# GNU-style driver (a clang++ targeting x86_64-pc-windows-msvc, which is what a Visual Studio-bundled clang is), where
# it is not a valid option. CI uses cl.exe via ilammy/msvc-dev-cmd; local Visual Studio clang builds do not.
#
# /std:c++latest is deliberately gone. The root CMakeLists.txt sets CMAKE_CXX_STANDARD 23 with EXTENSIONS OFF, from
# which CMake emits the correct /std: flag itself -- and, on MSVC, /permissive- along with it. Pinning c++latest here
# only fought that.
# ------------------------------------------------------------------------------

# Proof this file was processed, read by the stale-cache guard in the root CMakeLists.txt. Deliberately a normal
# variable and not a cache entry: it must vanish the moment the file stops running, which is exactly what a cache entry
# would not do.
set(HEMERION_TOOLCHAIN_APPLIED "x86_64-windows-msvc")

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/W4>")
add_compile_definitions(_WIN32_WINNT=0x0A00 NOMINMAX)
