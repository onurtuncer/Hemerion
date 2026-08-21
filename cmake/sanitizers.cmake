# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# AddressSanitizer + UndefinedBehaviorSanitizer for native test builds, enabled by HEMERION_ENABLE_SANITIZERS.
#
# This file is what .github/workflows/sanitizers.yml has always claimed to use. It did not exist: that job configured
# test-native exactly like the Linux build job, exported ASAN_OPTIONS and UBSAN_OPTIONS that nothing read, and reported
# success without a sanitizer ever being linked in. The CMakePresets description for test-native asserted the same
# thing. Both are now true.
#
# Opt-in rather than always-on for test-native, so the plain build-and-test jobs stay fast and the coverage job is not
# measuring an instrumented binary. The sanitizer job passes -DHEMERION_ENABLE_SANITIZERS=ON.
#
# Scope. Sanitizers instrument code and runtime alike, so the flags must reach both compile and link -- a common way to
# get a silently unsanitized binary is to set only the compile side. Applied at directory scope from the root, they
# cover modules/, sim/ and tests/ together.
#
# Not enabled on MSVC: /fsanitize=address exists there but has no UBSan counterpart, needs its own runtime DLL beside
# every test executable, and conflicts with the debug runtime checks a Debug preset turns on. Windows CI runs the same
# preset without sanitizers rather than half of one, and says so.
# ------------------------------------------------------------------------------

if(NOT HEMERION_ENABLE_SANITIZERS)
  return()
endif()

if(CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_ENABLE_SANITIZERS is a native-build option -- there is no sanitizer runtime for the "
                      "arm-none-eabi target. Turn it off for cross presets.")
endif()

set(_hemerion_sanitizer_flags -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)

if(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$" AND NOT MSVC)
  add_compile_options(${_hemerion_sanitizer_flags})
  add_link_options(${_hemerion_sanitizer_flags})
  message(STATUS "Sanitizers: ASan + UBSan enabled (${CMAKE_CXX_COMPILER_ID})")
else()
  # Loud, because a silent skip here is the whole failure this file exists to end: a job that reports success while
  # checking nothing.
  message(WARNING "HEMERION_ENABLE_SANITIZERS is ON but ${CMAKE_CXX_COMPILER_ID} is not supported here -- this build "
                  "has NO sanitizers. Only GCC and Clang with a GNU-style driver are instrumented.")
endif()

unset(_hemerion_sanitizer_flags)
