# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# gcov/gcovr line-coverage instrumentation, enabled by HEMERION_ENABLE_COVERAGE.
#
# Until this file existed, nothing in the tree passed a coverage flag -- not the preset, not the root CMakeLists, not
# the workflow. .github/workflows/coverage.yml had a step titled "Configure (native host + coverage flags)" that passed
# none, ran gcovr over a build tree containing no .gcda or .gcno, and uploaded the result with fail_ci_if_error: false.
# The codecov badge in README.md was reporting on a measurement that never happened.
#
# --coverage is the driver flag for both halves: -fprofile-arcs at compile time (emitting .gcno beside each object and
# .gcda when the binary runs) and -lgcov at link time. Applying it to only the compile side is the usual way to get a
# link error or, worse, a binary that writes no counters.
#
# Debug and no optimisation matter as much as the flag: gcov attributes counts to source lines, and an optimised build
# reorders and merges them into numbers that do not correspond to anything a reader can act on. test-native is already a
# Debug preset, so this only has to avoid re-introducing optimisation.
#
# GCC and Clang only. MSVC has its own coverage tooling with a different data format that gcovr cannot read, so a
# Windows coverage run is not silently half-configured -- it is refused.
# ------------------------------------------------------------------------------

if(NOT HEMERION_ENABLE_COVERAGE)
  return()
endif()

if(CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_ENABLE_COVERAGE is a native-build option: coverage counters are written by the process "
                      "under test, and cross-built firmware does not run on the build host.")
endif()

if(NOT
   CMAKE_CXX_COMPILER_ID
   MATCHES
   "^(GNU|Clang|AppleClang)$"
   OR MSVC)
  message(
    FATAL_ERROR
      "HEMERION_ENABLE_COVERAGE needs GCC or Clang with a GNU-style driver; got "
      "'${CMAKE_CXX_COMPILER_ID}'. Refused rather than configured without instrumentation, which "
      "would produce an empty report that still uploads successfully.")
endif()

add_compile_options(--coverage -O0 -g)
add_link_options(--coverage)
message(STATUS "Coverage: gcov instrumentation enabled (${CMAKE_CXX_COMPILER_ID})")
