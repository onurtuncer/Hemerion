# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Root superbuild -- includes all modules, bsp, sim, apps, tests, and docs.
# Cross/native/test build modes are selected via CMakePresets.json, which sets
# the toolchain file, HEMERION_BSP, and the HEMERION_BUILD_* options below.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

project(
  Hemerion
  VERSION 0.1.0
  DESCRIPTION "Embedded firmware framework for aerospace GNC and real-time embedded systems"
  HOMEPAGE_URL "https://github.com/onurtuncer/Hemerion"
  LANGUAGES C CXX ASM)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      RelWithDebInfo
      CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------------------------
# Build mode options -- named CMakePresets bind these to concrete configurations
# ------------------------------------------------------------------------------
option(HEMERION_BUILD_APPS "Build firmware app executables (apps/)" ON)
option(HEMERION_BUILD_FMU "Build modules as x86 FMI 2.0 shared libraries for Aetherion co-simulation" OFF)
option(HEMERION_BUILD_TESTS "Build native unit/integration tests" OFF)
option(HEMERION_BUILD_SIM "Build host-side simulation targets (sim/) -- never cross-compiled" OFF)
option(HEMERION_BUILD_DOCS "Build Doxygen/Sphinx documentation" OFF)

set(HEMERION_BSP
    ""
    CACHE STRING "Active board support package directory under bsp/ (empty = none selected)")

if(HEMERION_BUILD_FMU AND CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_BUILD_FMU requires a native toolchain, not a cross-compiling one")
endif()
if(HEMERION_BUILD_SIM AND CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_BUILD_SIM requires a native toolchain -- sim/ is host-only")
endif()

# ------------------------------------------------------------------------------
# Vendored dependencies (git submodules) -- no source copies live in this tree
# ------------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/CMakeLists.txt")
  add_subdirectory(vendor)
endif()

# ------------------------------------------------------------------------------
# Board support package -- owns linker scripts, startup code, FreeRTOSConfig.h
# ------------------------------------------------------------------------------
if(HEMERION_BSP AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bsp/${HEMERION_BSP}/CMakeLists.txt")
  add_subdirectory("bsp/${HEMERION_BSP}")
endif()

# ------------------------------------------------------------------------------
# Modules -- reusable firmware libraries; each builds a static lib, and
# optionally an FMU and/or native test binary depending on the options above
# ------------------------------------------------------------------------------
foreach(
  _hemerion_module
  sensors
  actuators
  comms
  gnc
  rtos_core
  fault
  power
  datalogger)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/modules/${_hemerion_module}/CMakeLists.txt")
    add_subdirectory("modules/${_hemerion_module}")
  endif()
endforeach()
unset(_hemerion_module)

# ------------------------------------------------------------------------------
# Apps -- top-level firmware executables (link modules + bsp); skipped when
# building FMUs, since FMU shared libraries are the link target in that mode
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_APPS
   AND NOT HEMERION_BUILD_FMU
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/CMakeLists.txt")
  add_subdirectory(apps)
endif()

# ------------------------------------------------------------------------------
# Host-side simulation targets -- Renode glue, FMI master, shm/UDP bridges
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_SIM AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sim/CMakeLists.txt")
  add_subdirectory(sim)
endif()

# ------------------------------------------------------------------------------
# Cross-cutting integration / SWIL tests
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_TESTS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()

# ------------------------------------------------------------------------------
# Documentation
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_DOCS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/doc/CMakeLists.txt")
  add_subdirectory(doc)
endif()
