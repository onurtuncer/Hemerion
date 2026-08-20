# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Root superbuild -- includes all modules, bsp, sim, apps, tests, and docs. Cross/native/test build modes are selected
# via CMakePresets.json, which sets the toolchain file, HEMERION_BSP, and the HEMERION_BUILD_* options below.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

# Single source of truth for the project version is package.xml -- extract it here rather than duplicating the number in
# project(VERSION ...).
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" _hemerion_package_xml)
string(
  REGEX MATCH
        "<package[^>]* version=\"([0-9]+\\.[0-9]+\\.[0-9]+)\""
        _hemerion_version_match
        "${_hemerion_package_xml}")
if(NOT CMAKE_MATCH_1)
  message(FATAL_ERROR "Could not find version=\"X.Y.Z\" in package.xml")
endif()
set(HEMERION_VERSION "${CMAKE_MATCH_1}")
unset(_hemerion_package_xml)
unset(_hemerion_version_match)

project(
  Hemerion
  VERSION ${HEMERION_VERSION}
  DESCRIPTION "Embedded firmware framework for aerospace GNC and real-time embedded systems"
  HOMEPAGE_URL "https://github.com/onurtuncer/Hemerion"
  LANGUAGES C CXX ASM)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ------------------------------------------------------------------------------
# Stale-cache guard: a toolchain file that is named but never ran.
#
# CMAKE_TOOLCHAIN_FILE is honoured only on a build tree's *first* configure. Point a preset at a toolchain file it did
# not previously name, re-run `cmake --preset` against a directory configured before that change, and CMake adopts none
# of it -- while still writing CMAKE_TOOLCHAIN_FILE into the cache. The build tree then reads as correctly configured by
# every obvious check (the cache names the right file) while compiling with none of that file's flags or definitions.
# The only surface warning is one "Manually-specified variables were not used by the project" line in a wall of
# configure output, and it is not an error.
#
# The discriminator is the file having actually run: every toolchain file under toolchains/ sets
# HEMERION_TOOLCHAIN_APPLIED as a normal variable, which is present on a healthy configure *and on every reconfigure of
# a healthy tree* (CMake re-reads an adopted toolchain file each time), and absent exactly when the cache was built
# without it.
#
# Scoped to our own files: a foreign toolchain (vcpkg, a distribution's cross file, a user's own) will not set the
# sentinel and must not be failed over it.
# ------------------------------------------------------------------------------
if(CMAKE_TOOLCHAIN_FILE AND NOT HEMERION_TOOLCHAIN_APPLIED)
  file(REAL_PATH "${CMAKE_TOOLCHAIN_FILE}" _hemerion_named_toolchain)
  file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/toolchains" _hemerion_toolchain_dir)
  # Compared case-insensitively: on Windows the same path reaches us spelled both "D:/dev/..." and "D:/Dev/...".
  string(TOLOWER "${_hemerion_named_toolchain}" _hemerion_named_toolchain)
  string(TOLOWER "${_hemerion_toolchain_dir}/" _hemerion_toolchain_dir)
  string(FIND "${_hemerion_named_toolchain}" "${_hemerion_toolchain_dir}" _hemerion_toolchain_is_ours)

  if(_hemerion_toolchain_is_ours EQUAL 0)
    get_filename_component(_hemerion_build_dir_name "${CMAKE_BINARY_DIR}" NAME)
    message(
      FATAL_ERROR
        "${CMAKE_TOOLCHAIN_FILE} is named by this configure but was never applied: "
        "${CMAKE_BINARY_DIR} was configured before that toolchain file was in play, and CMake honours "
        "CMAKE_TOOLCHAIN_FILE only on a build tree's first configure. Everything it sets -- compiler, flags, "
        "definitions -- is silently missing from this build tree.
"
        "Reconfigure from scratch:
"
        "    cmake --preset ${_hemerion_build_dir_name} --fresh
"
        "(or delete ${CMAKE_BINARY_DIR} and configure again). Preset names match their build directory name; if this "
        "tree was not made by a preset, delete it and configure again however you did the first time.")
  endif()

  unset(_hemerion_named_toolchain)
  unset(_hemerion_toolchain_dir)
  unset(_hemerion_toolchain_is_ours)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------------------------
# Build mode options -- named CMakePresets bind these to concrete configurations
# ------------------------------------------------------------------------------
option(HEMERION_BUILD_APPS "Build firmware app executables (apps/)" ON)
option(HEMERION_BUILD_FMU "Build modules as x86 FMI 2.0 + 3.0 co-simulation FMUs for Aetherion co-simulation" OFF)
option(HEMERION_BUILD_TESTS "Build native unit/integration tests" OFF)
option(HEMERION_BUILD_SIM "Build host-side simulation targets (sim/) -- never cross-compiled" OFF)
option(HEMERION_BUILD_EXAMPLES "Build host-side co-simulation examples (examples/) -- requires HEMERION_BUILD_FMU" OFF)
option(HEMERION_BUILD_DOCS "Build Doxygen/Sphinx documentation" OFF)

# ------------------------------------------------------------------------------
# Native-build instrumentation. Both default OFF and are turned on by the workflow that wants them
# (.github/workflows/sanitizers.yml, coverage.yml) rather than baked into test-native, so the ordinary build-and-test
# jobs stay fast and the coverage run does not measure a sanitizer-instrumented binary.
#
# Included here, before any add_subdirectory(), because both work by add_compile_options()/add_link_options() at
# directory scope -- which only reach directories processed after the call.
# ------------------------------------------------------------------------------
option(HEMERION_ENABLE_SANITIZERS "Build native targets with ASan + UBSan (GCC/Clang only)" OFF)
option(HEMERION_ENABLE_COVERAGE "Build native targets with gcov coverage instrumentation (GCC/Clang only)" OFF)
include(sanitizers)
include(coverage)

# Must run before any add_subdirectory() that calls add_test() (every module's test/CMakeLists.txt, plus tests/ below)
# -- enable_testing() only takes effect in directories processed after it runs, so calling it down in the "cross-cutting
# integration / SWIL tests" block (after modules/apps/sim are already added) left ctest unable to see any module-level
# test.
#
# HEMERION_BUILD_FMU counts too, even though it turns HEMERION_BUILD_TESTS off: generateFMU() registers one
# "fmu"-labelled archive-layout test per packaged FMU (cmake/fmu_archive_test.cmake), and without enable_testing() here
# those would be recorded and never written out -- leaving the fmu-native test preset filtering on a label no test in
# the tree carried, which is exactly the vacuous pass it used to be.
if(HEMERION_BUILD_TESTS OR HEMERION_BUILD_FMU)
  enable_testing()
endif()

set(HEMERION_BSP "" CACHE STRING "Active board support package directory under bsp/ (empty = none selected)")

if(HEMERION_BUILD_FMU AND CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_BUILD_FMU requires a native toolchain, not a cross-compiling one")
endif()
if(HEMERION_BUILD_SIM AND CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_BUILD_SIM requires a native toolchain -- sim/ is host-only")
endif()
if(HEMERION_BUILD_EXAMPLES AND CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "HEMERION_BUILD_EXAMPLES requires a native toolchain -- examples/ is host-only")
endif()

# ------------------------------------------------------------------------------
# FreeRTOS port selection -- must run before add_subdirectory(vendor) below: vendor/FreeRTOS-Kernel's CMakeLists.txt
# requires a `freertos_config` target and FREERTOS_PORT to already exist, and FreeRTOSConfig.h lives in the BSP, not in
# vendor/ (see bsp/README.md's FreeRTOSConfig notes). The active BSP supplies both via freertos_port.cmake; BSPs without
# FreeRTOS support (e.g. native_linux) simply omit that file and FreeRTOS-Kernel is left unconfigured.
# ------------------------------------------------------------------------------
if(HEMERION_BSP AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bsp/${HEMERION_BSP}/freertos_port.cmake")
  add_library(freertos_config INTERFACE)
  target_include_directories(freertos_config INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/bsp/${HEMERION_BSP}")
  include("bsp/${HEMERION_BSP}/freertos_port.cmake")
endif()

# ------------------------------------------------------------------------------
# Vendored dependencies -- git submodules where upstream's layout allows it (see vendor/CMakeLists.txt)
# ------------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/CMakeLists.txt")
  add_subdirectory(vendor)
endif()

# ------------------------------------------------------------------------------
# FMU export helper -- generateFMU() builds and packages FMI co-simulation FMUs on top of the vendored fmu4cpp export
# layer that vendor/CMakeLists.txt just added. Included here (not per module) so every module that exports an FMU calls
# the same helper, and so the fmu4cpp targets it needs already exist.
#
# HEMERION_FMU_OUTPUT_DIR collects the packaged archives in one place instead of scattering them across each model's
# build directory; examples/ and any external FMI master read them from there.
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_FMU)
  set(HEMERION_FMU_OUTPUT_DIR "${CMAKE_BINARY_DIR}/fmus"
      CACHE INTERNAL "Directory the packaged .fmu archives are written to, one subdirectory per FMI version")
  include(generate_fmu)
endif()

# ------------------------------------------------------------------------------
# Board support package -- owns linker scripts, startup code, FreeRTOSConfig.h
# ------------------------------------------------------------------------------
if(HEMERION_BSP AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bsp/${HEMERION_BSP}/CMakeLists.txt")
  add_subdirectory("bsp/${HEMERION_BSP}")
endif()

# ------------------------------------------------------------------------------
# Host-side simulation targets -- Renode glue, FMI master, shm/UDP/SPI bridges.
#
# Nothing under sim/ depends on modules/, so this can go first, and it does. The dependency runs the other way exactly
# once: the IMU hardware-simulator FMU answers a shared-memory SPI bus (sim/spi_shm) instead of emitting UDP like the
# other four sensor FMUs, which makes it the one module target that links a sim/ library.
#
# That ordering is a guard, not a requirement -- an earlier version of this comment claimed the target "has to exist by
# the time modules/sensors is processed", which is not true of either CMake or this tree. target_link_libraries()
# accepts a name that is not a target yet and resolves it at generate time, so the link itself is indifferent to the
# order. What does read LINK_TARGETS at configure time is generateFMU()'s _bundle_link_libraries(), through if(TARGET)
# and get_target_property(): a name it has not seen yet is silently skipped, not diagnosed. hemerion_spi_shm is STATIC
# and that macro skips STATIC libraries regardless, so today the order changes nothing observable at all. It starts to
# matter the day an FMU's sim/ dependency is SHARED -- bundled, its runtime lands next to the model binary in the
# archive; skipped, the FMU packages cleanly and fails to load only once unzipped on a machine without that library.
#
# Building FMUs or examples implies the transports they sit on, hence the widened condition.
# ------------------------------------------------------------------------------
if((HEMERION_BUILD_SIM OR HEMERION_BUILD_FMU OR HEMERION_BUILD_EXAMPLES)
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sim/CMakeLists.txt")
  add_subdirectory(sim)
endif()

# ------------------------------------------------------------------------------
# Modules -- reusable firmware libraries; each builds a static lib, and optionally an FMU and/or native test binary
# depending on the options above
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
# Apps -- top-level firmware executables (link modules + bsp); skipped when building FMUs, since FMU shared libraries
# are the link target in that mode
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_APPS AND NOT HEMERION_BUILD_FMU AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/CMakeLists.txt")
  add_subdirectory(apps)
endif()

# ------------------------------------------------------------------------------
# Examples -- host-only end-to-end demonstrations (e.g. Ecos FMU co-simulation)
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_EXAMPLES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()

# ------------------------------------------------------------------------------
# Cross-cutting integration / SWIL tests
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_TESTS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
  endif()
endif()

# ------------------------------------------------------------------------------
# Documentation
# ------------------------------------------------------------------------------
if(HEMERION_BUILD_DOCS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/doc/CMakeLists.txt")
  add_subdirectory(doc)
endif()
