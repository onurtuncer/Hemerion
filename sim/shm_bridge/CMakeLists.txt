# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# sim/shm_bridge -- shared-memory transport between the FMI master and a
# locally running Aetherion process (see package.xml). Host-only: this
# directory is only reached when HEMERION_BUILD_SIM is ON, which the root
# CMakeLists.txt already guards against CMAKE_CROSSCOMPILING. Implemented on
# both POSIX (shm_open/mmap) and Windows (CreateFileMapping/MapViewOfFile),
# selected at compile time in src/shm_segment.cpp -- see
# cmake/Safety/FlightSafe.hpp, which already calls out this target by name
# as one of the host-only MSVC build cases.
# ------------------------------------------------------------------------------

find_package(Threads REQUIRED)

add_library(hemerion_shm_bridge STATIC src/shm_segment.cpp src/shm_bridge.cpp)

target_include_directories(hemerion_shm_bridge PUBLIC include)
target_compile_features(hemerion_shm_bridge PUBLIC cxx_std_23)
target_link_libraries(hemerion_shm_bridge PRIVATE Threads::Threads)

if(UNIX AND NOT APPLE)
  # glibc < 2.34 keeps shm_open()/shm_unlink() in librt rather than libc;
  # linking it is a no-op on newer glibc.
  target_link_libraries(hemerion_shm_bridge PRIVATE rt)
endif()

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
