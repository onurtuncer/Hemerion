# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# sim/spi_shm -- simulated SPI bus carried in shared memory (see package.xml). Host-only, like everything under sim/.
# The platform-specific mapping is not repeated here: this target reuses sim/shm_bridge's ShmSegment, which already
# wraps shm_open()+mmap() and CreateFileMapping()+MapViewOfFile() behind one RAII type.
#
# POSITION_INDEPENDENT_CODE is on because the peripheral side of the bus is linked into sensor hardware-simulator FMUs,
# which generateFMU() builds as shared libraries -- a non-PIC static library cannot go into one on Linux.
# ------------------------------------------------------------------------------

find_package(Threads REQUIRED)

add_library(hemerion_spi_shm STATIC src/spi_shm_link.cpp)

target_include_directories(hemerion_spi_shm PUBLIC include)
target_compile_features(hemerion_spi_shm PUBLIC cxx_std_23)
target_link_libraries(hemerion_spi_shm PUBLIC hemerion_shm_bridge)
target_link_libraries(hemerion_spi_shm PRIVATE Threads::Threads)
set_target_properties(hemerion_spi_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
