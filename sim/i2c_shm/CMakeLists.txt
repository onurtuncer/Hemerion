# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# sim/i2c_shm -- simulated I2C bus carried in shared memory (see package.xml). Host-only, like everything under sim/.
# The platform-specific mapping is not repeated here: this target reuses sim/shm_bridge's ShmSegment, which already
# wraps shm_open()+mmap() and CreateFileMapping()+MapViewOfFile() behind one RAII type.
#
# POSITION_INDEPENDENT_CODE is on because the peripheral side of the bus is linked into sensor hardware-simulator FMUs,
# which generateFMU() builds as shared libraries -- a non-PIC static library cannot go into one on Linux.
# ------------------------------------------------------------------------------

find_package(Threads REQUIRED)

add_library(hemerion_i2c_shm STATIC src/i2c_shm_link.cpp src/i2c_peripheral_endpoint.cpp)

target_include_directories(hemerion_i2c_shm PUBLIC include)
target_compile_features(hemerion_i2c_shm PUBLIC cxx_std_23)
target_link_libraries(hemerion_i2c_shm PUBLIC hemerion_shm_bridge)
target_link_libraries(hemerion_i2c_shm PRIVATE Threads::Threads)
set_target_properties(hemerion_i2c_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The BMP390 device model, named once for both subdirectories below. Both compile these sources in directly rather than
# linking hemerion_sensors, so that sim/ never has to be ordered against modules/ -- but they had each spelled out the
# same three-level reach-in and the same file list, so splitting a translation unit (or moving either directory) meant
# editing two files and link-erroring in whichever was forgotten.
set(HEMERION_I2C_SHM_SENSORS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../modules/sensors")
set(HEMERION_I2C_SHM_BMP390_SOURCES "${HEMERION_I2C_SHM_SENSORS_DIR}/src/baro/bmp390/bmp390_compensation.cpp"
                                    "${HEMERION_I2C_SHM_SENSORS_DIR}/src/baro/bmp390/fmu/bmp390_i2c_slave.cpp")

# The MMC5983MA device model, named the same way and for the same reason. Shorter than the BMP390's list by one file:
# that part needs its compensation polynomial compiled in on both sides, this one's transfer function is a single scalar
# living in a header.
set(HEMERION_I2C_SHM_MMC5983MA_SOURCES "${HEMERION_I2C_SHM_SENSORS_DIR}/src/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.cpp")

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()

# Gated like test/ above, and for the same reason: these are host *tools*, not part of the bus library. Leaving them
# unconditional built them (and recompiled the BMP390 device-model sources a second time) in every fmu-native and
# examples-native configure, where nothing consumes them. HEMERION_BUILD_SIM is the axis that actually asks for sim/'s
# own deliverables -- fmu-native and examples-native only reach sim/ because FMUs need the transports underneath them.
if((HEMERION_BUILD_SIM OR HEMERION_BUILD_TESTS) AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/CMakeLists.txt")
  add_subdirectory(tools)
endif()
