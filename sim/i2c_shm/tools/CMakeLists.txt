# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Host tools for the simulated I2C bus -- the two halves the Renode SWIL loop needs (see
# sim/renode/i2c_bridge/DESIGN.md): i2c_shm_tcp_bridge serves the controller end over TCP for a controller living in
# another world (Renode's emulated STM32), and bmp390_shm_peripheral is the BMP390 FMU minus FMI, for harnesses with no
# FMI master in the room. The BMP390 sources are compiled in directly rather than linked from hemerion_sensors, for the
# same target-ordering reason as sim/i2c_shm/test/CMakeLists.txt -- both take the file list from
# HEMERION_I2C_SHM_BMP390_SOURCES, set by the parent directory, rather than each spelling out its own copy.
# ------------------------------------------------------------------------------

add_executable(i2c_shm_tcp_bridge i2c_shm_tcp_bridge.cpp)
target_link_libraries(i2c_shm_tcp_bridge PRIVATE hemerion_i2c_shm)
target_compile_features(i2c_shm_tcp_bridge PRIVATE cxx_std_23)
if(WIN32)
  target_link_libraries(i2c_shm_tcp_bridge PRIVATE ws2_32)
endif()

add_executable(bmp390_shm_peripheral bmp390_shm_peripheral.cpp ${HEMERION_I2C_SHM_BMP390_SOURCES})
target_include_directories(bmp390_shm_peripheral PRIVATE "${HEMERION_I2C_SHM_SENSORS_DIR}/include")
target_link_libraries(bmp390_shm_peripheral PRIVATE hemerion_i2c_shm)
target_compile_features(bmp390_shm_peripheral PRIVATE cxx_std_23)

add_executable(mmc5983ma_shm_peripheral mmc5983ma_shm_peripheral.cpp ${HEMERION_I2C_SHM_MMC5983MA_SOURCES})
target_include_directories(mmc5983ma_shm_peripheral PRIVATE "${HEMERION_I2C_SHM_SENSORS_DIR}/include")
target_link_libraries(mmc5983ma_shm_peripheral PRIVATE hemerion_i2c_shm)
target_compile_features(mmc5983ma_shm_peripheral PRIVATE cxx_std_23)
