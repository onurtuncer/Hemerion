# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# BMP390 barometer hardware simulator FMU (Co-Simulation). Native-only -- never cross-compiled, added by
# modules/sensors/CMakeLists.txt only when HEMERION_BUILD_FMU is ON, exactly like the GPS FMU
# (include/Hemerion/gps/fmu/CMakeLists.txt, which documents the layout and the generateFMU() contract shared by the
# sensor FMUs).
#
# Like the IMU FMU (a SPI part on sim/spi_shm), this FMU does not emit bytes over UDP: the BMP390 is an I2C part, so the
# simulator answers addressed register transactions on a shared-memory bus (sim/i2c_shm) instead. modules/README.md's
# "modules must not depend on sim/ targets" rule protects cross-compiled firmware from host-only code, and this subtree
# is host-only simulation code by construction. bmp390_compensation.cpp is compiled in because the measurement model
# inverts the real compensation polynomial; bmp390_driver.cpp (the controller-side driver) is deliberately NOT -- the
# FMU is the peripheral, and the driver is on-target code living in the hemerion_sensors library.
#
# The packaged archive lands in ${HEMERION_FMU_OUTPUT_DIR}/{fmi2,fmi3}/hemerion_bmp390_fmu.fmu.
# ------------------------------------------------------------------------------

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../src")

generateFMU(
  hemerion_bmp390_fmu
  FMI_VERSIONS fmi2 fmi3
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fmu_main.cpp" "${_hemerion_sensors_src_dir}/baro/bmp390/fmu/bmp390_i2c_slave.cpp"
          "${_hemerion_sensors_src_dir}/baro/bmp390/bmp390_compensation.cpp"
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
  LINK_TARGETS hemerion_i2c_shm
  DESTINATION "${HEMERION_FMU_OUTPUT_DIR}")

unset(_hemerion_sensors_src_dir)
