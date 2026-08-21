# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# MMC5983MA magnetometer hardware simulator FMU (Co-Simulation). Native-only -- never cross-compiled, added by
# modules/sensors/CMakeLists.txt only when HEMERION_BUILD_FMU is ON, exactly like the GPS FMU
# (include/Hemerion/gps/fmu/CMakeLists.txt, which documents the layout and the generateFMU() contract shared by the
# sensor FMUs).
#
# Like the BMP390 FMU, this one does not emit bytes over UDP: the MMC5983MA is an I2C part, so the simulator answers
# addressed register transactions on a shared-memory bus (sim/i2c_shm) instead. modules/README.md's "modules must not
# depend on sim/ targets" rule protects cross-compiled firmware from host-only code, and this subtree is host-only
# simulation code by construction.
#
# Only the device model is compiled in. There is no compensation source to add -- unlike the BMP390, this part's
# transfer function is a single scalar, so the measurement model is header-only arithmetic -- and mmc5983ma_driver.cpp
# (the controller-side driver) is deliberately NOT compiled: the FMU is the peripheral, and the driver is on-target code
# living in the hemerion_sensors library.
#
# The packaged archive lands in ${HEMERION_FMU_OUTPUT_DIR}/{fmi2,fmi3}/hemerion_mmc5983ma_fmu.fmu.
# ------------------------------------------------------------------------------

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../src")

generateFMU(
  hemerion_mmc5983ma_fmu
  FMI_VERSIONS fmi2 fmi3
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fmu_main.cpp"
          "${_hemerion_sensors_src_dir}/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.cpp"
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
  LINK_TARGETS hemerion_i2c_shm
  DESTINATION "${HEMERION_FMU_OUTPUT_DIR}")

unset(_hemerion_sensors_src_dir)
