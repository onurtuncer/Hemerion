# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# IMU hardware simulator FMU (Co-Simulation). Native-only -- never cross-compiled, added by
# modules/sensors/CMakeLists.txt only when HEMERION_BUILD_FMU is ON, exactly like the GPS FMU one subtree over
# (include/Hemerion/gps/fmu/CMakeLists.txt, which documents the layout and the generateFMU() contract shared by all five
# sensor FMUs).
#
# Unlike the other four, this FMU does not emit its bytes over UDP: the IMU is an SPI part, so the simulator answers
# chip-select-framed transfers on a shared-memory bus (sim/spi_shm) instead. That makes this the one module target that
# links a sim/ library. modules/README.md's "modules must not depend on sim/ targets" rule protects cross-compiled
# firmware from host-only code, and this subtree is host-only simulation code by construction -- see the dependency
# section there. imu_packet.cpp (the parser) and imu_spi_driver.cpp (the controller-side driver) are deliberately NOT
# compiled in: the FMU is the peripheral; both of those are on-target code living in the hemerion_sensors library.
#
# The packaged archive lands in ${HEMERION_FMU_OUTPUT_DIR}/{fmi2,fmi3}/hemerion_imu_fmu.fmu, which is what
# examples/rocket_gps_ecos consumes.
# ------------------------------------------------------------------------------

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")

generateFMU(
  hemerion_imu_fmu
  FMI_VERSIONS fmi2 fmi3
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fmu_main.cpp" "${_hemerion_sensors_src_dir}/imu/fmu/imu_packet_emitter.cpp"
          "${_hemerion_sensors_src_dir}/imu/fmu/imu_spi_slave.cpp"
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  LINK_TARGETS hemerion_spi_shm
  DESTINATION "${HEMERION_FMU_OUTPUT_DIR}")

unset(_hemerion_sensors_src_dir)
