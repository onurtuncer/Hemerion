# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Magnetometer hardware simulator FMU (Co-Simulation). Native-only -- never cross-compiled, added by
# modules/sensors/CMakeLists.txt only when HEMERION_BUILD_FMU is ON, exactly like the GPS FMU
# (include/Hemerion/gps/fmu/CMakeLists.txt, which documents the layout and the generateFMU() contract shared by all five
# sensor FMUs).
#
# udpSender.cpp is reused from the GPS FMU's sources -- it is sensor-agnostic and lives in the same module, so
# duplicating it here would only invite drift. mag_packet.cpp (the parser) is deliberately NOT compiled in: the FMU only
# emits; the parser is on-target code living in the hemerion_sensors library.
#
# The packaged archive lands in ${HEMERION_FMU_OUTPUT_DIR}/{fmi2,fmi3}/hemerion_mag_fmu.fmu.
# ------------------------------------------------------------------------------

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")

generateFMU(
  hemerion_mag_fmu
  FMI_VERSIONS fmi2 fmi3
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fmu_main.cpp" "${_hemerion_sensors_src_dir}/mag/fmu/mag_packet_emitter.cpp"
          "${_hemerion_sensors_src_dir}/gps/fmu/udpSender.cpp"
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  LINK_TARGETS ${HEMERION_FMU_SOCKET_LIBRARIES}
  DESTINATION "${HEMERION_FMU_OUTPUT_DIR}")

unset(_hemerion_sensors_src_dir)
