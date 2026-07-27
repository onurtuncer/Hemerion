# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# GPS UBX hardware simulator FMU (Co-Simulation). Native-only -- never cross-compiled, added by
# modules/sensors/CMakeLists.txt only when HEMERION_BUILD_FMU is ON, matching the package.xml-declared <artifact
# type="fmu" condition="HEMERION_BUILD_FMU"/> export.
#
# This subdirectory lives under include/Hemerion/gps/fmu/ rather than the include/src split the rest of modules/sensors
# uses, because fmu_main.cpp is this component's entry point (like an app's main.cpp), not a module public header. Its
# supporting .cpp files (ubxEmitter.cpp, udpSender.cpp) do follow the normal split and live under src/gps/fmu/ --
# referenced below via a relative path back into modules/sensors/src/.
#
# generateFMU() (cmake/generate_fmu.cmake) does the rest: it compiles these sources against the vendored fmu4cpp export
# layer into the hemerion_gps_fmu_fmi2 shared library, generates modelDescription.xml from the variables fmu_main.cpp
# registers, and packages both into ${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_gps_fmu.fmu so FMI masters such as Ecos can
# load it directly. That archive is what examples/rocket_gps_ecos consumes. Adding fmi3 to FMI_VERSIONS would emit an
# FMI 3.0 archive alongside it from the very same sources.
# ------------------------------------------------------------------------------

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")

generateFMU(
  hemerion_gps_fmu
  FMI_VERSIONS fmi2
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fmu_main.cpp" "${_hemerion_sensors_src_dir}/gps/fmu/ubxEmitter.cpp"
          "${_hemerion_sensors_src_dir}/gps/fmu/udpSender.cpp"
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  LINK_TARGETS ${HEMERION_FMU_SOCKET_LIBRARIES}
  DESTINATION "${HEMERION_FMU_OUTPUT_DIR}")

unset(_hemerion_sensors_src_dir)
