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
# uses, because fmuMain.cpp is this component's entry point (like an app's main.cpp), not a module public header. Its
# supporting .cpp files (ubxEmitter.cpp, udpSender.cpp) do follow the normal split and live under src/gps/fmu/ --
# referenced below via a relative path back into modules/sensors/src/.
#
# Two targets: the hemerion_gps_fmu shared library, and hemerion_gps_fmu_package, which zips it into a proper FMI 2.0
# archive (<build>/hemerion_gps_fmu.fmu: modelDescription.xml at the archive root, the binary under
# binaries/<win64|linux64>/) so FMI masters such as Ecos can load it directly. The packaging step is what
# examples/rocket_gps_ecos consumes.
# ------------------------------------------------------------------------------

if(NOT HEMERION_BUILD_FMU)
  return()
endif()

set(_hemerion_sensors_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")

add_library(hemerion_gps_fmu SHARED fmuMain.cpp "${_hemerion_sensors_src_dir}/gps/fmu/ubxEmitter.cpp"
                                    "${_hemerion_sensors_src_dir}/gps/fmu/udpSender.cpp")

target_include_directories(hemerion_gps_fmu PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
target_compile_features(hemerion_gps_fmu PRIVATE cxx_std_23)
set_target_properties(hemerion_gps_fmu PROPERTIES PREFIX "")

if(WIN32)
  target_link_libraries(hemerion_gps_fmu PRIVATE ws2_32)
endif()

unset(_hemerion_sensors_src_dir)

# ------------------------------------------------------------------------------
# .fmu archive packaging. FMI 2.0 requires the file to be named modelDescription.xml inside the archive (the source file
# keeps the repo's snake_case name), and the shared library to live under binaries/<platform>/ named exactly after the
# <CoSimulation modelIdentifier>. `cmake -E tar --format=zip` produces the archive so no external zip tool is needed.
# ------------------------------------------------------------------------------
if(WIN32)
  set(_hemerion_fmu_platform win64)
else()
  set(_hemerion_fmu_platform linux64)
endif()

set(_hemerion_fmu_stage_dir "${CMAKE_CURRENT_BINARY_DIR}/fmu_stage")
# The custom command below cd's into the stage dir (WORKING_DIRECTORY) before its first COMMAND runs, so the directory
# must already exist at configure time.
file(MAKE_DIRECTORY "${_hemerion_fmu_stage_dir}")
set(HEMERION_GPS_FMU_ARCHIVE "${CMAKE_CURRENT_BINARY_DIR}/hemerion_gps_fmu.fmu"
    CACHE INTERNAL "Path to the packaged GPS UBX simulator .fmu archive")

add_custom_command(
  OUTPUT "${HEMERION_GPS_FMU_ARCHIVE}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${_hemerion_fmu_stage_dir}/binaries/${_hemerion_fmu_platform}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/model_description.xml"
          "${_hemerion_fmu_stage_dir}/modelDescription.xml"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:hemerion_gps_fmu>"
          "${_hemerion_fmu_stage_dir}/binaries/${_hemerion_fmu_platform}/"
  COMMAND ${CMAKE_COMMAND} -E tar cf "${HEMERION_GPS_FMU_ARCHIVE}" --format=zip -- modelDescription.xml binaries
  WORKING_DIRECTORY "${_hemerion_fmu_stage_dir}"
  DEPENDS hemerion_gps_fmu "${CMAKE_CURRENT_SOURCE_DIR}/model_description.xml"
  COMMENT "Packaging hemerion_gps_fmu.fmu (FMI 2.0 zip archive)"
  VERBATIM)

add_custom_target(hemerion_gps_fmu_package ALL DEPENDS "${HEMERION_GPS_FMU_ARCHIVE}")

unset(_hemerion_fmu_platform)
unset(_hemerion_fmu_stage_dir)
