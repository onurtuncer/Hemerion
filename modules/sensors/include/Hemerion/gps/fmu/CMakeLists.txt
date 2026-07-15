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
# Building the shared library here is the target this CMakeLists.txt produces; zip-packaging it into an actual .fmu
# archive (model_description.xml at the archive root, the binary under binaries/<platform>/) is not yet automated
# anywhere in this repo and is a follow-up, not done here.
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
