# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# examples/rocket_gps_ecos -- Ecos co-simulation of Aetherion's TwoStageRocket.fmu feeding Hemerion's GPS UBX
# hardware-simulator FMU, whose UBX-NAV-PVT UDP stream is decoded by the same GpsDriver/UbxParser stack that runs on the
# STM32H743 flight computer. See README.md and doc/rocket_gps_ecos_cosim.rst.
#
# Two host executables: rocket_gps_cosim    -- Ecos master: loads both FMUs, wires truth -> GPS, logs rocket truth to
# CSV gps_flight_computer -- consumes the FMU's UBX UDP stream with the on-target GPS stack, logs decoded fixes
#
# The Ecos platform library (github.com/Ecos-platform/ecos) is fetched and built from source here; its own CMake fetches
# everything else it needs (fmi4c for FMU loading, header-only utilities). CLI/C-bindings/proxyfmu are switched off --
# only the C++ library target `libecos` is consumed.
# ------------------------------------------------------------------------------

if(NOT TARGET hemerion_gps_fmu_package OR NOT TARGET hemerion_imu_fmu_package)
  message(FATAL_ERROR "examples/rocket_gps_ecos requires the GPS and IMU FMU package targets -- configure with "
                      "HEMERION_BUILD_FMU=ON (use the examples-native preset)")
endif()

include(FetchContent)

set(ECOS_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_CLIB OFF CACHE BOOL "" FORCE)
set(ECOS_WITH_PROXYFMU OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  ecos GIT_REPOSITORY https://github.com/Ecos-platform/ecos.git GIT_TAG v.0.7.2 # note the vendor's unusual tag
                                                                                # spelling; this is release 0.7.2
)
FetchContent_MakeAvailable(ecos)

# Toolchain workarounds for the fetched sources, applied at the target level rather than patching them: - fmi4c's
# vendored ezxml calls POSIX read() with no declaration in scope on Windows; MSVC merely warns (C4013) but clang treats
# implicit function declarations as a hard error. The symbol itself still links via the MSVC CRT's oldnames read->_read
# alias. - ecos' bundled spdlog/fmt trips clang >= 17's stricter consteval checking inside FMT_STRING; an empty
# FMT_CONSTEVAL moves the format-string checks to runtime, which is fine for a demo host program.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(fmi4c PRIVATE -Wno-implicit-function-declaration)
  foreach(_ecos_target libecos fmilibcpp)
    if(TARGET ${_ecos_target})
      target_compile_definitions(${_ecos_target} PRIVATE FMT_CONSTEVAL=)
    endif()
  endforeach()
  unset(_ecos_target)
endif()

# Aetherion installs its example FMUs under <prefix>/share/Aetherion/fmu/. The path is baked in as the runtime default;
# --rocket overrides it, so a machine without Aetherion can still build this example and point at a copied FMU at
# runtime.
find_file(
  HEMERION_TWO_STAGE_ROCKET_FMU
  NAMES TwoStageRocket.fmu
  HINTS ENV AETHERION_ROOT
  PATHS "C:/Program Files/Aetherion"
        "C:/Program Files (x86)/Aetherion"
        /usr/local
        /usr
        /opt/aetherion
  PATH_SUFFIXES share/Aetherion/fmu)

if(NOT HEMERION_TWO_STAGE_ROCKET_FMU)
  message(WARNING "TwoStageRocket.fmu not found (set AETHERION_ROOT or install Aetherion) -- "
                  "rocket_gps_cosim will need --rocket <path> at runtime")
  set(HEMERION_TWO_STAGE_ROCKET_FMU "")
endif()

add_executable(rocket_gps_cosim cosim_host_main.cpp)
target_link_libraries(rocket_gps_cosim PRIVATE libecos)
target_compile_features(rocket_gps_cosim PRIVATE cxx_std_23)
target_compile_definitions(
  rocket_gps_cosim
  PRIVATE HEMERION_ROCKET_FMU_PATH="${HEMERION_TWO_STAGE_ROCKET_FMU}"
          HEMERION_GPS_FMU_PATH="${HEMERION_GPS_FMU_ARCHIVE}" HEMERION_IMU_FMU_PATH="${HEMERION_IMU_FMU_ARCHIVE}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # ecos/logger/logger.hpp pulls the bundled fmt into consumers too -- same consteval workaround as above.
  target_compile_definitions(rocket_gps_cosim PRIVATE FMT_CONSTEVAL=)
endif()
add_dependencies(rocket_gps_cosim hemerion_gps_fmu_package hemerion_imu_fmu_package)

add_executable(gps_flight_computer flight_computer_main.cpp udpReceiver.cpp)
target_link_libraries(gps_flight_computer PRIVATE hemerion_sensors)
target_compile_features(gps_flight_computer PRIVATE cxx_std_23)
if(WIN32)
  target_link_libraries(gps_flight_computer PRIVATE ws2_32)
endif()
