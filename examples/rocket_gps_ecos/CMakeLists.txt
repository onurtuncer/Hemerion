# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# examples/rocket_gps_ecos -- Ecos co-simulation of Aetherion's TwoStageRocket.fmu feeding Hemerion's GPS and IMU
# hardware-simulator FMUs, whose sensor buses (UBX-NAV-PVT over UDP, and a shared-memory SPI bus) are driven by the same
# GpsDriver/UbxParser and ImuSpiDriver/ImuPacketParser stacks that run on the STM32H743 flight computer. See README.md
# and doc/rocket_gps_ecos_cosim.rst.
#
# Two host executables: rocket_gps_cosim    -- Ecos master: loads all three FMUs, wires truth -> sensors, logs rocket
# truth to CSV gps_flight_computer -- drives both sensor buses with the on-target stacks, logs decoded fixes and samples
#
# The Ecos platform library (github.com/Ecos-platform/ecos) is fetched and built from source here; its own CMake fetches
# everything else it needs (fmi4c for FMU loading, header-only utilities). CLI/C-bindings/proxyfmu are switched off --
# only the C++ library target `libecos` is consumed.
# ------------------------------------------------------------------------------

# The sensor FMUs are exported for both FMI generations; Ecos imports through fmi4c, so this example consumes the FMI
# 2.0 pair. generateFMU() names its shared-library targets <modelIdentifier>_<fmiVersion> and packages each .fmu as a
# POST_BUILD step, so depending on the fmi2 targets is what guarantees those archives exist before this example runs.
if(NOT TARGET hemerion_gps_fmu_fmi2 OR NOT TARGET hemerion_imu_fmu_fmi2)
  message(FATAL_ERROR "examples/rocket_gps_ecos requires the FMI 2.0 GPS and IMU FMU targets -- configure with "
                      "HEMERION_BUILD_FMU=ON (use the examples-native preset)")
endif()

set(HEMERION_GPS_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_gps_fmu.fmu")
set(HEMERION_IMU_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_imu_fmu.fmu")

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
# Ecos keeps src/ off its public include path. cosim_host_main.cpp reaches into it for util/unzipper.hpp so its
# FMU-extraction preflight runs the same code Ecos will rather than a reimplementation that could drift -- see the
# comment on that include. Ecos is pinned, so a moved header fails the build instead of rotting the check.
target_include_directories(rocket_gps_cosim PRIVATE "${ecos_SOURCE_DIR}/src")
target_compile_definitions(
  rocket_gps_cosim
  PRIVATE HEMERION_ROCKET_FMU_PATH="${HEMERION_TWO_STAGE_ROCKET_FMU}"
          HEMERION_GPS_FMU_PATH="${HEMERION_GPS_FMU_ARCHIVE}" HEMERION_IMU_FMU_PATH="${HEMERION_IMU_FMU_ARCHIVE}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # ecos/logger/logger.hpp pulls the bundled fmt into consumers too -- same consteval workaround as above.
  target_compile_definitions(rocket_gps_cosim PRIVATE FMT_CONSTEVAL=)
endif()
add_dependencies(rocket_gps_cosim hemerion_gps_fmu_fmi2 hemerion_imu_fmu_fmi2)

# hemerion_sensors supplies the on-target stacks (GpsDriver/UbxParser, ImuSpiDriver, ImuPacketParser,
# convert_raw_to_si); hemerion_spi_shm supplies only the *board* under the IMU driver -- the controller end of the
# shared-memory SPI bus the IMU FMU answers on. udpReceiver.cpp is the equivalent stand-in for the GPS receiver's UART.
add_executable(gps_flight_computer flight_computer_main.cpp udpReceiver.cpp)
target_link_libraries(gps_flight_computer PRIVATE hemerion_sensors hemerion_spi_shm)
target_compile_features(gps_flight_computer PRIVATE cxx_std_23)
if(WIN32)
  target_link_libraries(gps_flight_computer PRIVATE ws2_32)
endif()
