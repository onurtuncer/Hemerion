# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# examples/f16_trim_ecos -- Ecos co-simulation of Aetherion's F16Plant.fmu (NASA TM-2015-218675 check-case 11: F-16
# subsonic trim flyout over Kitty Hawk) feeding all five of Hemerion's hardware-simulator FMUs. The sensor buses
# (UBX-NAV-PVT and radar returns over UDP, shared-memory SPI and I2C) are driven by the same driver stacks that run on
# the STM32H743 flight computer. See README.md.
#
# Two host executables: f16_trim_cosim     -- Ecos master: loads all six FMUs, wires truth -> sensors, logs plant truth
# f16_flight_computer -- drives all five sensor buses with the on-target stacks, logs decoded samples
#
# This example is deliberately self-contained rather than sharing a flight computer with examples/rocket_gps_ecos: each
# example in this directory is meant to be readable end to end, and the two scenarios exercise different sensor sets
# (this one adds the radar altimeter, which the rocket leaves behind within seconds of the pad).
# ------------------------------------------------------------------------------

# The sensor FMUs are exported for both FMI generations; Ecos imports through fmi4c, so this example consumes the FMI
# 2.0 set. generateFMU() names its shared-library targets <modelIdentifier>_<fmiVersion> and packages each .fmu as a
# POST_BUILD step, so depending on the fmi2 targets is what guarantees those archives exist before this example runs.
if(NOT TARGET hemerion_gps_fmu_fmi2
   OR NOT TARGET hemerion_imu_fmu_fmi2
   OR NOT TARGET hemerion_bmp390_fmu_fmi2
   OR NOT TARGET hemerion_mmc5983ma_fmu_fmi2
   OR NOT TARGET hemerion_radalt_fmu_fmi2)
  message(FATAL_ERROR "examples/f16_trim_ecos requires the FMI 2.0 GPS, IMU, BMP390, MMC5983MA and radalt FMU "
                      "targets -- configure with HEMERION_BUILD_FMU=ON (use the examples-native preset)")
endif()

set(HEMERION_GPS_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_gps_fmu.fmu")
set(HEMERION_IMU_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_imu_fmu.fmu")
set(HEMERION_BMP390_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_bmp390_fmu.fmu")
set(HEMERION_MMC5983MA_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_mmc5983ma_fmu.fmu")
set(HEMERION_RADALT_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_radalt_fmu.fmu")

# Ecos is fetched by whichever example directory CMake reaches first; the guard makes this one buildable on its own
# rather than only when examples/rocket_gps_ecos happens to precede it in examples/CMakeLists.txt.
include(FetchContent)

if(NOT TARGET libecos)
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
  # vendored ezxml calls POSIX read() with no declaration in scope on Windows; MSVC merely warns (C4013) but clang
  # treats implicit function declarations as a hard error. - ecos' bundled spdlog/fmt trips clang >= 17's stricter
  # consteval checking inside FMT_STRING; an empty FMT_CONSTEVAL moves the checks to runtime.
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(fmi4c PRIVATE -Wno-implicit-function-declaration)
    foreach(_ecos_target libecos fmilibcpp)
      if(TARGET ${_ecos_target})
        target_compile_definitions(${_ecos_target} PRIVATE FMT_CONSTEVAL=)
      endif()
    endforeach()
    unset(_ecos_target)
  endif()
endif()

# FetchContent sets <name>_SOURCE_DIR in the scope that populated the content, which is the *other* example directory
# whenever that one is configured first. Asking for the properties here recovers the path either way -- without it,
# target_include_directories() below silently receives an empty path and util/unzipper.hpp is not found.
FetchContent_GetProperties(ecos)

# Aetherion installs its example FMUs under <prefix>/share/Aetherion/fmu/. The path is baked in as the runtime default;
# --f16 overrides it, so a machine without Aetherion can still build this example and point at a copied FMU at runtime.
find_file(
  HEMERION_F16_PLANT_FMU
  NAMES F16Plant.fmu
  HINTS ENV AETHERION_ROOT
  PATHS "C:/Program Files/Aetherion"
        "C:/Program Files (x86)/Aetherion"
        /usr/local
        /usr
        /opt/aetherion
  PATH_SUFFIXES share/Aetherion/fmu)

if(NOT HEMERION_F16_PLANT_FMU)
  message(WARNING "F16Plant.fmu not found (set AETHERION_ROOT or install Aetherion) -- "
                  "f16_trim_cosim will need --f16 <path> at runtime")
  set(HEMERION_F16_PLANT_FMU "")
endif()

# Same version floor and the same reasoning as examples/rocket_gps_ecos: the requirement is a set of FMI ports, the
# check reads them out of the archive, and the version number exists only so the diagnostics have something to name.
# Aetherion 0.13.0 put geodetic position into degrees (out.lat_deg/out.lon_deg); 0.14.0 added
# out.specificForce_{x,y,z}_m_s2, the (F_aero + F_thrust)/m at the CG that the IMU FMU's inputs take.
# cosim_host_main.cpp binds both sets by name, so an older plant is a missing variable rather than a unit to compensate
# for.
set(HEMERION_AETHERION_MIN_VERSION 0.16.0)

if(HEMERION_F16_PLANT_FMU)
  # Re-check when the FMU is replaced under an already-configured build tree -- what happens when Aetherion is rebuilt
  # and reinstalled to the same prefix, where find_file()'s cached result would not change.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HEMERION_F16_PLANT_FMU}")

  # An FMU is a zip, and one small member of it answers the question. `cmake -E tar` rather than file(ARCHIVE_EXTRACT)
  # because the pinned cmake-format cannot parse the latter; it is also CMake's own libarchive rather than whichever tar
  # is first on PATH.
  set(_f16_unpack_dir "${CMAKE_CURRENT_BINARY_DIR}/F16Plant.modelDescription")
  file(REMOVE_RECURSE "${_f16_unpack_dir}")
  file(MAKE_DIRECTORY "${_f16_unpack_dir}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E tar xf "${HEMERION_F16_PLANT_FMU}" -- modelDescription.xml
    WORKING_DIRECTORY "${_f16_unpack_dir}"
    RESULT_VARIABLE _f16_unpack_result
    OUTPUT_QUIET ERROR_QUIET)

  set(_f16_description "${_f16_unpack_dir}/modelDescription.xml")
  if(_f16_unpack_result OR NOT EXISTS "${_f16_description}")
    message(FATAL_ERROR "cannot read modelDescription.xml out of ${HEMERION_F16_PLANT_FMU} -- not an FMU?")
  endif()

  # Plain substring searches, not regexes: a variable name occurs exactly once in modelDescription.xml, in the element
  # that declares it, because the FMI dependency graph refers to variables by index rather than by name.
  file(READ "${_f16_description}" _f16_ports)
  string(FIND "${_f16_ports}" "out.lat_deg" _f16_lat_deg_at)
  string(FIND "${_f16_ports}" "out.lat_rad" _f16_lat_rad_at)
  if(_f16_lat_deg_at LESS 0)
    if(_f16_lat_rad_at LESS 0)
      set(_f16_diagnosis "publishes neither out.lat_deg nor out.lat_rad")
    else()
      set(_f16_diagnosis "publishes the pre-0.13.0 out.lat_rad/out.lon_rad, in radians")
    endif()
    message(
      FATAL_ERROR
        "${HEMERION_F16_PLANT_FMU} ${_f16_diagnosis}, but f16_trim_cosim binds out.lat_deg and out.lon_deg. "
        "Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} (https://github.com/onurtuncer/Aetherion) and "
        "reconfigure, or point AETHERION_ROOT at one.")
  endif()

  # Tested separately from the geodetic ports because the two requirements landed in different releases: a 0.13.x plant
  # passes the test above and fails this one, and should be told which port it is missing rather than merely that it is
  # too old.
  string(FIND "${_f16_ports}" "out.specificForce_x_m_s2" _f16_specific_force_at)
  if(_f16_specific_force_at LESS 0)
    message(
      FATAL_ERROR
        "${HEMERION_F16_PLANT_FMU} publishes no out.specificForce_x_m_s2, but f16_trim_cosim binds the three "
        "out.specificForce_* ports into the IMU FMU. Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()

  # Third requirement, third release: 0.16.0 gave the plant an environment. The host writes wind.* and atm.* into the
  # parameter set on every run (their defaults are the calm standard day) and turb.* when --turbulence asks for it, so
  # an older plant fails at load inside Ecos with an unknown-variable error rather than here with a sentence.
  # wind.north_mps stands for the set: they landed together.
  string(FIND "${_f16_ports}" "wind.north_mps" _f16_wind_at)
  if(_f16_wind_at LESS 0)
    message(
      FATAL_ERROR
        "${HEMERION_F16_PLANT_FMU} publishes no wind.north_mps parameter, but f16_trim_cosim writes the "
        "wind.*/atm.* environment into every run. Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()
  unset(_f16_wind_at)

  unset(_f16_unpack_dir)
  unset(_f16_unpack_result)
  unset(_f16_description)
  unset(_f16_ports)
  unset(_f16_lat_deg_at)
  unset(_f16_lat_rad_at)
  unset(_f16_specific_force_at)
endif()

add_executable(f16_trim_cosim cosim_host_main.cpp)
target_link_libraries(f16_trim_cosim PRIVATE libecos)
target_compile_features(f16_trim_cosim PRIVATE cxx_std_23)
# Ecos keeps src/ off its public include path. cosim_host_main.cpp reaches into it for util/unzipper.hpp so its
# FMU-extraction preflight runs the same code Ecos will rather than a reimplementation that could drift.
target_include_directories(f16_trim_cosim PRIVATE "${ecos_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/examples/common")
target_compile_definitions(
  f16_trim_cosim
  PRIVATE HEMERION_F16_FMU_PATH="${HEMERION_F16_PLANT_FMU}"
          HEMERION_GPS_FMU_PATH="${HEMERION_GPS_FMU_ARCHIVE}"
          HEMERION_IMU_FMU_PATH="${HEMERION_IMU_FMU_ARCHIVE}"
          HEMERION_BMP390_FMU_PATH="${HEMERION_BMP390_FMU_ARCHIVE}"
          HEMERION_MMC5983MA_FMU_PATH="${HEMERION_MMC5983MA_FMU_ARCHIVE}"
          HEMERION_RADALT_FMU_PATH="${HEMERION_RADALT_FMU_ARCHIVE}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # ecos/logger/logger.hpp pulls the bundled fmt into consumers too -- same consteval workaround as above.
  target_compile_definitions(f16_trim_cosim PRIVATE FMT_CONSTEVAL=)
endif()
add_dependencies(
  f16_trim_cosim
  hemerion_gps_fmu_fmi2
  hemerion_imu_fmu_fmi2
  hemerion_bmp390_fmu_fmi2
  hemerion_mmc5983ma_fmu_fmi2
  hemerion_radalt_fmu_fmi2)

# hemerion_sensors supplies the on-target stacks (GpsDriver/UbxParser, ImuSpiDriver, ImuPacketParser, convert_raw_to_si,
# Bmp390Driver/Bmp390Compensator, Mmc5983maDriver, RadAltPacketParser); hemerion_spi_shm and hemerion_i2c_shm supply
# only the *board* under the IMU, BMP390 and MMC5983MA drivers -- the controller ends of the shared-memory buses their
# FMUs answer on. The two I2C parts get one bus each rather than sharing one as they would on a real board: sim/i2c_shm
# models a bus with a single peripheral on it. udpReceiver.cpp is the equivalent stand-in for the GPS receiver's and the
# radar altimeter's serial links -- two sockets, one class.
add_executable(f16_flight_computer f16_flight_computer.cpp udpReceiver.cpp)
target_link_libraries(f16_flight_computer PRIVATE hemerion_sensors hemerion_spi_shm hemerion_i2c_shm)
target_compile_features(f16_flight_computer PRIVATE cxx_std_23)
if(WIN32)
  target_link_libraries(f16_flight_computer PRIVATE ws2_32)
endif()
