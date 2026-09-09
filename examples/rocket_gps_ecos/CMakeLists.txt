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
if(NOT TARGET hemerion_gps_fmu_fmi2
   OR NOT TARGET hemerion_imu_fmu_fmi2
   OR NOT TARGET hemerion_bmp390_fmu_fmi2
   OR NOT TARGET hemerion_mmc5983ma_fmu_fmi2)
  message(FATAL_ERROR "examples/rocket_gps_ecos requires the FMI 2.0 GPS, IMU, BMP390 and MMC5983MA FMU targets "
                      "-- configure with HEMERION_BUILD_FMU=ON (use the examples-native preset)")
endif()

set(HEMERION_GPS_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_gps_fmu.fmu")
set(HEMERION_IMU_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_imu_fmu.fmu")
set(HEMERION_BMP390_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_bmp390_fmu.fmu")
set(HEMERION_MMC5983MA_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_mmc5983ma_fmu.fmu")

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

# Aetherion 0.13.0 renamed TwoStageRocket's out.lat_rad/out.lon_rad to out.lat_deg/out.lon_deg and converted them on the
# way out. Before that the FMU forwarded the library's internal radians to the FMI boundary unconverted, so those two
# outputs disagreed with the lat0_deg/lon0_deg parameters that seed the very same quantity on the way in.
# cosim_host_main.cpp binds the degree-valued ports by name and wires them straight into the GPS FMU's degree-valued
# inputs, so an older plant is not a unit mismatch to compensate for here -- it is a variable that does not exist.
#
# The check reads the archive rather than calling find_package(Aetherion) for two reasons. The FMU is the whole of what
# this example consumes, and a version number is only a proxy for it -- AETHERION_ROOT aimed at a stale build tree
# beside a current install is exactly the case a proxy waves through. And Hemerion packages its own FMUs with the
# vendored fmu4cpp under cmake/generate_fmu.cmake, so it has no other need of the Aetherion CMake package; asking for it
# would drag in that package's own REQUIRED Eigen3 and CppAD lookups to read one string.
#
# Aetherion 0.14.0 then added out.specificForce_{x,y,z}_m_s2 -- (F_aero + F_thrust)/m at the CG, in body axes, with
# gravitation structurally excluded. That is what an accelerometer measures and what the IMU FMU's inputs take, so
# cosim_host_main.cpp binds those three ports directly. Before 0.14.0 the plant published the ingredients (out.thrust_N,
# out.mass_kg, out.aero_F*_N) but not the sum, and this host divided them itself -- which meant the bench, not the
# plant, asserting that thrust acts along body +X.
#
# So HEMERION_AETHERION_MIN_VERSION is the release this floor corresponds to, and the one place that number is written
# down for the diagnostics and the docs to quote -- the tests below are against the ports, not against it.
set(HEMERION_AETHERION_MIN_VERSION 0.14.0)

if(HEMERION_TWO_STAGE_ROCKET_FMU)
  # Re-check when the FMU is replaced under an already-configured build tree -- what happens when Aetherion is rebuilt
  # and reinstalled to the same prefix, where find_file()'s cached result would not change.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HEMERION_TWO_STAGE_ROCKET_FMU}")

  # An FMU is a zip, and one small member of it answers the question. Cleared first so that an archive carrying no
  # modelDescription.xml is reported as such rather than silently passing on the copy a previous configure extracted.
  #
  # `cmake -E tar` rather than file(ARCHIVE_EXTRACT) because the pinned cmake-format cannot parse the latter and reflows
  # it to one token per line -- the same 0.6.13 gap that .cmake-format disables lint code E1126 over. It also sidesteps
  # the PATH hazard the runtime side of this example documents, being CMake's own libarchive rather than whichever tar
  # is first on PATH.
  set(_rocket_unpack_dir "${CMAKE_CURRENT_BINARY_DIR}/TwoStageRocket.modelDescription")
  file(REMOVE_RECURSE "${_rocket_unpack_dir}")
  file(MAKE_DIRECTORY "${_rocket_unpack_dir}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E tar xf "${HEMERION_TWO_STAGE_ROCKET_FMU}" -- modelDescription.xml
    WORKING_DIRECTORY "${_rocket_unpack_dir}"
    RESULT_VARIABLE _rocket_unpack_result
    OUTPUT_QUIET ERROR_QUIET)

  set(_rocket_description "${_rocket_unpack_dir}/modelDescription.xml")
  if(_rocket_unpack_result OR NOT EXISTS "${_rocket_description}")
    message(FATAL_ERROR "cannot read modelDescription.xml out of ${HEMERION_TWO_STAGE_ROCKET_FMU} -- not an FMU?")
  endif()

  # Plain substring searches, not regexes, and without the surrounding name="..." -- a variable name occurs exactly once
  # in modelDescription.xml, in the one element that declares it, because the FMI dependency graph refers to variables
  # by index rather than by name. There is nothing else in the file for these to collide with.
  #
  # string(FIND) rather than if(... MATCHES ...) for the same reason as `cmake -E tar` above: cmake-format 0.6.13
  # mis-parses if(NOT <var> <binary-op> <value>) and splays it over four lines. if(<index> LESS 0) is one operator with
  # no leading NOT, which it handles -- and a substring test is what this wanted in the first place.
  file(READ "${_rocket_description}" _rocket_ports)
  string(FIND "${_rocket_ports}" "out.lat_deg" _rocket_lat_deg_at)
  string(FIND "${_rocket_ports}" "out.lat_rad" _rocket_lat_rad_at)
  if(_rocket_lat_deg_at LESS 0)
    if(_rocket_lat_rad_at LESS 0)
      set(_rocket_diagnosis "publishes neither out.lat_deg nor out.lat_rad")
    else()
      set(_rocket_diagnosis "publishes the pre-0.13.0 out.lat_rad/out.lon_rad, in radians")
    endif()
    message(
      FATAL_ERROR
        "${HEMERION_TWO_STAGE_ROCKET_FMU} ${_rocket_diagnosis}, but rocket_gps_cosim binds out.lat_deg and "
        "out.lon_deg. Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()

  # Second port test, same file and same reasoning as the geodetic one above. A 0.13.x plant passes that test and fails
  # this one, which is the whole point of testing ports rather than a version string: the two requirements landed in
  # different releases and the diagnosis should say which one is missing.
  string(FIND "${_rocket_ports}" "out.specificForce_x_m_s2" _rocket_specific_force_at)
  if(_rocket_specific_force_at LESS 0)
    message(
      FATAL_ERROR
        "${HEMERION_TWO_STAGE_ROCKET_FMU} publishes no out.specificForce_x_m_s2, but rocket_gps_cosim binds the "
        "three out.specificForce_* ports into the IMU FMU. Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()

  unset(_rocket_unpack_dir)
  unset(_rocket_unpack_result)
  unset(_rocket_description)
  unset(_rocket_ports)
  unset(_rocket_lat_deg_at)
  unset(_rocket_lat_rad_at)
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
          HEMERION_GPS_FMU_PATH="${HEMERION_GPS_FMU_ARCHIVE}"
          HEMERION_IMU_FMU_PATH="${HEMERION_IMU_FMU_ARCHIVE}"
          HEMERION_BMP390_FMU_PATH="${HEMERION_BMP390_FMU_ARCHIVE}"
          HEMERION_MMC5983MA_FMU_PATH="${HEMERION_MMC5983MA_FMU_ARCHIVE}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # ecos/logger/logger.hpp pulls the bundled fmt into consumers too -- same consteval workaround as above.
  target_compile_definitions(rocket_gps_cosim PRIVATE FMT_CONSTEVAL=)
endif()
add_dependencies(
  rocket_gps_cosim
  hemerion_gps_fmu_fmi2
  hemerion_imu_fmu_fmi2
  hemerion_bmp390_fmu_fmi2
  hemerion_mmc5983ma_fmu_fmi2)

# hemerion_sensors supplies the on-target stacks (GpsDriver/UbxParser, ImuSpiDriver, ImuPacketParser, convert_raw_to_si,
# Bmp390Driver/Bmp390Compensator, Mmc5983maDriver); hemerion_spi_shm and hemerion_i2c_shm supply only the *board* under
# the IMU, BMP390 and MMC5983MA drivers -- the controller ends of the shared-memory buses their FMUs answer on. The two
# I2C parts get one bus each rather than sharing one as they would on a real board: sim/i2c_shm models a bus with a
# single peripheral on it, so multi-drop addressing is the one thing this example cannot exercise. udpReceiver.cpp is
# the equivalent stand-in for the GPS receiver's UART.
add_executable(gps_flight_computer flight_computer_main.cpp udpReceiver.cpp)
target_link_libraries(gps_flight_computer PRIVATE hemerion_sensors hemerion_spi_shm hemerion_i2c_shm)
target_compile_features(gps_flight_computer PRIVATE cxx_std_23)
if(WIN32)
  target_link_libraries(gps_flight_computer PRIVATE ws2_32)
endif()
