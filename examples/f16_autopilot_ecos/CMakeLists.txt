# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# examples/f16_autopilot_ecos -- Ecos co-simulation of NASA TM-2015-218675's four closed-loop autopilot check-cases
# (13.1-13.4): Aetherion's F16Plant.fmu and F16Autopilot.fmu in a loop, with all five Hemerion sensor FMUs riding the
# same truth. See README.md.
#
# One host executable: f16_autopilot_cosim. The flight computer is examples/f16_trim_ecos's f16_flight_computer, reused
# rather than copied -- the sensor complement, the buses and the trim condition are identical, so a second copy would
# only be a second place for them to drift. The geomagnetic dipole header is included from there for the same reason.
# ------------------------------------------------------------------------------

if(NOT TARGET hemerion_gps_fmu_fmi2
   OR NOT TARGET hemerion_imu_fmu_fmi2
   OR NOT TARGET hemerion_bmp390_fmu_fmi2
   OR NOT TARGET hemerion_mmc5983ma_fmu_fmi2
   OR NOT TARGET hemerion_radalt_fmu_fmi2)
  message(FATAL_ERROR "examples/f16_autopilot_ecos requires the FMI 2.0 GPS, IMU, BMP390, MMC5983MA and radalt FMU "
                      "targets -- configure with HEMERION_BUILD_FMU=ON (use the examples-native preset)")
endif()

set(HEMERION_GPS_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_gps_fmu.fmu")
set(HEMERION_IMU_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_imu_fmu.fmu")
set(HEMERION_BMP390_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_bmp390_fmu.fmu")
set(HEMERION_MMC5983MA_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_mmc5983ma_fmu.fmu")
set(HEMERION_RADALT_FMU_ARCHIVE "${HEMERION_FMU_OUTPUT_DIR}/fmi2/hemerion_radalt_fmu.fmu")

# Ecos is fetched by whichever example directory CMake reaches first; the guard makes this one buildable on its own.
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

  # Same toolchain workarounds for the fetched sources as the sibling examples.
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

FetchContent_GetProperties(ecos)

# Aetherion installs its example FMUs under <prefix>/share/Aetherion/fmu/. Both paths are baked in as runtime defaults;
# --f16 / --ap override them, so a machine without Aetherion can still build this example.
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

find_file(
  HEMERION_F16_AUTOPILOT_FMU
  NAMES F16Autopilot.fmu
  HINTS ENV AETHERION_ROOT
  PATHS "C:/Program Files/Aetherion"
        "C:/Program Files (x86)/Aetherion"
        /usr/local
        /usr
        /opt/aetherion
  PATH_SUFFIXES share/Aetherion/fmu)

if(NOT HEMERION_F16_PLANT_FMU)
  message(WARNING "F16Plant.fmu not found (set AETHERION_ROOT or install Aetherion) -- "
                  "f16_autopilot_cosim will need --f16 <path> at runtime")
  set(HEMERION_F16_PLANT_FMU "")
endif()
if(NOT HEMERION_F16_AUTOPILOT_FMU)
  message(WARNING "F16Autopilot.fmu not found (set AETHERION_ROOT or install Aetherion) -- "
                  "f16_autopilot_cosim will need --ap <path> at runtime")
  set(HEMERION_F16_AUTOPILOT_FMU "")
endif()

# The requirement is a set of FMI ports, tested by reading them out of each archive -- same approach and same helper
# pattern as examples/f16_trim_ecos, one macro because there are now two archives to interrogate.
set(HEMERION_AETHERION_MIN_VERSION 0.16.0)

# Reads modelDescription.xml out of ${archive} into ${out_var}. `cmake -E tar` for the reasons documented in
# examples/f16_trim_ecos/CMakeLists.txt (the pinned cmake-format cannot parse file(ARCHIVE_EXTRACT)).
function(
  hemerion_read_model_description
  archive
  workdir
  out_var)
  file(REMOVE_RECURSE "${workdir}")
  file(MAKE_DIRECTORY "${workdir}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E tar xf "${archive}" -- modelDescription.xml
    WORKING_DIRECTORY "${workdir}"
    RESULT_VARIABLE _unpack_result
    OUTPUT_QUIET ERROR_QUIET)
  if(_unpack_result OR NOT EXISTS "${workdir}/modelDescription.xml")
    message(FATAL_ERROR "cannot read modelDescription.xml out of ${archive} -- not an FMU?")
  endif()
  file(READ "${workdir}/modelDescription.xml" _content)
  set(${out_var} "${_content}" PARENT_SCOPE)
endfunction()

if(HEMERION_F16_PLANT_FMU)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HEMERION_F16_PLANT_FMU}")
  hemerion_read_model_description("${HEMERION_F16_PLANT_FMU}" "${CMAKE_CURRENT_BINARY_DIR}/F16Plant.modelDescription"
                                  _f16_ports)
  # Same two floors as the trim example: degrees position (0.13.0) and specific force (0.14.0)...
  string(FIND "${_f16_ports}" "out.lat_deg" _f16_lat_deg_at)
  string(FIND "${_f16_ports}" "out.specificForce_x_m_s2" _f16_specific_force_at)
  # ...plus the loop itself: the four surface inputs the autopilot drives...
  string(FIND "${_f16_ports}" "ctrl.el_deg" _f16_ctrl_at)
  # ...plus the environment (0.16.0), which the host writes into the parameter set on every run: wind.* and atm.*
  # always, turb.* when --turbulence asks for it. wind.north_mps stands for the set; they landed together.
  string(FIND "${_f16_ports}" "wind.north_mps" _f16_wind_at)
  if(_f16_lat_deg_at LESS 0
     OR _f16_specific_force_at LESS 0
     OR _f16_ctrl_at LESS 0
     OR _f16_wind_at LESS 0)
    message(
      FATAL_ERROR
        "${HEMERION_F16_PLANT_FMU} does not publish every port this example binds (out.lat_deg, "
        "out.specificForce_x_m_s2, ctrl.el_deg, wind.north_mps). Install Aetherion >= "
        "${HEMERION_AETHERION_MIN_VERSION} (https://github.com/onurtuncer/Aetherion) and reconfigure, or point "
        "AETHERION_ROOT at one.")
  endif()
  unset(_f16_ports)
  unset(_f16_lat_deg_at)
  unset(_f16_specific_force_at)
  unset(_f16_ctrl_at)
  unset(_f16_wind_at)
endif()

if(HEMERION_F16_AUTOPILOT_FMU)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HEMERION_F16_AUTOPILOT_FMU}")
  hemerion_read_model_description("${HEMERION_F16_AUTOPILOT_FMU}"
                                  "${CMAKE_CURRENT_BINARY_DIR}/F16Autopilot.modelDescription" _ap_ports)
  # One representative of each variable group the host binds: a command input, a feedback input, a surface output.
  string(FIND "${_ap_ports}" "cmd.altCmd_ft" _ap_cmd_at)
  string(FIND "${_ap_ports}" "fb.alt_m" _ap_fb_at)
  string(FIND "${_ap_ports}" "ctrl.el_deg" _ap_ctrl_at)
  if(_ap_cmd_at LESS 0 OR _ap_fb_at LESS 0 OR _ap_ctrl_at LESS 0)
    message(
      FATAL_ERROR
        "${HEMERION_F16_AUTOPILOT_FMU} does not publish every port this example binds (cmd.altCmd_ft, fb.alt_m, "
        "ctrl.el_deg). Install Aetherion >= ${HEMERION_AETHERION_MIN_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()
  unset(_ap_ports)
  unset(_ap_cmd_at)
  unset(_ap_fb_at)
  unset(_ap_ctrl_at)
endif()

add_executable(f16_autopilot_cosim cosim_host_main.cpp)
target_link_libraries(f16_autopilot_cosim PRIVATE libecos)
target_compile_features(f16_autopilot_cosim PRIVATE cxx_std_23)
# Ecos' src/ for util/unzipper.hpp (the extraction preflight), and the sibling example for geomagnetic_field.hpp --
# shared, not copied; see the header comment in cosim_host_main.cpp.
target_include_directories(f16_autopilot_cosim PRIVATE "${ecos_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/examples/common"
                                                       "${CMAKE_CURRENT_SOURCE_DIR}/../f16_trim_ecos")
target_compile_definitions(
  f16_autopilot_cosim
  PRIVATE HEMERION_F16_FMU_PATH="${HEMERION_F16_PLANT_FMU}"
          HEMERION_AP_FMU_PATH="${HEMERION_F16_AUTOPILOT_FMU}"
          HEMERION_GPS_FMU_PATH="${HEMERION_GPS_FMU_ARCHIVE}"
          HEMERION_IMU_FMU_PATH="${HEMERION_IMU_FMU_ARCHIVE}"
          HEMERION_BMP390_FMU_PATH="${HEMERION_BMP390_FMU_ARCHIVE}"
          HEMERION_MMC5983MA_FMU_PATH="${HEMERION_MMC5983MA_FMU_ARCHIVE}"
          HEMERION_RADALT_FMU_PATH="${HEMERION_RADALT_FMU_ARCHIVE}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_definitions(f16_autopilot_cosim PRIVATE FMT_CONSTEVAL=)
endif()
add_dependencies(
  f16_autopilot_cosim
  hemerion_gps_fmu_fmi2
  hemerion_imu_fmu_fmi2
  hemerion_bmp390_fmu_fmi2
  hemerion_mmc5983ma_fmu_fmi2
  hemerion_radalt_fmu_fmi2)
