# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# hemerion_require_aetherion_plant_version(<fmu_path> <example>): the dynamics floor for an Aetherion plant FMU.
#
# The examples already test the plant FMU's *ports* -- out.lat_deg (0.13.0), out.specificForce_* (0.14.0), wind.* and
# atm.* (0.16.0) -- because a missing port is something a version string can only stand in for. But 0.17.4 and 0.18.0
# fixed the plant's *dynamics* without touching a single port, and no port test can see that:
#
# * 0.17.4 (pre-DOI audit F02/F03): the rigid body took the inertia tensor about the centre of mass as if it were about
#   the body origin, with no parallel-axis term -- TwoStageRocket flew with Iyy short by m DXCG^2, of order 1e7 against
#   3e7 kg m^2 at lift-off -- and gravity scaled with the integrated mass while the mass matrix kept the initial one.
# * 0.18.0 (N01/N02/N07): the gravity wrench is moved from the centre of mass to the body origin before it is summed;
#   F16Plant's restored checkpoints keep the held gust; an FMI start time is counted once.
#
# An older plant therefore runs, and produces figures that disagree with every number this repository publishes, with
# nothing to say why. So this reads the version of the install the FMU came from -- <prefix>/lib/cmake/Aetherion/
# AetherionConfigVersion.cmake beside <prefix>/share/Aetherion/fmu/<name>.fmu -- and refuses one below
# HEMERION_AETHERION_DYNAMICS_VERSION. One read of one string, not find_package(Aetherion), for the reason the examples'
# port tests give: that package's own REQUIRED Eigen3 and CppAD lookups are not this example's business.
#
# An FMU that is not inside an install (copied, or a build tree's) has no version to read. That is reported and let
# through: the port tests still run, and the floor is a statement about what the published figures were made with, not a
# reason to refuse a deliberately pointed-at plant.
# ------------------------------------------------------------------------------

set(HEMERION_AETHERION_DYNAMICS_VERSION 0.18.0)

# Fails configure if the plant FMU at <fmu_path> comes from an Aetherion install older than
# HEMERION_AETHERION_DYNAMICS_VERSION; <example> names the caller in the messages.
function(hemerion_require_aetherion_plant_version fmu_path example)
  get_filename_component(_fmu_dir "${fmu_path}" DIRECTORY) # <prefix>/share/Aetherion/fmu
  get_filename_component(_prefix "${_fmu_dir}/../../.." ABSOLUTE)
  set(_version_file "${_prefix}/lib/cmake/Aetherion/AetherionConfigVersion.cmake")
  if(NOT EXISTS "${_version_file}")
    message(STATUS "${example}: ${fmu_path} is not inside an Aetherion install, so its version cannot be read -- the "
                   "published figures were made with Aetherion ${HEMERION_AETHERION_DYNAMICS_VERSION}")
    return()
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_version_file}")

  # The line is selected by its shape, so the first dotted number on it is the version.
  file(STRINGS "${_version_file}" _line REGEX "^set\\(PACKAGE_VERSION \"[0-9.]+\"\\)" LIMIT_COUNT 1)
  string(
    REGEX MATCH
          "[0-9]+(\\.[0-9]+)*"
          _version
          "${_line}")
  if(NOT _version)
    message(STATUS "${example}: cannot read a version out of ${_version_file}")
    return()
  endif()

  if(_version VERSION_LESS HEMERION_AETHERION_DYNAMICS_VERSION)
    message(
      FATAL_ERROR
        "${fmu_path} is Aetherion ${_version}. Its ports are all there, but its dynamics are not the ones this "
        "example's figures were made with: 0.17.4 corrected the rigid body's spatial inertia (the parallel-axis term) "
        "and its gravity-mass coupling, and 0.18.0 moved the gravity wrench to the body origin -- see "
        "cmake/aetherion_plant_version.cmake. Install Aetherion >= ${HEMERION_AETHERION_DYNAMICS_VERSION} "
        "(https://github.com/onurtuncer/Aetherion) and reconfigure, or point AETHERION_ROOT at one.")
  endif()
  message(STATUS "${example}: plant FMU from Aetherion ${_version}")
endfunction()
