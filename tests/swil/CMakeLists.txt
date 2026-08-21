# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# SWIL tests -- driven by pytest + pyrenode3 against a real Renode instance (see tests/README.md). Both are optional
# system dependencies pytest/ctest can't install for you, so this file degrades to a configure-time message rather than
# a hard error when either is missing -- test-native and fmu-native must keep configuring cleanly on machines without
# Renode.
# ------------------------------------------------------------------------------

find_package(Renode 1.15)
find_package(Pyrenode3)

if(NOT Renode_FOUND OR NOT Pyrenode3_FOUND)
  message(STATUS "tests/swil: skipping -- requires both Renode and pyrenode3 (see tests/swil/requirements.txt)")
  return()
endif()

# pyrenode3 cannot introspect the /usr/bin/renode shell launcher the Renode .deb installs (see tests/README.md's "Setup:
# Renode + pyrenode3" section) -- when the deb's /opt/renode CoreCLR layout is present, point pyrenode3 at the build
# output directory instead. PYRENODE_BIN stays as the fallback for installs where Renode_EXECUTABLE is a real binary
# pyrenode3 can load. RENODE_ROOT takes precedence so a non-default install location keeps working.
if(DEFINED ENV{RENODE_ROOT} AND EXISTS "$ENV{RENODE_ROOT}/bin/Renode.dll")
  set(_swil_pyrenode_env "PYRENODE_BUILD_DIR=$ENV{RENODE_ROOT};PYRENODE_BUILD_OUTPUT=bin")
elseif(EXISTS "/opt/renode/bin/Renode.dll")
  set(_swil_pyrenode_env "PYRENODE_BUILD_DIR=/opt/renode;PYRENODE_BUILD_OUTPUT=bin")
else()
  set(_swil_pyrenode_env "PYRENODE_BIN=${Renode_EXECUTABLE}")
endif()

# conftest.firmware_elf() used to hardcode build/renode-h743, which the test-swil preset CI builds (binaryDir
# build/test-swil) never populates -- every SWIL test skipped there, and ctest scores a pytest skip as a pass, so the
# job was green having run nothing. Handing the harness this build's own apps/ directory ties the tests to whichever
# cross preset registered them. HEMERION_SWIL_TOOLS_DIR and HEMERION_SWIL_STRICT are deliberately not set here: they are
# the caller's, and ENVIRONMENT adds to the inherited environment rather than replacing it.
set(_swil_env "${_swil_pyrenode_env};HEMERION_SWIL_FIRMWARE_DIR=${CMAKE_BINARY_DIR}/apps")

add_test(NAME swil.led_blink COMMAND "${Python3_EXECUTABLE}" -m pytest "${CMAKE_CURRENT_SOURCE_DIR}/test_led_blink.py"
                                     -v)
set_tests_properties(
  swil.led_blink
  PROPERTIES LABELS
             "swil"
             WORKING_DIRECTORY
             "${CMAKE_CURRENT_SOURCE_DIR}"
             ENVIRONMENT
             "${_swil_env}"
             TIMEOUT
             300)

# The whole BMP390 SWIL loop: emulated firmware -> Renode STM32F7_I2C -> runtime-loaded C# bridge -> TCP ->
# i2c_shm_tcp_bridge -> shared-memory bus -> the register-accurate part model. Skips inside pytest when the firmware or
# the host tools are missing -- the latter come from a *native* build (sim/ is host-only, so this cross build cannot
# produce them), pointed at by HEMERION_SWIL_TOOLS_DIR; set HEMERION_SWIL_STRICT to turn those skips into failures.
#
# The TIMEOUT is explicit because this test drives four processes across a TCP hop: if the bridge or the part wedges,
# the emulated CPU thread blocks with Renode's virtual clock stopped, so the pytest-side 30 s TerminalTester budget --
# counted in *virtual* seconds -- never elapses. Without this, ctest's 1500 s default is what ends the job.
add_test(NAME swil.baro_logger COMMAND "${Python3_EXECUTABLE}" -m pytest
                                       "${CMAKE_CURRENT_SOURCE_DIR}/test_baro_logger.py" -v)
set_tests_properties(
  swil.baro_logger
  PROPERTIES LABELS
             "swil"
             WORKING_DIRECTORY
             "${CMAKE_CURRENT_SOURCE_DIR}"
             ENVIRONMENT
             "${_swil_env}"
             TIMEOUT
             300)

# The same loop for the MMC5983MA, and a strictly stronger one: the barometer is configured once and then read, while
# this part is *commanded* -- its bring-up is a blocking SET / measure / RESET / measure / SET handshake with the
# emulated core parked on the result, so every layer of the chain has to deliver a multi-transaction, stateful sequence
# in order rather than just a register burst. The test asserts the bridge offset the firmware recovers equals the one
# the peripheral independently reports being born with, which is the only check that distinguishes a working calibration
# from a firmware that skipped it and is reporting its own offset as field.
#
# Same explicit TIMEOUT as swil.baro_logger, for the same reason: four processes across a TCP hop, and a wedge stops
# Renode's virtual clock so the pytest-side budget never elapses.
add_test(NAME swil.mag_logger COMMAND "${Python3_EXECUTABLE}" -m pytest
                                      "${CMAKE_CURRENT_SOURCE_DIR}/test_mag_logger.py" -v)
set_tests_properties(
  swil.mag_logger
  PROPERTIES LABELS
             "swil"
             WORKING_DIRECTORY
             "${CMAKE_CURRENT_SOURCE_DIR}"
             ENVIRONMENT
             "${_swil_env}"
             TIMEOUT
             300)
unset(_swil_env)
unset(_swil_pyrenode_env)
