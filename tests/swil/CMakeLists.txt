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

add_test(NAME swil.led_blink COMMAND "${Python3_EXECUTABLE}" -m pytest "${CMAKE_CURRENT_SOURCE_DIR}/test_led_blink.py"
                                     -v)
set_tests_properties(
  swil.led_blink
  PROPERTIES LABELS
             "swil"
             WORKING_DIRECTORY
             "${CMAKE_CURRENT_SOURCE_DIR}"
             ENVIRONMENT
             "${_swil_pyrenode_env}")
unset(_swil_pyrenode_env)
