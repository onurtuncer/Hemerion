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

add_test(NAME swil.led_blink COMMAND "${Python3_EXECUTABLE}" -m pytest "${CMAKE_CURRENT_SOURCE_DIR}/test_led_blink.py"
                                     -v)
set_tests_properties(
  swil.led_blink
  PROPERTIES LABELS
             "swil"
             WORKING_DIRECTORY
             "${CMAKE_CURRENT_SOURCE_DIR}"
             ENVIRONMENT
             "PYRENODE_BIN=${Renode_EXECUTABLE}")
