# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Unit tests for host-side code that does not belong to a module.
#
# examples/ cannot be tested where it lives: every example needs an Aetherion plant FMU, and the preset CI runs ctest
# under (test-native) builds no examples at all. So the examples' pure logic -- a formula out of MIL-F-8785C, an option
# parser, and the two GPS receiver presets the documentation quotes -- lives in a header with no FMI, no Ecos and no
# sockets, and is covered here instead. The co-simulation smoke tests stay in examples/, registered only where a plant
# FMU was found, so their absence in CI is visible in the test count rather than scored as a pass.
# ------------------------------------------------------------------------------

# Host-only. The cross (test-swil) build compiles with -fno-exceptions, and examples/common/environment.hpp reports a
# bad --wind or --atmosphere by throwing std::invalid_argument -- the option parser for a desktop co-simulation host,
# which is not code that will ever run on the STM32H743. Same gate, and the same reason, as apps/can_actuator_link.
if(CMAKE_CROSSCOMPILING)
  return()
endif()

add_executable(test_hemerion_example_environment test_example_environment.cpp)
target_include_directories(test_hemerion_example_environment PRIVATE "${CMAKE_SOURCE_DIR}/examples/common")
target_link_libraries(test_hemerion_example_environment PRIVATE hemerion_test_support)

add_test(NAME examples.environment COMMAND test_hemerion_example_environment)
set_tests_properties(examples.environment PROPERTIES LABELS "unit")

# The log contract with Aetherion's replay (sensor fusion step 2): the names the examples write, against a pinned copy
# of the names Aetherion v0.18.0 binds. Pure, so it runs in CI like the test above.
add_executable(test_hemerion_log_contract test_log_contract.cpp)
target_include_directories(test_hemerion_log_contract PRIVATE "${CMAKE_SOURCE_DIR}/examples/common")
target_link_libraries(test_hemerion_log_contract PRIVATE hemerion_test_support)

add_test(NAME examples.log_contract COMMAND test_hemerion_log_contract)
set_tests_properties(examples.log_contract PROPERTIES LABELS "unit")

# The pinned copy against the real thing, where an Aetherion install is around to read it from. Registered only then, so
# its absence in CI shows in the test count rather than scoring a skip as a pass.
find_file(
  HEMERION_AETHERION_LOG_CONTRACT
  NAMES Aetherion/Estimation/LogContract.h
  HINTS "${AETHERION_ROOT}" "$ENV{AETHERION_ROOT}"
  PATHS "$ENV{ProgramFiles}/Aetherion" "/usr/local" "/opt/aetherion"
  PATH_SUFFIXES include)
find_package(Python3 COMPONENTS Interpreter QUIET)
if(HEMERION_AETHERION_LOG_CONTRACT AND Python3_Interpreter_FOUND)
  add_test(NAME examples.log_contract_upstream
           COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/scripts/check_log_contract.py" --aetherion-header
                   "${HEMERION_AETHERION_LOG_CONTRACT}")
  set_tests_properties(examples.log_contract_upstream PROPERTIES LABELS "unit")
else()
  message(STATUS "Aetherion LogContract.h not found (set AETHERION_ROOT) -- examples.log_contract_upstream not added")
endif()
