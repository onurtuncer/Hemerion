# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for sim/udp_bridge. Only added when HEMERION_BUILD_TESTS
# is ON (test-native preset) -- see sim/udp_bridge/CMakeLists.txt. Both
# binaries run single-process: they bind two sockets on 127.0.0.1 to exercise
# the protocol without spawning a second process.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_udp_bridge_socket test_udp_socket.cpp)
target_link_libraries(test_hemerion_udp_bridge_socket PRIVATE hemerion_udp_bridge)

add_executable(test_hemerion_udp_bridge_bridge test_udp_bridge.cpp)
target_link_libraries(test_hemerion_udp_bridge_bridge PRIVATE hemerion_udp_bridge)

add_test(NAME udp_bridge.socket COMMAND test_hemerion_udp_bridge_socket)
set_tests_properties(udp_bridge.socket PROPERTIES LABELS "unit")

add_test(NAME udp_bridge.bridge COMMAND test_hemerion_udp_bridge_bridge)
set_tests_properties(udp_bridge.bridge PROPERTIES LABELS "unit")
