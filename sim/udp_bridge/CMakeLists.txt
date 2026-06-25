# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# sim/udp_bridge -- UDP transport for the FMI master when Aetherion runs
# out-of-process or on a different host (see package.xml). Host-only: this
# directory is only reached when HEMERION_BUILD_SIM is ON, which the root
# CMakeLists.txt already guards against CMAKE_CROSSCOMPILING. Implemented on
# both POSIX (BSD sockets) and Windows (Winsock), selected at compile time in
# src/udp_socket.cpp.
# ------------------------------------------------------------------------------

add_library(hemerion_udp_bridge STATIC src/udp_socket.cpp src/udp_bridge.cpp)

target_include_directories(hemerion_udp_bridge PUBLIC include)
target_compile_features(hemerion_udp_bridge PUBLIC cxx_std_23)

if(WIN32)
  target_link_libraries(hemerion_udp_bridge PRIVATE ws2_32)
endif()

if(HEMERION_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
