# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Native unit tests for sim/spi_shm. Only added when HEMERION_BUILD_TESTS is ON (test-native preset) -- see
# sim/spi_shm/CMakeLists.txt. The binary runs single-process: it opens both ends of the same named bus to exercise the
# handshake and the peripheral service thread without spawning a second process.
# ------------------------------------------------------------------------------

add_executable(test_hemerion_spi_shm_link test_spi_shm_link.cpp)
target_link_libraries(test_hemerion_spi_shm_link PRIVATE hemerion_spi_shm)

add_test(NAME spi_shm.link COMMAND test_hemerion_spi_shm_link)
set_tests_properties(spi_shm.link PROPERTIES LABELS "unit")
