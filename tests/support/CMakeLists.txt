# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Scaffolding shared by every native test executable in the tree -- the per-module ones under modules/*/test and
# sim/*/test as well as anything added under tests/. See include/hemerion/test/fail_fast.h for what it installs and why
# it installs itself rather than asking each test to call it.
#
# OBJECT, not STATIC, is load-bearing: the guard is a namespace-scope initializer that nothing references by name, and a
# static library's members are pulled in only to satisfy an undefined symbol. The object file must be compiled into each
# test executable unconditionally or the guard would quietly not be there.
#
# Added from the top-level CMakeLists.txt before sim/ and modules/, since their test/ subdirectories link this.
# ------------------------------------------------------------------------------

add_library(hemerion_test_support OBJECT fail_fast.cpp)

target_include_directories(hemerion_test_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
