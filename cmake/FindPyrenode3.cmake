# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# FindPyrenode3.cmake
#
# Locates the pyrenode3 Python package (https://github.com/antmicro/pyrenode3), the binding tests/swil/conftest.py uses
# to drive Renode from pytest.
#
# Usage: find_package(Pyrenode3 [REQUIRED]) Exposes: Pyrenode3_FOUND, Pyrenode3_VERSION, Python3_EXECUTABLE (via the
# Python3 find-module this depends on).
#
# This module only confirms the package is importable by the discovered Python interpreter -- it does not also require
# Renode itself, since tests/unit and tests/integration build fine without either. tests/swil needs both
# find_package(Pyrenode3) and find_package(Renode).
# ------------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

find_package(Python3 QUIET COMPONENTS Interpreter)

if(Python3_Interpreter_FOUND)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import importlib.metadata as m; print(m.version('pyrenode3'))"
    OUTPUT_VARIABLE Pyrenode3_VERSION
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pyrenode3_result)
  if(NOT
     _pyrenode3_result
     EQUAL
     0)
    unset(Pyrenode3_VERSION)
  endif()
  unset(_pyrenode3_result)
endif()

find_package_handle_standard_args(Pyrenode3 REQUIRED_VARS Python3_EXECUTABLE Pyrenode3_VERSION
                                  VERSION_VAR Pyrenode3_VERSION)
