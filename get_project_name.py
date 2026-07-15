# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Extract the project name from a top-level CMakeLists.txt.

Used by doc/conf.py so Sphinx and CMake share a single source of truth for
the project name.
"""

import re


def get_project_name(cmake_path):
    """Return the first argument of the project() call in cmake_path."""
    with open(cmake_path, "r", encoding="utf-8") as f:
        content = f.read()
    match = re.search(r"^\s*project\s*\(\s*([A-Za-z0-9_.-]+)", content, re.MULTILINE)
    if match is None:
        raise ValueError(f"No project() call found in {cmake_path}")
    return match.group(1)
