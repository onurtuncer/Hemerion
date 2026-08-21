# ------------------------------------------------------------------------------
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Configuration file for the Sphinx documentation builder.

import os
import re
import sys

# Step 1: Add parent directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Local Sphinx extensions live in doc/_ext.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '_ext')))

# Step 2: Import the function
from get_project_name import get_project_name

# Step 3: Compute absolute path to CMakeLists.txt
cmake_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'CMakeLists.txt'))

# Step 4: Call the function
project_name = get_project_name(cmake_path)
print(f"Project name from top level: {project_name}")

# -- Project information -----------------------------------------------------

# Single source of truth for the project version is package.xml (same as the
# top-level CMakeLists.txt).
package_xml = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'package.xml'))
with open(package_xml, 'r', encoding='utf-8') as _f:
    _version_match = re.search(r'<package[^>]* version="([0-9]+\.[0-9]+\.[0-9]+)"', _f.read())
if _version_match is None:
    raise ValueError(f'Could not find version="X.Y.Z" in {package_xml}')
_project_version = _version_match.group(1)

project = project_name
author = 'Onur Tuncer, PhD'
copyright = '2025-2026, Onur Tuncer, PhD'
version = _project_version
release = _project_version

# -- General configuration ---------------------------------------------------

extensions = [
    'breathe',
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.graphviz',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinxcontrib.bibtex',
    # Local, in doc/_ext -- see the module docstring for what it repairs.
    'breathe_dashes',
]

source_encoding = 'utf-8'
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Metrix++ complexity report ----------------------------------------------

# doc/generated/metrixpp_report.rst is rewritten on every build so the report
# published to GitHub Pages describes the sources it ships with. With metrix++
# missing the generator writes a placeholder page instead of failing the build.
_doc_dir = os.path.abspath(os.path.dirname(__file__))
if _doc_dir not in sys.path:
    sys.path.insert(0, _doc_dir)

from generate_metrixpp_report import write_report

_repo_root = os.path.abspath(os.path.join(_doc_dir, '..'))
write_report(
    repo_root=_repo_root,
    out_path=os.path.join(_doc_dir, 'generated', 'metrixpp_report.rst'),
    work_dir=os.path.join(_repo_root, 'build', 'metrixpp'),
)

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_title = f'{project} {release}'

html_theme_options = {
    # The API reference nests four levels deep (section -> namespace -> class
    # -> member); the theme's default of 4 collapses the last of them.
    'navigation_depth': 5,
    'collapse_navigation': False,
    'sticky_navigation': True,
    'titles_only': False,
}

# -- Doxygen XML -------------------------------------------------------------
#
# Breathe renders the C++ API pages under api/ from Doxygen's XML output. The
# XML has to exist before sphinx-build runs, and there are three ways in which
# it comes to:
#
#   * the CMake doc target and the GitHub Pages workflow run Doxygen first and
#     point Breathe at the result with -Dbreathe_projects.Hemerion=...;
#   * a bare `sphinx-build doc/ build/sphinx` (or a ReadTheDocs build) has no
#     such step, so this file runs Doxygen itself;
#   * HEMERION_DOXYGEN_XML overrides the location outright.
#
# The fallback matters more than convenience: without it a plain sphinx-build
# still succeeds, silently publishing every API page as an empty stub.

# _repo_root is set above, alongside the Metrix++ report generation.
_doxygen_xml = os.environ.get(
    'HEMERION_DOXYGEN_XML',
    os.path.join(_repo_root, 'build', 'doxygen', 'xml'),
)


def _run_doxygen(output_dir):
    """Generate Doxygen XML into output_dir from doc/Doxyfile.in."""
    import subprocess

    doxyfile_in = os.path.join(os.path.dirname(__file__), 'Doxyfile.in')
    doxygen_root = os.path.dirname(output_dir)
    os.makedirs(doxygen_root, exist_ok=True)

    with open(doxyfile_in, 'r', encoding='utf-8') as f:
        config = f.read()
    for placeholder, value in (
        ('@CMAKE_PROJECT_NAME@', project),
        ('@HEMERION_VERSION@', release),
        ('@CMAKE_SOURCE_DIR@', _repo_root.replace(os.sep, '/')),
        ('@CMAKE_BINARY_DIR@', os.path.join(_repo_root, 'build').replace(os.sep, '/')),
        ('@DOXYGEN_OUTPUT_DIR@', doxygen_root.replace(os.sep, '/')),
    ):
        config = config.replace(placeholder, value)

    generated = os.path.join(doxygen_root, 'Doxyfile')
    with open(generated, 'w', encoding='utf-8') as f:
        f.write(config)
    subprocess.run(['doxygen', generated], check=True)


if not os.path.isfile(os.path.join(_doxygen_xml, 'index.xml')):
    print(f'Doxygen XML not found at {_doxygen_xml}; running Doxygen')
    _run_doxygen(_doxygen_xml)

# -- Breathe configuration ---------------------------------------------------

breathe_projects = {project: _doxygen_xml}
breathe_default_project = project

# Doxygen names these headers .h even though they are C++ (only bsp/'s HAL is C,
# and it is a plain function API that reads the same either way). Without this
# mapping Breathe would hand .h files to the C domain, where the namespaces and
# classes that make up most of this API have no representation at all.
breathe_domain_by_extension = {
    'h': 'cpp',
    'hpp': 'cpp',
    'cc': 'cpp',
    'cpp': 'cpp',
}

# Document members by default so the api/ pages do not have to repeat
# :members: on every directive. Private members are already filtered out at
# the Doxygen level (EXTRACT_PRIVATE = NO).
breathe_default_members = ('members',)

# Register maps and protocol constants are only meaningful with their values --
# an enumerator named kCalibNvm says nothing without the 0x31 next to it.
breathe_show_enumvalue_initializer = True

# Show the #include line a declaration is reached through.
breathe_show_include = True

# Whether to add '()' to function entries in the index and elsewhere
add_function_parentheses = True

# -- Diagrams ----------------------------------------------------------------

# The hand-drawn figures under api/ are mostly boxes of text, which vector
# output keeps legible at any zoom and on a high-DPI screen. Doxygen's own
# automatic graphs are switched off (HAVE_DOT = NO): with only four one-level
# inheritance relationships in the tree, and base classes already named and
# cross-linked in every signature, they would restate the line above them.
graphviz_output_format = 'svg'

# -- C++ domain --------------------------------------------------------------

primary_domain = 'cpp'
highlight_language = 'cpp'

# Sort index entries under the name rather than under 'h' for hemerion.
cpp_index_common_prefix = ['hemerion::']

# API entries appear in the page-local sidebar, but as `FaultRegistry` rather
# than `hemerion::fault::FaultRegistry` -- the fully qualified names are wider
# than the sidebar and all share a prefix anyway.
toc_object_entries_show_parents = 'hide'

# Breathe emits the enclosing namespace scope for every doxygenfile directive,
# so a page documenting ten headers from hemerion::sensors::baro declares that
# namespace ten times and the C++ domain reports each repeat as a duplicate.
# They are not: no entity is documented twice, which doc/check_api_coverage.py
# asserts directly by refusing to let two directives name the same header.
# Suppressing the category here keeps -W usable for warnings that do mean
# something. Note this covers only the C++ domain; the matching docutils
# duplicate-target notices are filtered in the docs workflow instead, since
# suppressing 'docutils' wholesale would hide unrelated markup errors.
suppress_warnings = ['duplicate_declaration.cpp']

# -- MathJax macros ----------------------------------------------------------

mathjax3_config = {
    "tex": {
        "macros": {
            "SO":     r"\mathrm{SO}",
            "SE":     r"\mathrm{SE}",
            "so":     r"\mathfrak{so}",
            "R":      r"\mathbb{R}",
            "Exp":    r"\mathrm{Exp}",
            "Log":    r"\mathrm{Log}",
            "Lie":    [r"\mathfrak{#1}", 1],
            "dexp":   r"\mathrm{dexp}",
            "ad":     r"\operatorname{ad}",
            "norm":   [r"\left\lVert #1 \right\rVert", 1],
            "twist":  r"\boldsymbol{\mathcal{T}}",
            "wrench": r"\boldsymbol{\mathcal{W}}",
            "coloneqq": r"\mathrel{:=}",
        }
    }
}

# -- BibTeX references -------------------------------------------------------

bibtex_bibfiles = ['references.bib']
