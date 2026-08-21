# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Documentation build: Doxygen parses the module/bsp/sim headers into XML, and Sphinx renders that XML into the API
# reference pages under api/ via Breathe, alongside the hand-written pages here.
#
# Enabled with -DHEMERION_BUILD_DOCS=ON. The same two steps run in .github/workflows/deploy-docs.yml, which is what
# publishes to GitHub Pages; keep the two in step.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

find_package(Doxygen REQUIRED)

# There is no FindSphinx module in cmake/, and Sphinx ships no CMake package config of its own, so locate the console
# script directly. pip installs it onto PATH, which is how doc/requirements.txt gets it here.
find_program(SPHINX_EXECUTABLE NAMES sphinx-build sphinx-build.exe DOC "Path to the sphinx-build executable")
if(NOT SPHINX_EXECUTABLE)
  message(FATAL_ERROR "sphinx-build not found. Install the documentation dependencies: pip install -r "
                      "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tikz)

set(PROJECT_DOC_NAME ${PROJECT_NAME}_Docs)

# ------------------------------------------------------------------------------
# Doxygen: headers -> XML
# ------------------------------------------------------------------------------

# The trees Doxyfile.in names in INPUT. Listed here as well so the custom command reruns when a documented header
# changes -- a glob is resolved at configure time, so a newly added header needs a reconfigure to be picked up.
set(DOXYGEN_INPUT_DIRS
    ${PROJECT_SOURCE_DIR}/modules
    ${PROJECT_SOURCE_DIR}/bsp
    ${PROJECT_SOURCE_DIR}/sim
    ${PROJECT_SOURCE_DIR}/apps)

set(DOXYGEN_INPUT_HEADERS "")
foreach(_dir IN LISTS DOXYGEN_INPUT_DIRS)
  file(
    GLOB_RECURSE
    _headers
    CONFIGURE_DEPENDS
    ${_dir}/*.h
    ${_dir}/*.hpp
    ${_dir}/*.hh)
  list(APPEND DOXYGEN_INPUT_HEADERS ${_headers})
endforeach()
list(
  FILTER
  DOXYGEN_INPUT_HEADERS
  EXCLUDE
  REGEX
  "/(test|tests)/")

set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_XML_DIR ${DOXYGEN_OUTPUT_DIR}/xml)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_XML_DIR}/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Substitutes @CMAKE_SOURCE_DIR@, @DOXYGEN_OUTPUT_DIR@, @CMAKE_PROJECT_NAME@ and @HEMERION_VERSION@.
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us, and WARN_LOGFILE is written into it.
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_command(
  OUTPUT ${DOXYGEN_INDEX_FILE}
  DEPENDS ${DOXYGEN_INPUT_HEADERS}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
  MAIN_DEPENDENCY ${DOXYFILE_OUT}
  COMMENT "Generating ${PROJECT_DOC_NAME} API XML with Doxygen"
  VERBATIM)

add_custom_target(${PROJECT_DOC_NAME}_Doxygen DEPENDS ${DOXYGEN_INDEX_FILE})

# ------------------------------------------------------------------------------
# Sphinx: XML + hand-written pages -> HTML
# ------------------------------------------------------------------------------

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

file(
  GLOB_RECURSE
  SPHINX_SOURCES
  CONFIGURE_DEPENDS
  ${SPHINX_SOURCE}/*.rst
  ${SPHINX_SOURCE}/_ext/*.py)

message(STATUS "Doxygen XML dir:  ${DOXYGEN_XML_DIR}")
message(STATUS "Sphinx build dir: ${SPHINX_BUILD}")

# -D overrides the breathe_projects entry conf.py computes, so the docs build out of this build tree rather than out of
# whatever build/doxygen/xml happens to be lying around. conf.py's own Doxygen fallback then finds the XML already
# present and does not re-run it.
add_custom_command(
  OUTPUT ${SPHINX_INDEX_FILE}
  COMMAND ${SPHINX_EXECUTABLE} -b html -D breathe_projects.${PROJECT_NAME}=${DOXYGEN_XML_DIR} ${SPHINX_SOURCE}
          ${SPHINX_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${SPHINX_SOURCES} ${DOXYGEN_INDEX_FILE}
  MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
  COMMENT "Generating ${PROJECT_DOC_NAME} HTML with Sphinx"
  VERBATIM)

add_custom_target(${PROJECT_DOC_NAME} ALL DEPENDS ${SPHINX_INDEX_FILE} tikz_figures)

include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD}/ DESTINATION ${CMAKE_INSTALL_DOCDIR})
