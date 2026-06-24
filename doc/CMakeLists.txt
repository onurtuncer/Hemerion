# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

set(PROJECT_DOC_NAME ${CMAKE_CURRENT_PROJECT_NAME}_Docs)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tikz)

set(PROJECT_DOC_NAME ${CMAKE_CURRENT_PROJECT_NAME}_Docs)

set(PROJECT_PUBLIC_HEADER_DIR "${PROJECT_SOURCE_DIR}/Aetherion")

file(
  GLOB_RECURSE
  PROJECT_PUBLIC_HEADERS
  ${PROJECT_PUBLIC_HEADER_DIR}/*.h
  ${PROJECT_PUBLIC_HEADER_DIR}/*.hpp)

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/Aetherion)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/Aetherion)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Set the Doxygen input and output directories in the Doxyfile
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

# Only regenerate Doxygen when the Doxyfile or public headers change
add_custom_command(
  OUTPUT ${DOXYGEN_INDEX_FILE}
  DEPENDS ${PROJECT_PUBLIC_HEADERS}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
  MAIN_DEPENDENCY ${DOXYFILE_OUT}
  ${DOXYFILE_IN}
  COMMENT "Generating ${PROJECT_DOC_NAME} docs"
  VERBATIM)

# Nice named target so we can run the job easily
add_custom_target(${PROJECT_DOC_NAME}_Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# Namespace Sphinx variables
set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

message(STATUS "Doxygen output dir: ${DOXYGEN_OUTPUT_DIR}")
message(STATUS "Sphinx build dir: ${SPHINX_BUILD}")

# Only regenerate Sphinx when: - Doxygen has rerun - Our doc files have been updated - The Sphinx config has been
# updated
add_custom_command(
  OUTPUT ${SPHINX_INDEX_FILE}
  COMMAND ${SPHINX_EXECUTABLE} -b html -Dbreathe_projects.${CMAKE_PROJECT_NAME}=${DOXYGEN_OUTPUT_DIR}/xml
          ${SPHINX_SOURCE} ${SPHINX_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/index.rst ${DOXYGEN_INDEX_FILE}
  MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
  COMMENT "Generating ${PROJECT_DOC_NAME} documentation with Sphinx")

# Nice named target so we can run the job easily
add_custom_target(${PROJECT_DOC_NAME}_Sphinx ALL DEPENDS ${SPHINX_INDEX_FILE} tikz_figures)

include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD} DESTINATION ${CMAKE_INSTALL_DOCDIR})
