cmake_minimum_required(VERSION 3.16)

# Find pdflatex
find_program(PDFLATEX_EXECUTABLE pdflatex)

if(NOT PDFLATEX_EXECUTABLE)
  message(
    WARNING "pdflatex not found; skipping TikZ figure generation. Pre-built figures in doc/sphinx/_static will be used."
  )
  add_custom_target(tikz_figures)
  return()
endif()

# Directories
set(TIKZ_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(TIKZ_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Parent of this binary dir is build/doc
get_filename_component(DOC_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/.." ABSOLUTE)

# Destination: build/doc/sphinx/_static/figures
set(TIKZ_DEST_DIR "${DOC_BINARY_DIR}/sphinx/_static/")

file(MAKE_DIRECTORY "${TIKZ_BUILD_DIR}")
file(MAKE_DIRECTORY "${TIKZ_DEST_DIR}")

# Collect all .tex files in this directory
file(GLOB TIKZ_TEX_FILES "${TIKZ_SRC_DIR}/*.tex")

set(TIKZ_COPIED_PDFS "")

foreach(tex ${TIKZ_TEX_FILES})
  get_filename_component(fig_name "${tex}" NAME_WE)

  set(pdf_build "${TIKZ_BUILD_DIR}/${fig_name}.pdf")
  set(pdf_dest "${TIKZ_DEST_DIR}/${fig_name}.pdf")

  # 1) Build PDF for this figure
  add_custom_command(
    OUTPUT "${pdf_build}"
    COMMAND "${PDFLATEX_EXECUTABLE}" -interaction=nonstopmode -halt-on-error -output-directory "${TIKZ_BUILD_DIR}"
            "${tex}"
    WORKING_DIRECTORY "${TIKZ_BUILD_DIR}"
    DEPENDS "${tex}"
    COMMENT "Building TikZ figure ${fig_name}.pdf with pdflatex"
    VERBATIM)

  # 2) Copy PDF into build/doc/sphinx/_static/figures
  add_custom_command(
    OUTPUT "${pdf_dest}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${pdf_build}" "${pdf_dest}"
    DEPENDS "${pdf_build}"
    COMMENT "Copying ${fig_name}.pdf to Sphinx static dir (binary tree)"
    VERBATIM)

  list(APPEND TIKZ_COPIED_PDFS "${pdf_dest}")
endforeach()

# Aggregate target: build + copy all figures
add_custom_target(tikz_figures ALL DEPENDS ${TIKZ_COPIED_PDFS})
