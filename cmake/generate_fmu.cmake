# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# generateFMU() -- compiles plain C++ model sources against the vendored fmu4cpp export layer (vendor/fmu4cpp/export)
# and packages the result as a standards-conforming .fmu archive.
#
# Adapted from fmu4cpp's own CMake helper (github.com/markaren/fmu4cpp) and deliberately kept under fmu4cpp's MIT
# licence rather than Hemerion's GPL-3.0-only, so it stays cheap to re-sync against upstream. Hemerion-local changes:
#
# * the fmu4cpp checkout is located at vendor/fmu4cpp only -- upstream probes its own source tree;
# * `cmake -E tar` is handed archive-relative member names, so the packaged .fmu really does have modelDescription.xml
#   at its root and binaries/ next to it, instead of a copy of the build machine's absolute path prefix;
# * the FMI2/FMI3 macros on the shared fmu4cpp object libraries are defined once in vendor/CMakeLists.txt, where those
#   targets are created; this function only defines them on the model target it builds itself;
# * VERSION_DEFS is cleared per FMI version -- upstream clears a misspelled VERSIONS_DEFS, so a model built for both
#   fmi2 and fmi3 compiled the second one with both macros defined;
# * static libraries are not "bundled" next to the model binary -- a static library is linked into that binary and has
#   no runtime artifact, so copying it only buried a .a/.lib inside the packaged archive.
#
# Usage:
#
# cmake-format: off
# generateFMU(<modelIdentifier>
#             FMI_VERSIONS         fmi2 [fmi3]   # at least one; required
#             SOURCES              <src>...      # model sources; required
#             [INCLUDE_DIRS        <dir>...]
#             [LINK_TARGETS        <target>...]
#             [COMPILE_DEFINITIONS <def>...]
#             [RESOURCE_FOLDER     <dir>]        # copied to resources/ inside the archive
#             [DOC_FOLDER          <dir>]        # copied to documentation/ inside the archive
#             [DESTINATION         <dir>]        # archive lands in <dir>/<fmiVersion>/
#             [WITH_SOURCES])                    # fmi3 only: embed sources/ + buildDescription.xml
# cmake-format: on
#
# For every requested FMI version this creates one shared-library target named <modelIdentifier>_<fmiVersion> -- which
# is also the modelIdentifier written into modelDescription.xml, since one binary cannot export both FMI ABIs --
# generates that library's modelDescription.xml by loading it with the descriptionGenerator host tool, and zips the
# staged layout under ${CMAKE_BINARY_DIR}/models/<fmiVersion>/<modelIdentifier>/ into <modelIdentifier>.fmu. All of it
# runs as POST_BUILD steps, so a plain `cmake --build` leaves loadable archives behind.
# ------------------------------------------------------------------------------

get_filename_component(_fmu4cpp_root "${CMAKE_CURRENT_LIST_DIR}/../vendor/fmu4cpp" ABSOLUTE)

if(NOT EXISTS "${_fmu4cpp_root}/export/src/fmu4cpp/model_identifier.cpp.in")
  message(FATAL_ERROR "Cannot locate the fmu4cpp export templates under ${_fmu4cpp_root}/export/.\n"
                      "vendor/fmu4cpp must contain fmu4cpp's export/ subtree -- see vendor/CMakeLists.txt.")
endif()

set(_fmu4cpp_root "${_fmu4cpp_root}" CACHE INTERNAL "Root of the vendored fmu4cpp checkout")

# ------------------------------------------------------------------------------
# generateFMU
# ------------------------------------------------------------------------------
function(generateFMU modelIdentifier)

  set(options WITH_SOURCES)
  set(oneValueArgs RESOURCE_FOLDER DOC_FOLDER DESTINATION)
  set(multiValueArgs
      FMI_VERSIONS
      SOURCES
      INCLUDE_DIRS
      LINK_TARGETS
      COMPILE_DEFINITIONS)
  cmake_parse_arguments(
    FMU
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(NOT FMU_FMI_VERSIONS)
    message(FATAL_ERROR "generateFMU(${modelIdentifier}) requires at least one FMI version in FMI_VERSIONS")
  endif()
  if(NOT FMU_SOURCES)
    message(FATAL_ERROR "generateFMU(${modelIdentifier}) requires SOURCES to be provided")
  endif()
  if(NOT TARGET fmu4cpp_base OR NOT TARGET descriptionGenerator)
    message(FATAL_ERROR "generateFMU(${modelIdentifier}) requires the fmu4cpp targets -- "
                        "vendor/CMakeLists.txt only adds them when HEMERION_BUILD_FMU is ON")
  endif()

  set(fmuResultDir "${CMAKE_BINARY_DIR}/models")
  set(generatedSourcesDir "${CMAKE_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${fmuResultDir}" "${generatedSourcesDir}")

  foreach(fmiVersion IN LISTS FMU_FMI_VERSIONS)

    _gettargetplatform(${fmiVersion})

    set(modelOutputDir "${fmuResultDir}/${fmiVersion}/${modelIdentifier}")
    set(binaryOutputDir "$<1:${modelOutputDir}/binaries/${TARGET_PLATFORM}>")

    # One binary cannot export both the FMI 2.0 and the FMI 3.0 ABI, so the per-version target name doubles as the
    # modelIdentifier: fmu4cpp's model_identifier() is generated from it, and modelDescription.xml has to name the
    # shared library sitting in binaries/<platform>/.
    set(versionTarget "${modelIdentifier}_${fmiVersion}")
    set(FMU4CPP_MODEL_IDENTIFIER "${versionTarget}")

    set(VERSION_OBJECTS "${generatedSourcesDir}/fmu4cpp/model_identifier_${versionTarget}.cpp")
    configure_file("${_fmu4cpp_root}/export/src/fmu4cpp/model_identifier.cpp.in" "${VERSION_OBJECTS}" @ONLY)

    set(VERSION_DEFS "")
    if(fmiVersion STREQUAL "fmi2")
      list(APPEND VERSION_DEFS "FMI2")
      list(APPEND VERSION_OBJECTS "$<TARGET_OBJECTS:fmu4cpp_fmi2>")
    elseif(fmiVersion STREQUAL "fmi3")
      list(APPEND VERSION_DEFS "FMI3")
      list(APPEND VERSION_OBJECTS "$<TARGET_OBJECTS:fmu4cpp_fmi3>")
    else()
      message(FATAL_ERROR "Unknown FMI version: ${fmiVersion}. Supported versions are 'fmi2' and 'fmi3'.")
    endif()

    add_library(${versionTarget} SHARED "$<TARGET_OBJECTS:fmu4cpp_base>" "${FMU_SOURCES}" "${VERSION_OBJECTS}")
    target_include_directories(${versionTarget} PRIVATE "${_fmu4cpp_root}/export/include" "${FMU_INCLUDE_DIRS}")
    target_compile_definitions(${versionTarget} PRIVATE ${VERSION_DEFS} ${FMU_COMPILE_DEFINITIONS})

    if(FMU_LINK_TARGETS)
      target_link_libraries(${versionTarget} PRIVATE ${FMU_LINK_TARGETS})
      _bundle_link_libraries()
    endif()

    if(FMU_WITH_SOURCES)
      if(fmiVersion STREQUAL "fmi2")
        message(WARNING "[generateFMU-fmi2] WITH_SOURCES is an FMI 3.0 feature; ignoring it for '${modelIdentifier}'")
      else()
        _include_sources_in_fmu()
      endif()
    endif()

    if(WIN32)
      set_target_properties(${versionTarget} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${binaryOutputDir}")
    else()
      set_target_properties(${versionTarget} PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${binaryOutputDir}")
    endif()

    # descriptionGenerator loads the freshly built library and calls its write_description() export, which serialises
    # the variables the model registered in its constructor. It writes ../../modelDescription.xml relative to its
    # working directory, i.e. straight into modelOutputDir.
    add_custom_command(
      TARGET ${versionTarget}
      POST_BUILD
      WORKING_DIRECTORY "${binaryOutputDir}"
      COMMAND ${CMAKE_COMMAND} -E echo
              "[generateFMU-${fmiVersion}] Generating modelDescription.xml for model '${modelIdentifier}'"
      COMMAND descriptionGenerator "$<TARGET_FILE_NAME:${versionTarget}>")

    _package_fmu()

  endforeach()

endfunction()

# ------------------------------------------------------------------------------
# Zips the staged model directory into <modelIdentifier>.fmu. Every member name is relative to modelOutputDir (the
# custom command's working directory): an .fmu is a zip whose root holds modelDescription.xml, so absolute member names
# would bury the whole layout under the build machine's own path.
# ------------------------------------------------------------------------------
macro(_package_fmu)
  set(TAR_INPUTS "modelDescription.xml" "binaries")
  if(FMU_WITH_SOURCES AND fmiVersion STREQUAL "fmi3")
    list(APPEND TAR_INPUTS "sources")
  endif()

  if(FMU_DOC_FOLDER)
    message("[generateFMU-${fmiVersion}] Using documentation folder=${FMU_DOC_FOLDER} for model '${modelIdentifier}'")
    file(COPY "${FMU_DOC_FOLDER}/" DESTINATION "${modelOutputDir}/documentation")
    list(APPEND TAR_INPUTS "documentation")
  endif()

  if(FMU_RESOURCE_FOLDER)
    message("[generateFMU-${fmiVersion}] Using resource folder=${FMU_RESOURCE_FOLDER} for model '${modelIdentifier}'")
    file(COPY "${FMU_RESOURCE_FOLDER}/" DESTINATION "${modelOutputDir}/resources")
    list(APPEND TAR_INPUTS "resources")
  endif()

  if(FMU_DESTINATION)
    file(MAKE_DIRECTORY "${FMU_DESTINATION}/${fmiVersion}")
    set(FMU_ARCHIVE "${FMU_DESTINATION}/${fmiVersion}/${modelIdentifier}.fmu")
  else()
    set(FMU_ARCHIVE "${modelIdentifier}.fmu")
  endif()

  add_custom_command(
    TARGET ${versionTarget}
    POST_BUILD
    WORKING_DIRECTORY "${modelOutputDir}"
    COMMAND ${CMAKE_COMMAND} -E echo "[generateFMU-${fmiVersion}] Packaging ${modelIdentifier}.fmu in ${FMU_ARCHIVE}"
    COMMAND ${CMAKE_COMMAND} -E tar c "${FMU_ARCHIVE}" --format=zip -- ${TAR_INPUTS})
endmacro()

# ------------------------------------------------------------------------------
# Copies each linked dependency's runtime artifact next to the model binary, so the FMU stays loadable once unzipped
# somewhere else. INTERFACE libraries, STATIC libraries (linked into the model binary itself) and bare system link flags
# (e.g. ws2_32) have no runtime file and are skipped.
# ------------------------------------------------------------------------------
macro(_bundle_link_libraries)
  foreach(dep IN LISTS FMU_LINK_TARGETS)
    if(TARGET ${dep})
      get_target_property(target_type ${dep} TYPE)
      if(NOT
         "${target_type}"
         MATCHES
         "^(INTERFACE|STATIC)_LIBRARY$")
        add_custom_command(
          TARGET ${versionTarget}
          POST_BUILD
          WORKING_DIRECTORY "${modelOutputDir}"
          COMMAND ${CMAKE_COMMAND} -E echo "[generateFMU-${fmiVersion}] Copying runtime of ${dep} to ${binaryOutputDir}"
          COMMAND ${CMAKE_COMMAND} -E make_directory "${binaryOutputDir}"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:${dep}>
                  "${binaryOutputDir}/$<TARGET_FILE_NAME:${dep}>")
      endif()
    endif()
  endforeach()
endmacro()

# ------------------------------------------------------------------------------
# FMI 3.0 source-code FMU: stages every compilable source plus a buildDescription.xml, so an importer can rebuild the
# binary itself. FMI 2.0 has no equivalent, hence the caller-side guard.
# ------------------------------------------------------------------------------
macro(_include_sources_in_fmu)

  message("[generateFMU-${fmiVersion}] Including sources in FMU for model '${modelIdentifier}'")

  set(VERSION_SOURCES ${VERSION_OBJECTS})
  file(MAKE_DIRECTORY "${modelOutputDir}/sources")
  file(COPY "${generatedSourcesDir}/fmu4cpp/lib_info.cpp" DESTINATION "${modelOutputDir}/sources/fmu4cpp/")
  file(COPY ${_fmu4cpp_root}/export/src/fmu4cpp DESTINATION "${modelOutputDir}/sources/")
  file(REMOVE_RECURSE "${modelOutputDir}/sources/fmu4cpp/fmi2") # remove fmi2 sources
  file(COPY ${_fmu4cpp_root}/export/include/fmu4cpp DESTINATION "${modelOutputDir}/sources/")

  foreach(dir IN LISTS FMU_INCLUDE_DIRS)
    file(COPY "${dir}/" DESTINATION "${modelOutputDir}/sources/")
  endforeach()

  foreach(s IN LISTS FMU_SOURCES VERSION_SOURCES)
    if(NOT
       "${s}"
       MATCHES
       "^\\$<") # skip generator expressions
      if(IS_ABSOLUTE "${s}")
        set(abs "${s}")
      else()
        get_filename_component(abs "${CMAKE_CURRENT_SOURCE_DIR}/${s}" ABSOLUTE)
      endif()
      file(COPY "${abs}" DESTINATION "${modelOutputDir}/sources/")
    endif()
  endforeach()

  set(SOURCE_SET "")
  file(GLOB_RECURSE COPIED_CPP_FILES "${modelOutputDir}/sources/*.cpp")
  foreach(cpp_file IN LISTS COPIED_CPP_FILES)
    file(
      RELATIVE_PATH
      rel_path
      "${modelOutputDir}/sources"
      "${cpp_file}")
    set(SOURCE_SET "${SOURCE_SET}\t\t\t<SourceFile name=\"${rel_path}\"/>\n")
  endforeach()

  file(
    WRITE "${modelOutputDir}/sources/buildDescription.xml"
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<fmiBuildDescription fmiVersion=\"3.0\">\n"
    "\t<BuildConfiguration modelIdentifier=\"${FMU4CPP_MODEL_IDENTIFIER}\">\n"
    "\t\t<SourceFileSet language=\"C++17\">\n"
    ${SOURCE_SET}
    "\t\t\t<PreprocessorDefinition name=\"FMI3\"/>\n"
    "\t\t\t<IncludeDirectory name=\"include/\"/>\n"
    "\t\t</SourceFileSet>\n"
    "\t</BuildConfiguration>\n"
    "</fmiBuildDescription>\n")
endmacro()

# ------------------------------------------------------------------------------
# binaries/<platform>/ naming. FMI 2.0 uses win64/linux64/darwin64; FMI 3.0 switched to the <arch>-<os> spelling, e.g.
# x86_64-windows.
# ------------------------------------------------------------------------------
function(_getTargetPlatform fmiVersion)
  set(_target "")
  if(fmiVersion STREQUAL "fmi2")

    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
      set(BITNESS 64)
    else()
      set(BITNESS 32)
    endif()

    if(WIN32)
      set(_target win${BITNESS})
    elseif(APPLE)
      set(_target darwin${BITNESS})
    else()
      set(_target linux${BITNESS})
    endif()

  elseif(fmiVersion STREQUAL "fmi3")

    set(_target "x86")
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
      set(_target "${_target}_64")
    endif()

    if(WIN32)
      set(_target ${_target}-windows)
    elseif(APPLE)
      set(_target ${_target}-darwin)
    else()
      set(_target ${_target}-linux)
    endif()

  else()
    message(FATAL_ERROR "Unknown FMI version: ${fmiVersion}. Supported versions are 'fmi2' and 'fmi3'.")
  endif()

  set(TARGET_PLATFORM ${_target} PARENT_SCOPE)

endfunction()
