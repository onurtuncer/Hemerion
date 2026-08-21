# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# hemerion_add_fmu_archive_test() -- registers the "fmu"-labelled ctest that cmake/verify_fmu_archive.cmake performs,
# one per packaged archive. generateFMU() calls this for every model/FMI-version pair it packages, so an FMU cannot be
# added without its packaging being checked; there is no list here to keep in sync.
#
# Why this lives in its own file rather than inside generate_fmu.cmake: that file is kept under fmu4cpp's MIT licence
# and deliberately close to upstream so it stays cheap to re-sync (see its header). Keeping the Hemerion-specific test
# registration here reduces that file's local delta to a single call.
#
# Why these tests exist at all. The fmu-native preset builds every FMU and builds no tests, and its ctest preset filters
# on label "fmu" -- which, until these landed, no test in the tree carried. Combined with noTestsAction "ignore", that
# made `ctest --preset fmu-native` a guaranteed pass that ran nothing: the same vacuous-green failure mode the SWIL
# suite and the clang-format job each hit once before. The preset now uses noTestsAction "error", which is only safe
# because these tests are registered from the FMU build itself and so cannot go missing while FMUs are still built.
#
# hemerion_add_fmu_archive_test(<modelIdentifier> <fmiVersion> <archivePath>)
#
# <modelIdentifier> is generateFMU()'s per-version target name -- the identifier that ends up inside
# modelDescription.xml and names the shared library -- not the bare model name the archive file is called after.
# ------------------------------------------------------------------------------
function(
  hemerion_add_fmu_archive_test
  modelIdentifier
  fmiVersion
  archivePath)
  set(_verifier "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/verify_fmu_archive.cmake")
  set(_scratch "${CMAKE_BINARY_DIR}/fmu_archive_checks/${fmiVersion}/${modelIdentifier}")
  # modelIdentifier already carries the FMI version (generateFMU names it <model>_<fmiVersion>), so the test name does
  # not repeat it.
  set(_test_name "fmu.${modelIdentifier}.archive")

  add_test(NAME ${_test_name}
           COMMAND "${CMAKE_COMMAND}" "-DFMU_ARCHIVE=${archivePath}" "-DFMU_MODEL_IDENTIFIER=${modelIdentifier}"
                   "-DFMU_SCRATCH_DIR=${_scratch}" -P "${_verifier}")

  # Each test extracts into its own scratch directory, so they are safe to run concurrently.
  set_tests_properties(${_test_name} PROPERTIES LABELS "fmu")
endfunction()
