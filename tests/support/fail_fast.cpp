// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fail_fast.cpp
/// @brief Definition of the guard, and the static initializer that installs
/// it before `main()`.

#include "hemerion/test/fail_fast.h"

#ifdef _WIN32
#include <crtdbg.h>
#include <cstdlib>
#include <initializer_list>
#endif

namespace hemerion::test
{

void fail_fast_instead_of_blocking()
{
#ifdef _WIN32
  // _WRITE_ABORT_MSG keeps the message; _CALL_REPORTFAULT is cleared so the
  // process does not hand itself to Windows Error Reporting, which is the
  // part that waits.
  (void)_set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);

  // _CRTDBG_MODE_FILE + stderr, rather than the default _CRTDBG_MODE_WNDW,
  // is what turns the dialog into a line of output.
  for (const int report : { _CRT_WARN, _CRT_ERROR, _CRT_ASSERT })
  {
    (void)_CrtSetReportMode(report, _CRTDBG_MODE_FILE);
    (void)_CrtSetReportFile(report, _CRTDBG_FILE_STDERR);
  }
#endif
}

namespace
{

// Dynamic initialization of a namespace-scope object runs before main(), so
// linking this translation unit is enough -- no test has to call anything.
//
// hemerion_test_support is an OBJECT library precisely so this survives: the
// object file is compiled into each test executable directly. As a static
// library it would be a candidate for the linker to drop, since nothing in
// any test references a symbol from it, and the guard would silently stop
// existing the moment it was needed.
const bool kInstalled = [] {
  fail_fast_instead_of_blocking();
  return true;
}();

}  // namespace

}  // namespace hemerion::test
