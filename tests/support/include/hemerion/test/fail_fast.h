// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fail_fast.h
/// @brief Makes a failing `assert()` abort immediately rather than block a
/// headless runner.
///
/// These tests assert and exit; there is no framework underneath them. On
/// Windows that is a hazard rather than a simplification: the debug CRT
/// answers `abort()` and a failed `assert()` with a modal dialog box, and a
/// CI runner has nobody to dismiss it. A one-line assertion failure then
/// reads as a hung job -- it cost a 180 s ctest stall here once, with no
/// output naming the test, the file, or the assertion.
///
/// Linking `hemerion_test_support` installs the fix before `main()` runs, so
/// a test author cannot forget it. That is deliberate: this guard was written
/// once, in test_mmc5983ma.cpp, and the twenty-nine sibling executables
/// carried the same exposure and no guard for as long as they existed.
/// Anything that must be remembered in thirty places is better spent as a
/// link-time dependency.
///
/// The function is exposed as well so a test can state the dependency at its
/// call site, and so the mechanism is greppable from the tests that rely on
/// it. Calling it is harmless and redundant.
///
/// No effect on any platform but Windows.

#pragma once

namespace hemerion::test
{

/// @brief Routes CRT diagnostics to stderr and makes `abort()` return an exit
/// code immediately, instead of raising a modal dialog.
///
/// Idempotent. Runs automatically before `main()` for anything that links
/// `hemerion_test_support`.
void fail_fast_instead_of_blocking();

}  // namespace hemerion::test
