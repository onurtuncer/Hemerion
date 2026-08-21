# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Check that doc/api/ documents every public header, exactly once.

The API reference pages name their headers by hand, which is what lets them be
grouped and introduced rather than dumped alphabetically -- and which means a
header added, renamed or moved after a page was written silently stops being
documented. Sphinx does not notice: Breathe warns about a directive pointing at
a header that no longer exists, but nothing at all warns about a header that no
directive points at. The reference just quietly gets less complete.

This script closes that gap by comparing two sets:

  * every public header in the tree -- anything under a module's, BSP's or
    simulation library's ``include/``, minus the exclusions Doxygen applies;
  * every header named by a ``.. doxygenfile::`` directive under ``doc/api/``.

and failing if either side has an entry the other does not, or if a header is
named by more than one directive (which produces genuinely duplicated
declarations in the output rather than merely repeated namespace scopes).

Run it directly, or let the docs workflow run it:

    python doc/check_api_coverage.py
"""

from __future__ import annotations

import pathlib
import re
import sys

#: Trees whose ``include/`` directories make up the public API surface.
SOURCE_TREES = ("modules", "bsp", "sim")

#: Header suffixes Doxygen's FILE_PATTERNS accepts.
HEADER_SUFFIXES = (".h", ".hpp", ".hh")

#: Headers that are inside an ``include/`` directory but are not Hemerion API.
#: Keep in step with EXCLUDE_PATTERNS in doc/Doxyfile.in -- a header excluded
#: there has no Doxygen XML, so an api/ page could not document it anyway.
EXCLUDED_NAMES = frozenset(
    {
        "FreeRTOSConfig.h",  # FreeRTOS build configuration, not an interface.
        "stm32h7xx_hal_conf.h",  # ST vendor HAL configuration.
    }
)

DIRECTIVE = re.compile(r"^\s*\.\.\s+doxygenfile::\s*(?P<target>\S+)\s*$", re.MULTILINE)


def repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parent.parent


def public_headers(root: pathlib.Path) -> set[pathlib.Path]:
    """Every header a consumer is meant to be able to include."""
    found: set[pathlib.Path] = set()
    for tree in SOURCE_TREES:
        for path in (root / tree).rglob("*"):
            if path.suffix not in HEADER_SUFFIXES or not path.is_file():
                continue
            if "include" not in path.relative_to(root).parts:
                continue
            if path.name in EXCLUDED_NAMES:
                continue
            found.add(path.relative_to(root))
    return found


def documented_headers(root: pathlib.Path) -> tuple[dict[str, list[str]], list[str]]:
    """Map each doxygenfile target to the pages naming it, plus unresolvable ones."""
    targets: dict[str, list[str]] = {}
    for page in sorted((root / "doc" / "api").glob("*.rst")):
        text = page.read_text(encoding="utf-8")
        for match in DIRECTIVE.finditer(text):
            targets.setdefault(match.group("target"), []).append(page.name)

    resolved: dict[str, list[str]] = {}
    unresolved: list[str] = []
    headers = public_headers(root)
    for target, pages in targets.items():
        suffix = target.replace("\\", "/")
        matches = [h for h in headers if h.as_posix().endswith("/" + suffix)]
        if len(matches) == 1:
            resolved[matches[0].as_posix()] = pages
        else:
            unresolved.append(f"{target} ({'ambiguous' if matches else 'no such header'})")
    return resolved, unresolved


def main() -> int:
    root = repo_root()
    headers = {h.as_posix() for h in public_headers(root)}
    documented, unresolved = documented_headers(root)

    problems: list[str] = []

    if unresolved:
        problems.append(
            "doxygenfile directives that do not name a public header:\n  "
            + "\n  ".join(sorted(unresolved))
        )

    undocumented = sorted(headers - set(documented))
    if undocumented:
        problems.append(
            "public headers with no page in doc/api/ (add a section for each):\n  "
            + "\n  ".join(undocumented)
        )

    repeated = sorted(f"{h} (on {', '.join(p)})" for h, p in documented.items() if len(p) > 1)
    if repeated:
        problems.append(
            "headers documented by more than one directive:\n  " + "\n  ".join(repeated)
        )

    if problems:
        print("API reference coverage check FAILED\n", file=sys.stderr)
        for problem in problems:
            print(problem + "\n", file=sys.stderr)
        return 1

    print(f"API reference coverage OK: {len(headers)} public headers, each documented once.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
