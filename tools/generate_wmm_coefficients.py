# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Turns NOAA's WMM.COF into the C++ table the field model compiles against.

The coefficients are 90 lines of numbers that no reviewer can check by eye, and
a single mistyped digit produces a field that is entirely plausible and wrong.
So they are never typed: ``vendor/wmm/WMM.COF`` is fetched once, checksummed
and committed, and this script is the only thing that reads it.

The generated header is committed too, so building needs no Python -- which
matters for a table the cross build compiles. That leaves one failure mode, a
generated header that has drifted from the ``.COF`` beside it, and ``--check``
closes it: it regenerates in memory and compares, so CI fails rather than
shipping a table nobody can trace.

    python tools/generate_wmm_coefficients.py            # write the header
    python tools/generate_wmm_coefficients.py --check    # fail if it is stale
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import sys

#: Degree and order of the model. WMM has been 12 since 2000 and the format
#: gives no way to declare it, so it is asserted against the file's contents
#: rather than read from it.
DEGREE = 12

#: Two lines of 48 nines end the coefficient block.
TERMINATOR = "9" * 48


def repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parent.parent


def parse_cof(text: str) -> tuple[dict[str, str], list[tuple[int, int, float, float, float, float]]]:
    """Reads a WMM.COF into its header fields and coefficient rows."""
    lines = [line.rstrip("\n\r") for line in text.splitlines()]
    if not lines:
        raise ValueError("empty coefficient file")

    head = lines[0].split()
    if len(head) != 3:
        raise ValueError(f"header line has {len(head)} fields, expected 3: {lines[0]!r}")
    header = {"epoch": head[0], "model": head[1], "released": head[2]}

    rows: list[tuple[int, int, float, float, float, float]] = []
    for line in lines[1:]:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith(TERMINATOR):
            break
        fields = stripped.split()
        if len(fields) != 6:
            raise ValueError(f"coefficient line has {len(fields)} fields, expected 6: {line!r}")
        n, m = int(fields[0]), int(fields[1])
        rows.append((n, m, float(fields[2]), float(fields[3]), float(fields[4]), float(fields[5])))

    # The file is meant to hold every (n, m) with 1 <= n <= 12 and 0 <= m <= n,
    # exactly once and in order. Checking that here means the C++ side can index
    # the table arithmetically instead of searching it.
    expected = [(n, m) for n in range(1, DEGREE + 1) for m in range(n + 1)]
    found = [(n, m) for n, m, *_ in rows]
    if found != expected:
        missing = set(expected) - set(found)
        extra = set(found) - set(expected)
        raise ValueError(
            f"coefficient rows are not the expected degree-{DEGREE} sequence "
            f"({len(found)} rows; missing {sorted(missing)[:4]}, unexpected {sorted(extra)[:4]})"
        )
    return header, rows


def render(header: dict[str, str], rows: list[tuple[int, int, float, float, float, float]], digest: str) -> str:
    """Emits the C++ header."""
    epoch = float(header["epoch"])
    body = [
        "// ------------------------------------------------------------------------------",
        "// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University",
        "//",
        "// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE",
        "// ------------------------------------------------------------------------------",
        "",
        "/// @file wmm_coefficients.h",
        "/// @brief Spherical-harmonic coefficients of the World Magnetic Model.",
        "///",
        "/// **Generated. Do not edit.** Produced by tools/generate_wmm_coefficients.py",
        "/// from vendor/wmm/WMM.COF; `--check` fails if this file has drifted from it.",
        "///",
        f"/// Model {header['model']}, epoch {header['epoch']}, released {header['released']}.",
        f"/// Source SHA-256 {digest}.",
        "///",
        "/// The coefficients are a work of the U.S. Government and are in the public",
        "/// domain. Cite as: NOAA NCEI Geomagnetic Modeling Team; British Geological",
        "/// Survey. 2024: World Magnetic Model 2025. See vendor/wmm/README.md.",
        "",
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "",
        "namespace hemerion::sensors::mag::wmm",
        "{",
        "",
        "/// Degree and order of the expansion.",
        f"inline constexpr std::size_t kDegree = {DEGREE};",
        "",
        "/// Number of (n, m) pairs with 1 <= n <= kDegree and 0 <= m <= n.",
        f"inline constexpr std::size_t kCoefficientCount = {len(rows)};",
        "",
        "/// Reference epoch of the model [decimal year].",
        f"inline constexpr double kEpochYear = {epoch:.1f};",
        "",
        "/// The model is predictive and is not valid past this date [decimal year].",
        "/// WMM is issued on a five-year cycle; see vendor/wmm/README.md on updating.",
        f"inline constexpr double kValidUntilYear = {epoch + 5.0:.1f};",
        "",
        "/// Model name as the coefficient file declares it.",
        f'inline constexpr const char* kModelName = "{header["model"]}";',
        "",
        "/// One (n, m) term: the Gauss coefficients and their annual rates.",
        "struct Coefficient",
        "{",
        "  int n;        ///< Degree.",
        "  int m;        ///< Order.",
        "  double g;     ///< g(n,m) at the epoch [nT].",
        "  double h;     ///< h(n,m) at the epoch [nT]; zero for m = 0.",
        "  double g_dot;  ///< Annual rate of change of g [nT/year].",
        "  double h_dot;  ///< Annual rate of change of h [nT/year].",
        "};",
        "",
        "/// Every term, ordered by degree then order, exactly as WMM.COF lists them.",
        f"inline constexpr std::array<Coefficient, kCoefficientCount> kCoefficients = {{ {{",
    ]
    for n, m, g, h, gd, hd in rows:
        body.append(f"    {{ {n:2d}, {m:2d}, {g:>10.1f}, {h:>10.1f}, {gd:>7.1f}, {hd:>7.1f} }},")
    body += [
        "} };",
        "",
        "}  // namespace hemerion::sensors::mag::wmm",
        "",
    ]
    return "\n".join(body)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--check",
        action="store_true",
        help="do not write; exit non-zero if the committed header differs from what this would generate",
    )
    args = parser.parse_args()

    root = repo_root()
    source = root / "vendor" / "wmm" / "WMM.COF"
    target = root / "modules" / "sensors" / "include" / "Hemerion" / "mag" / "wmm_coefficients.h"

    if not source.is_file():
        print(f"error: {source} not found", file=sys.stderr)
        return 1

    raw = source.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    try:
        header, rows = parse_cof(raw.decode("ascii"))
    except (ValueError, UnicodeDecodeError) as error:
        print(f"error: {source} is not a WMM coefficient file: {error}", file=sys.stderr)
        return 1

    generated = render(header, rows, digest)

    if args.check:
        if not target.is_file():
            print(f"error: {target} does not exist; run this script without --check", file=sys.stderr)
            return 1
        current = target.read_text(encoding="utf-8")
        if current != generated:
            print(
                "error: wmm_coefficients.h is out of step with vendor/wmm/WMM.COF.\n"
                "       Run: python tools/generate_wmm_coefficients.py",
                file=sys.stderr,
            )
            return 1
        print(f"wmm_coefficients.h matches {source.name} ({header['model']}, {len(rows)} terms)")
        return 0

    target.write_text(generated, encoding="utf-8", newline="\n")
    print(f"wrote {target.relative_to(root)} from {header['model']} ({len(rows)} terms, sha256 {digest[:12]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
