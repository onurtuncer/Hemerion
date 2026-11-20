#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Whether two runs of the same seed logged the same sensor values.

A change that should only move timing (when samples arrive, how pacing is
reported) must leave every value a seeded run logs exactly as it was. This
compares two results directories -- e.g. a step-7 artifact from main and one
from a branch -- cell for cell, as text, so a difference in the last printed
digit counts:

    file               rows matched by   compared
    gps_fixes.csv      fix_index         every column but host_time_s
    imu_samples.csv    part_time_s       every column but host_time_s
    baro_samples.csv   part_time_s       every column but host_time_s
    mag_samples.csv    sample_index      every column but host_time_s and
                                          imu_part_time_s (both are when the
                                          sample was seen, not what it was)
    f16_truth.csv      row               every column (also rocket_truth.csv)
    the sidecar        key               every key of the first run but the
                                          host's pacing_* report; keys only the
                                          second has are listed, not failed

IMU and BMP390 rows are matched by the part's own clock, so their running
sample_index is not compared; the row counts are.

Row counts must match too, per file. Exit status 0 when nothing differs, 1 when
something does (each difference named), 2 when a run has none of the files.
On Windows the polled BMP390's log is not reproducible run to run; give
--skip baro (and mag) there.

    python scripts/compare_run_values.py main-run/case11-seed1 branch-run/case11-seed1
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

# Columns that record when the flight computer saw a sample rather than what the sample was: the arrival stamp, and
# the magnetometer's alignment to the last IMU stamp before it (an unstamped part's only time base).
STAMPS = {"host_time_s", "imu_part_time_s"}
LOGS = {
    "gps": ("gps_fixes.csv", "fix_index"),
    "imu": ("imu_samples.csv", "part_time_s"),
    "baro": ("baro_samples.csv", "part_time_s"),
    "mag": ("mag_samples.csv", "sample_index"),
}
TRUTHS = ("f16_truth.csv", "rocket_truth.csv")
PACING_PREFIX = "pacing_"  # the host's pacing report (examples/common/pacing.hpp): measured, so never compared


def keyed_rows(path: Path, key: str | None) -> tuple[list[str], dict[str, list[str]]]:
    """Header, and each row's compared cells keyed by `key` (or by position when None)."""
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = [name.strip() for name in next(reader, [])]
        # A log keyed by the part's own clock carries its running count as sample_index: which row a sample landed
        # in, not what it was. The row-count check below still catches a sample gained or lost.
        skipped = STAMPS | ({"sample_index"} if key == "part_time_s" else set())
        keep = [i for i, name in enumerate(header) if name not in skipped]
        index = header.index(key) if key else None
        rows: dict[str, list[str]] = {}
        for position, row in enumerate(reader):
            row_key = row[index].strip() if index is not None else str(position)
            rows[row_key] = [row[i].strip() if i < len(row) else "" for i in keep]
    return [header[i] for i in keep], rows


def compare_table(a: Path, b: Path, key: str | None, problems: list[str]) -> None:
    header_a, rows_a = keyed_rows(a, key)
    header_b, rows_b = keyed_rows(b, key)
    if header_a != header_b:
        problems.append(f"{a.name}: columns differ")
        return
    if len(rows_a) != len(rows_b):
        problems.append(f"{a.name}: {len(rows_a)} rows against {len(rows_b)}")
    for row_key in sorted(set(rows_a) & set(rows_b)):
        if rows_a[row_key] != rows_b[row_key]:
            cells = [f"{name} {x!r} vs {y!r}" for name, x, y in zip(header_a, rows_a[row_key], rows_b[row_key])
                     if x != y]
            problems.append(f"{a.name} {key or 'row'} {row_key}: " + "; ".join(cells[:3]))
            return  # the first differing row says enough; the rest usually follow from it
    if set(rows_a) != set(rows_b):
        problems.append(f"{a.name}: {len(set(rows_a) ^ set(rows_b))} rows present in only one run")


def read_sidecar(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            values[key.strip()] = value.strip()
    return values


def compare_runs(a: Path, b: Path, skip: set[str]) -> tuple[list[str], list[str], int]:
    """(differences, notes, files compared)."""
    problems: list[str] = []
    notes: list[str] = []
    compared = 0
    for stream, (name, key) in LOGS.items():
        if stream in skip or not (a / name).exists():
            continue
        if not (b / name).exists():
            problems.append(f"{name}: missing from {b}")
            continue
        compare_table(a / name, b / name, key, problems)
        compared += 1
    for name in TRUTHS:
        if (a / name).exists():
            if not (b / name).exists():
                problems.append(f"{name}: missing from {b}")
                continue
            compare_table(a / name, b / name, None, problems)
            compared += 1
            sidecar = Path(name).with_suffix(".config")
            if (a / sidecar).exists() and (b / sidecar).exists():
                keys_a, keys_b = read_sidecar(a / sidecar), read_sidecar(b / sidecar)
                for key, value in keys_a.items():
                    if key.startswith(PACING_PREFIX):
                        continue  # how pacing went is a measurement of the host, not a configuration of the run
                    if keys_b.get(key) != value:
                        problems.append(f"{sidecar}: {key} = {value!r} vs {keys_b.get(key)!r}")
                added = sorted(set(keys_b) - set(keys_a))
                if added:
                    notes.append(f"{sidecar}: keys only in the second run: {', '.join(added)}")
                compared += 1
    return problems, notes, compared


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("first", type=Path, help="results directory of the reference run")
    parser.add_argument("second", type=Path, help="results directory of the run to check")
    parser.add_argument("--skip", action="append", default=[], choices=sorted(LOGS),
                        help="leave a sensor log out (repeatable); e.g. baro on Windows")
    args = parser.parse_args(argv)
    problems, notes, compared = compare_runs(args.first, args.second, set(args.skip))
    if compared == 0:
        print(f"no logs to compare in {args.first}", file=sys.stderr)
        return 2
    for note in notes:
        print(f"note: {note}")
    for problem in problems:
        print(f"DIFFERS: {problem}")
    if not problems:
        print(f"same values: {compared} files compared")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
