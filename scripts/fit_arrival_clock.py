#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""How well the flight computer's arrival stamps work as a clock.

Each log the flight computer writes carries two times per row: the time the
sample belongs to (the GNSS epoch's validity time, or the part's own clock)
and `host_time_s`, when the flight computer received it. A filter that handles
delayed measurements needs the second to be a clock: under --rtf 1, the fit

    host_time_s = offset + scale x reference_time

should have scale near 1 and a residual at the level of the communication
step. This script fits that line per stream, by least squares, and reports
offset, scale, and the RMS and largest residual.

    stream  reference column          note
    gps     nominal_time_s            every epoch row, fix or not
    imu     part_time_s               the IMU payload's own clock
    baro    part_time_s               SENSORTIME, unwrapped (it wraps at 512 s)
    mag     imu_part_time_s           the last IMU stamp before the sample

The magnetometer's `host_time_s` is the I2C controller's clock rather than the
loop's, so its offset differs from the others'; its scale and residual mean
the same.

Give a results directory (the one run_integration_case.py or a hand run
writes). With --max-scale-error and/or --max-rms-s it is a check: exit 1 when
any stream that has rows is outside either bound. --json writes the fits.

    python scripts/fit_arrival_clock.py integration-case11 --max-scale-error 0.02 --max-rms-s 0.02
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

# (stream, file, reference column). Read by name, so a column order change in the logs does not move a fit.
STREAMS = [
    ("gps", "gps_fixes.csv", "nominal_time_s"),
    ("imu", "imu_samples.csv", "part_time_s"),
    ("baro", "baro_samples.csv", "part_time_s"),
    ("mag", "mag_samples.csv", "imu_part_time_s"),
]
HOST = "host_time_s"

# A usable clock: every stream within 2 % of scale 1 and under 20 ms rms (two 0.01 s steps). Runners that kept
# --rtf 1 fit at 1.0000 and 1-4 ms; those that fell behind at 1.03-1.36 and 231-471 ms (case 11, CI run
# 43482563098), so the bounds split the two with room. run_integration_case.py and aggregate_replay_summaries.py
# flag against these unless told otherwise.
DEFAULT_MAX_SCALE_ERROR = 0.02
DEFAULT_MAX_RMS_S = 0.02
SENSORTIME_WRAP_S = 512.0  # the BMP390's 24-bit counter at 30.5 us per count, as the log contract documents


def unwrap(times: list[float], period: float) -> list[float]:
    """A counter that wraps every `period` seconds, made monotonic: each fall of more than half a period adds one."""
    out: list[float] = []
    offset = 0.0
    previous = None
    for t in times:
        if previous is not None and t < previous - period / 2.0:
            offset += period
        out.append(t + offset)
        previous = t
    return out


def fit_line(x: list[float], y: list[float]) -> dict[str, float]:
    """Least-squares y = offset + scale x, and its residuals. Needs two distinct x values."""
    n = len(x)
    if n < 2:
        raise ValueError("fewer than two rows")
    mean_x = math.fsum(x) / n
    mean_y = math.fsum(y) / n
    sxx = math.fsum((xi - mean_x) ** 2 for xi in x)
    if sxx == 0.0:
        raise ValueError("every reference time is the same")
    sxy = math.fsum((xi - mean_x) * (yi - mean_y) for xi, yi in zip(x, y))
    scale = sxy / sxx
    offset = mean_y - scale * mean_x
    residuals = [yi - (offset + scale * xi) for xi, yi in zip(x, y)]
    return {
        "rows": n,
        "offset_s": offset,
        "scale": scale,
        "rms_s": math.sqrt(math.fsum(r * r for r in residuals) / n),
        "max_abs_s": max(abs(r) for r in residuals),
        "span_s": max(x) - min(x),
    }


def read_columns(path: Path, reference: str) -> tuple[list[float], list[float]]:
    """(reference, host) for every row where both are finite numbers."""
    xs: list[float] = []
    ys: list[float] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [name.strip() for name in reader.fieldnames or []]
        for name in (reference, HOST):
            if name not in reader.fieldnames:
                raise ValueError(f"{path.name}: no column {name}")
        for row in reader:
            try:
                x = float(row[reference])
                y = float(row[HOST])
            except (TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x)
                ys.append(y)
    return xs, ys


def fit_run(run: Path) -> dict[str, dict]:
    """One fit per stream whose log exists; a stream that cannot be fitted says why instead."""
    fits: dict[str, dict] = {}
    for stream, name, reference in STREAMS:
        path = run / name
        if not path.exists():
            continue
        try:
            x, y = read_columns(path, reference)
            if stream == "baro":
                x = unwrap(x, SENSORTIME_WRAP_S)
            fits[stream] = fit_line(x, y)
        except ValueError as error:
            fits[stream] = {"error": str(error)}
    return fits


def table(fits: dict[str, dict]) -> str:
    lines = [f"{'stream':6} {'rows':>6} {'offset s':>10} {'scale':>9} {'rms ms':>9} {'max ms':>9} {'span s':>8}"]
    for stream, fit in fits.items():
        if "error" in fit:
            lines.append(f"{stream:6} {fit['error']}")
            continue
        lines.append(f"{stream:6} {fit['rows']:6d} {fit['offset_s']:10.4f} {fit['scale']:9.5f} "
                     f"{1e3 * fit['rms_s']:9.2f} {1e3 * fit['max_abs_s']:9.2f} {fit['span_s']:8.2f}")
    return "\n".join(lines)


def violations(fits: dict[str, dict], max_scale_error: float | None, max_rms_s: float | None) -> list[str]:
    problems = []
    for stream, fit in fits.items():
        if "error" in fit:
            problems.append(f"{stream}: {fit['error']}")
            continue
        if max_scale_error is not None and abs(fit["scale"] - 1.0) > max_scale_error:
            problems.append(f"{stream}: scale {fit['scale']:.5f} is more than {max_scale_error:g} from 1")
        if max_rms_s is not None and fit["rms_s"] > max_rms_s:
            problems.append(f"{stream}: residual {1e3 * fit['rms_s']:.2f} ms rms exceeds {1e3 * max_rms_s:g} ms")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="results directory holding the flight computer's logs")
    parser.add_argument("--json", type=Path, help="write the fits here")
    parser.add_argument("--max-scale-error", type=float, help="fail when |scale - 1| exceeds this")
    parser.add_argument("--max-rms-s", type=float, help="fail when the residual RMS exceeds this [s]")
    args = parser.parse_args(argv)

    fits = fit_run(args.run)
    if not fits:
        print(f"no flight-computer logs in {args.run}", file=sys.stderr)
        return 2
    print(table(fits))
    if args.json:
        args.json.write_text(json.dumps(fits, indent=2) + "\n", encoding="utf-8")
    problems = violations(fits, args.max_scale_error, args.max_rms_s)
    for problem in problems:
        print(f"FAIL: {problem}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
