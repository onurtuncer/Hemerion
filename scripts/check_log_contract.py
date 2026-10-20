#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Checks the co-simulation logs against Aetherion's replay contract.

The examples' logs are an interface: Aetherion's replay
(Aetherion/Estimation/LogContract.h) binds their columns and the truth
sidecar's keys by name. Hemerion writes every such name from
examples/common/log_contract.hpp, and tests/unit/test_log_contract.cpp pins
that header against a copy of Aetherion's names. This script closes the two
gaps that test cannot:

  --aetherion-header LogContract.h
        Compare log_contract.hpp with the real header, and the test's pinned
        copy with it too, so the copy cannot go stale unnoticed.
  --results DIR [--truth FILE]
        Check the logs a finished run actually wrote: the three sensor logs,
        the truth CSV and its .config sidecar, including the drawn sensor-error
        columns (sensor fusion step 3).

Either or both. Exit status 0 when every check passes, 1 otherwise, with each
failure named.

    python scripts/check_log_contract.py --aetherion-header <prefix>/include/Aetherion/Estimation/LogContract.h
    python scripts/check_log_contract.py --results results --truth results/f16_truth.csv
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
HEMERION_HEADER = REPO / "examples" / "common" / "log_contract.hpp"
PINNED_TEST = REPO / "tests" / "unit" / "test_log_contract.cpp"

# Used for --results when no Aetherion header is given: Aetherion v0.18.0's
# names, the same copy the unit test pins.
DEFAULT_REQUIRED = {
    "Gps": ["host_time_s", "nominal_time_s", "latitude_deg", "longitude_deg", "altitude_m", "horizontal_accuracy_m",
            "vertical_accuracy_m", "fix_type", "vel_north_mps", "vel_east_mps", "vel_down_mps", "speed_accuracy_mps"],
    "Imu": ["part_time_s", "host_time_s", "accel_x_mps2", "accel_y_mps2", "accel_z_mps2", "gyro_x_rad_s",
            "gyro_y_rad_s", "gyro_z_rad_s"],
    "Baro": ["part_time_s", "host_time_s", "pressure_pa", "temperature_c"],
    "Truth": ["time", "out.lat_deg", "out.lon_deg", "out.alt_m", "out.v_north_m_s", "out.v_east_m_s",
              "out.v_down_m_s", "out.yaw_rad", "out.pitch_rad", "out.roll_rad"],
    "Sidecar": ["lat0_deg", "lon0_deg", "alt0_ft", "gps_latency_s", "realtime_factor"],
}

# Step 3's drawn-error truth columns, after normalisation (instance and [TYPE]
# stripped) -- examples/common/environment.hpp, drawn_error_variables().
DRAWN_ERRORS = ([f"accel_bias_{a}_mps2" for a in "xyz"] + [f"gyro_bias_{a}_rad_s" for a in "xyz"]
                + [f"accel_scale_error_{a}" for a in "xyz"] + [f"gyro_scale_error_{a}" for a in "xyz"]
                + [f"accel_misalignment_{a}_rad" for a in "xyz"] + [f"gyro_misalignment_{a}_rad" for a in "xyz"]
                + ["pressure_bias_pa", "temperature_bias_c"])

LOGS = {"Gps": "gps_fixes.csv", "Imu": "imu_samples.csv", "Baro": "baro_samples.csv"}


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def parse_aetherion(path: Path) -> dict[str, list[str]]:
    """{namespace: [names]} for the column/key namespaces of LogContract.h."""
    text = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    out: dict[str, list[str]] = {}
    for space in ("Gps", "Imu", "Baro", "Truth", "Sidecar"):
        match = re.search(r"namespace\s+" + space + r"\s*\{(.*?)\}", text, flags=re.S)
        if not match:
            raise SystemExit(f"{path}: no namespace {space} -- has the contract's layout changed?")
        out[space] = re.findall(r"std::string_view\s+k\w+\s*=\s*\"([^\"]*)\"", match.group(1))
    return out


def parse_hemerion(path: Path) -> dict[str, list[str]]:
    """{kGpsHeader: [columns], ..., "keys": [sidecar keys]} from log_contract.hpp."""
    text = strip_comments(path.read_text(encoding="utf-8"))
    out: dict[str, list[str]] = {}
    for name in ("kGpsHeader", "kImuHeader", "kBaroHeader"):
        match = re.search(name + r"\s*=\s*((?:\"[^\"]*\"\s*)+);", text)
        if not match:
            raise SystemExit(f"{path}: cannot find {name}")
        out[name] = "".join(re.findall(r"\"([^\"]*)\"", match.group(1))).split(",")
    out["keys"] = re.findall(r"std::string_view\s+k\w+Key\s*=\s*\"([^\"]*)\"", text)
    return out


def parse_pinned(path: Path) -> dict[str, list[str]]:
    """The copy of Aetherion's names that tests/unit/test_log_contract.cpp pins."""
    text = strip_comments(path.read_text(encoding="utf-8"))
    out: dict[str, list[str]] = {}
    for space in ("Gps", "Imu", "Baro", "Sidecar"):
        match = re.search(r"kAetherion" + space + r"\s*\{(.*?)\};", text, flags=re.S)
        if not match:
            raise SystemExit(f"{path}: cannot find kAetherion{space}")
        out[space] = re.findall(r"\"([^\"]*)\"", match.group(1))
    return out


def normalise(column: str) -> str:
    """"f16::out.lat_deg[REAL]" -> "out.lat_deg", as Aetherion's bindColumns does."""
    column = column.strip()
    if column.endswith("]") and "[" in column:
        column = column[: column.rfind("[")]
    if "::" in column:
        column = column[column.rfind("::") + 2:]
    return column


def check_upstream(aetherion: dict[str, list[str]], failures: list[str]) -> None:
    hemerion = parse_hemerion(HEMERION_HEADER)
    for space, header in (("Gps", "kGpsHeader"), ("Imu", "kImuHeader"), ("Baro", "kBaroHeader")):
        for name in aetherion[space]:
            if name not in hemerion[header]:
                failures.append(f"log_contract.hpp {header} lacks '{name}' (Aetherion LogContract::{space})")
    for key in aetherion["Sidecar"]:
        if key not in hemerion["keys"]:
            failures.append(f"log_contract.hpp has no sidecar key '{key}' (Aetherion LogContract::Sidecar)")

    pinned = parse_pinned(PINNED_TEST)
    for space, names in pinned.items():
        if sorted(names) != sorted(aetherion[space]):
            missing = sorted(set(aetherion[space]) - set(names))
            extra = sorted(set(names) - set(aetherion[space]))
            failures.append(f"test_log_contract.cpp's pinned kAetherion{space} is stale: "
                            f"missing {missing}, not in Aetherion {extra}")


def check_results(results: Path, truth: Path, required: dict[str, list[str]], failures: list[str]) -> None:
    for space, filename in LOGS.items():
        path = results / filename
        if not path.exists():
            failures.append(f"{path}: missing")
            continue
        with path.open(newline="") as f:
            header = [normalise(c) for c in next(csv.reader(f), [])]
        for name in required[space]:
            if name not in header:
                failures.append(f"{path}: no column '{name}'")

    if not truth.exists():
        failures.append(f"{truth}: missing")
        return
    with truth.open(newline="") as f:
        header = [normalise(c) for c in next(csv.reader(f), [])]
    for name in required["Truth"] + DRAWN_ERRORS:
        if name not in header:
            failures.append(f"{truth}: no column '{name}'")

    sidecar = truth.with_suffix(".config")
    if not sidecar.exists():
        failures.append(f"{sidecar}: missing")
        return
    keys = {line.split("=", 1)[0].strip() for line in sidecar.read_text().splitlines() if "=" in line}
    for key in required["Sidecar"]:
        if key not in keys:
            failures.append(f"{sidecar}: no key '{key}'")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aetherion-header", type=Path, help="Aetherion/Estimation/LogContract.h to check against")
    parser.add_argument("--results", type=Path, help="results directory holding the three sensor logs")
    parser.add_argument("--truth", type=Path, help="truth CSV (default: the *_truth.csv in --results)")
    args = parser.parse_args()
    if not args.aetherion_header and not args.results:
        parser.error("give --aetherion-header, --results, or both")

    failures: list[str] = []
    required = DEFAULT_REQUIRED
    if args.aetherion_header:
        required = parse_aetherion(args.aetherion_header)
        check_upstream(required, failures)
    if args.results:
        truth = args.truth
        if truth is None:
            candidates = sorted(args.results.glob("*_truth.csv"))
            if len(candidates) != 1:
                parser.error(f"name the truth log with --truth ({len(candidates)} *_truth.csv in {args.results})")
            truth = candidates[0]
        check_results(args.results, truth, required, failures)

    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        return 1
    print("log contract: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
