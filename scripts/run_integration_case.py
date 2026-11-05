#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""One small sensor-fusion integration case, end to end, as a gate.

Flies NESC check-case 11 (F-16 trim, calm air) through the five sensor FMUs
and the unmodified on-target drivers at the baseline rates -- IMU 100 Hz,
GNSS 10 Hz, 0.01 s communication step -- for 20 simulated seconds with seed
11, then hands the logs to Aetherion's reference filter:

  1. f16_flight_computer and f16_trim_cosim, paced at --rtf 1 (unpaced, the
     polled BMP390 is starved: examples/f16_trim_ecos/README.md), write
     gps_fixes.csv, imu_samples.csv, baro_samples.csv, f16_truth.csv and
     its f16_truth.config sidecar into --out.
  2. scripts/check_log_contract.py checks every header and sidecar key
     Aetherion's replay binds (against the installed LogContract.h when
     --aetherion-header is given).
  3. The GNSS velocity fields must carry values, not only names: every 3D
     fix has finite vel_north/east/down_mps and a positive
     speed_accuracy_mps, and there are enough 3D fixes to fly a filter on.
  4. Aetherion's EKFReplay replays the logs with --config nav-baro-gnss16 and
     must exit 0.
  5. Its summary, states and updates must be finite throughout.

Everything a reader needs to reproduce or judge the run lands in --out:
the logs, EKFReplay_summary.txt, EKFReplay_states.csv, EKFReplay_updates.csv,
the two processes' console logs, and manifest.json with both repositories'
commits, every command line and the outcome of each check. Exit status 0
when every check passes, 1 otherwise, with each failure named.

    python scripts/run_integration_case.py \\
        --bin build/examples-native/examples/f16_trim_ecos \\
        --ekfreplay <EKFReplay executable> \\
        --aetherion-header <prefix>/include/Aetherion/Estimation/LogContract.h \\
        --aetherion-commit <sha> --out integration-case11
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PIN_FILE = REPO / "cmake" / "aetherion_plant_version.cmake"
EXE = ".exe" if os.name == "nt" else ""

# The case. Rates are the sensor-fusion baseline (TODO-sensor-fusion.md, step 4):
# one IMU sample per 0.01 s step, a 10 Hz receiver whatever the step.
CASE = {
    "case": "11",
    "seed": 11,
    "stop_s": 20.0,
    "step_s": 0.01,
    "gps_period_s": 0.1,
    "imu_rate_hz": 100.0,
    "rtf": 1.0,
    "replay_config": "nav-baro-gnss16",
}

# 20 s at 10 Hz is 200 epochs; case 11 keeps a fix for all of them. Fewer than
# this many 3D fixes means the receiver or the link broke, not that the filter
# had little to work with.
MIN_3D_FIXES = 150

GNSS_VELOCITY = ["vel_north_mps", "vel_east_mps", "vel_down_mps", "speed_accuracy_mps"]


def read_pin() -> dict[str, str]:
    text = PIN_FILE.read_text(encoding="utf-8")
    pin = {}
    for key in ("VERSION", "COMMIT"):
        match = re.search(rf"^set\(HEMERION_AETHERION_PIN_{key} (\S+)\)", text, re.MULTILINE)
        pin[key.lower()] = match.group(1) if match else ""
    return pin


def git_head(path: Path) -> str:
    try:
        return subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def is_finite_number(cell: str) -> bool:
    try:
        return math.isfinite(float(cell))
    except ValueError:
        return False


# Any spelling of a non-finite value a C++ runtime may print: nan, -nan, nan(ind), -nan(ind) (MSVC), inf, -inf,
# infinity, 1.#INF and 1.#QNAN (old MSVC). Some of these float() cannot parse, so they are matched as text.
NON_FINITE = re.compile(r"^[-+]?(?:nan(?:\(\w*\))?|inf(?:inity)?|\d\.#(?:inf|q?nan|ind)\w*)$", re.IGNORECASE)


def is_non_finite(cell: str) -> bool:
    """A cell that spells a NaN or an infinity. Text that is not a number (a measurement name) is not."""
    cell = cell.strip()
    if NON_FINITE.match(cell):
        return True
    try:
        return not math.isfinite(float(cell))
    except ValueError:
        return False


def check_finite_csv(path: Path, failures: list[str]) -> int:
    """No cell of any data row is a NaN or an infinity; label columns may hold text. Returns the row count."""
    if not path.exists():
        failures.append(f"{path.name}: missing")
        return 0
    rows = 0
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, [])
        if not header:
            failures.append(f"{path.name}: empty")
            return 0
        for line, row in enumerate(reader, start=2):
            rows += 1
            for name, cell in zip(header, row):
                if is_non_finite(cell):
                    failures.append(f"{path.name}:{line}: {name.strip()} = {cell.strip()!r} is not finite")
                    return rows
    if rows == 0:
        failures.append(f"{path.name}: no data rows")
    return rows


def check_gnss_velocity(gps_csv: Path, failures: list[str]) -> int:
    """Every 3D fix carries finite NED velocity and a positive speed accuracy. Returns the 3D fix count."""
    if not gps_csv.exists():
        failures.append(f"{gps_csv.name}: missing")
        return 0
    fixes = 0
    with gps_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [name.strip() for name in reader.fieldnames or []]
        missing = [name for name in GNSS_VELOCITY + ["fix_type"] if name not in reader.fieldnames]
        if missing:
            failures.append(f"{gps_csv.name}: no column(s) {', '.join(missing)}")
            return 0
        for line, row in enumerate(reader, start=2):
            if (row["fix_type"] or "").strip() != "2":
                continue
            fixes += 1
            for name in GNSS_VELOCITY:
                cell = (row[name] or "").strip()
                if not is_finite_number(cell):
                    failures.append(f"{gps_csv.name}:{line}: 3D fix with {name} = {cell!r}")
                    return fixes
            if float(row["speed_accuracy_mps"]) <= 0.0:
                failures.append(f"{gps_csv.name}:{line}: 3D fix with speed_accuracy_mps <= 0")
                return fixes
    if fixes < MIN_3D_FIXES:
        failures.append(f"{gps_csv.name}: {fixes} 3D fixes, expected at least {MIN_3D_FIXES}")
    return fixes


def run(command: list[str], log: Path, timeout_s: float, cwd: Path) -> subprocess.CompletedProcess:
    with log.open("w", encoding="utf-8") as out:
        return subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, timeout=timeout_s, cwd=cwd, check=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", type=Path, required=True,
                        help="directory holding f16_trim_cosim and f16_flight_computer")
    parser.add_argument("--ekfreplay", type=Path, required=True,
                        help="Aetherion's EKFReplay executable (still built as EkfReplay at the pinned commit)")
    parser.add_argument("--out", type=Path, required=True, help="output directory (created; must not hold logs)")
    parser.add_argument("--aetherion-header", type=Path, help="installed Aetherion/Estimation/LogContract.h")
    parser.add_argument("--aetherion-commit", default="",
                        help="the Aetherion commit EKFReplay and the plant FMU were built from")
    parser.add_argument("--timeout-s", type=float, default=600.0, help="wall-clock cap per process")
    args = parser.parse_args()

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.glob("*.csv")):
        parser.error(f"{out} already holds CSV logs; give an empty --out")

    pin = read_pin()
    manifest: dict = {
        "case": CASE,
        "hemerion_commit": os.environ.get("GITHUB_SHA") or git_head(REPO),
        "hemerion_head": git_head(REPO),
        "aetherion_commit": args.aetherion_commit,
        "aetherion_pin": pin,
        "commands": {},
        "checks": {},
        "failures": [],
    }
    failures: list[str] = manifest["failures"]

    flight_computer = [
        str(args.bin.resolve() / f"f16_flight_computer{EXE}"),
        "--csv", str(out / "gps_fixes.csv"),
        "--imu-csv", str(out / "imu_samples.csv"),
        "--baro-csv", str(out / "baro_samples.csv"),
        "--mag-csv", str(out / "mag_samples.csv"),
        "--radalt-csv", str(out / "radalt_samples.csv"),
        "--fix-period", str(CASE["gps_period_s"]),
        "--max-wall-s", str(int(args.timeout_s)),
    ]
    cosim = [
        str(args.bin.resolve() / f"f16_trim_cosim{EXE}"),
        "--case", CASE["case"],
        "--seed", str(CASE["seed"]),
        "--stop", str(CASE["stop_s"]),
        "--step", str(CASE["step_s"]),
        "--gps-period", str(CASE["gps_period_s"]),
        "--imu-rate", str(CASE["imu_rate_hz"]),
        "--rtf", str(CASE["rtf"]),
        "--csv", str(out / "f16_truth.csv"),
    ]
    manifest["commands"]["flight_computer"] = flight_computer
    manifest["commands"]["cosim"] = cosim

    # 1. The co-simulation. The flight computer waits for the sensor buses, so it
    # goes first; it exits on its own once the streams fall quiet.
    started = time.monotonic()
    with (out / "flight_computer.log").open("w", encoding="utf-8") as fc_log:
        fc = subprocess.Popen(flight_computer, stdout=fc_log, stderr=subprocess.STDOUT, cwd=out)
        try:
            host = run(cosim, out / "cosim.log", args.timeout_s, out)
            manifest["checks"]["cosim_exit"] = host.returncode
            if host.returncode != 0:
                failures.append(f"f16_trim_cosim exited {host.returncode} (cosim.log)")
            fc_exit = fc.wait(timeout=args.timeout_s)
        except subprocess.TimeoutExpired as expired:
            failures.append(f"{Path(str(expired.cmd[0] if isinstance(expired.cmd, list) else expired.cmd)).name} "
                            f"did not finish within {args.timeout_s:.0f} s")
            fc.kill()
            fc_exit = fc.wait()
    manifest["checks"]["flight_computer_exit"] = fc_exit
    manifest["checks"]["cosim_wall_s"] = round(time.monotonic() - started, 1)
    if fc_exit != 0:
        failures.append(f"f16_flight_computer exited {fc_exit} (flight_computer.log)")

    # 2. Headers and sidecar keys, against the installed contract where given.
    contract = [sys.executable, str(REPO / "scripts" / "check_log_contract.py"), "--results", str(out), "--truth",
                str(out / "f16_truth.csv")]
    if args.aetherion_header:
        contract += ["--aetherion-header", str(args.aetherion_header)]
    manifest["commands"]["log_contract"] = contract
    checked = run(contract, out / "log_contract.log", 120, out)
    manifest["checks"]["log_contract_exit"] = checked.returncode
    if checked.returncode != 0:
        failures.append("check_log_contract.py failed (log_contract.log)")

    # 3. GNSS velocity: values, not only names.
    manifest["checks"]["gnss_3d_fixes"] = check_gnss_velocity(out / "gps_fixes.csv", failures)

    # 4. The reference filter over the logs.
    replay = [str(args.ekfreplay.resolve()), "--logs", str(out), "--truth", str(out / "f16_truth.csv"), "--config",
              CASE["replay_config"], "--output", str(out / "EKFReplay")]
    manifest["commands"]["ekfreplay"] = replay
    summary = out / "EKFReplay_summary.txt"
    try:
        replayed = run(replay, summary, args.timeout_s, out)
        manifest["checks"]["ekfreplay_exit"] = replayed.returncode
        if replayed.returncode != 0:
            failures.append(f"EKFReplay exited {replayed.returncode} (EKFReplay_summary.txt)")
    except subprocess.TimeoutExpired:
        failures.append(f"EKFReplay did not finish within {args.timeout_s:.0f} s")
    except OSError as error:
        failures.append(f"EKFReplay could not be started: {error}")

    # 5. Finite output: the summary's numbers and every cell of both tables.
    if summary.exists():
        text = summary.read_text(encoding="utf-8", errors="replace")
        bad = sorted({token for token in re.split(r"[\s,:=/]+", text) if NON_FINITE.match(token)})
        if bad:
            failures.append(f"EKFReplay_summary.txt reports non-finite values: {', '.join(bad)}")
    manifest["checks"]["states_rows"] = check_finite_csv(out / "EKFReplay_states.csv", failures)
    manifest["checks"]["updates_rows"] = check_finite_csv(out / "EKFReplay_updates.csv", failures)

    if args.aetherion_commit and pin["commit"] and args.aetherion_commit != pin["commit"]:
        failures.append(f"Aetherion {args.aetherion_commit} is not the pinned {pin['commit']}")

    manifest["passed"] = not failures
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    if summary.exists():
        print(summary.read_text(encoding="utf-8", errors="replace").rstrip())
    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        return 1
    print(f"integration case {CASE['case']}: all checks passed ({manifest['checks']['gnss_3d_fixes']} 3D fixes, "
          f"{manifest['checks']['states_rows']} state rows, {manifest['checks']['updates_rows']} updates)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
