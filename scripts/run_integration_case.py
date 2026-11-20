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

The gate runs with the defaults. --seed, --stop and --turbulence (with
--turbulence-seed) fly the same case over other realisations, and --case
flies another NESC check-case -- 12 through f16_trim_cosim, 13.1 to 13.4
through examples/f16_autopilot_ecos's f16_autopilot_cosim (--autopilot-bin)
-- as .github/workflows/step7-log-batch.yml does for Aetherion's step 7. The
rates and the step are not options, so every run stays at the baseline.
Without --case, --stop or --turbulence the co-simulation's command line is
the gate's, unchanged.

Case 12 (Mach 2.01 at 30 013 ft) is the degraded mode: the receiver is
outside its airborne dynamics envelope for the whole run and the BMP390
below its rated floor. So the GNSS check there wants every epoch logged and
none of them a 3D fix (a fix would mean the case no longer tests what it is
for), and the replay runs on the IMU and the barometer alone.

Everything a reader needs to reproduce or judge the run lands in --out:
the logs, EKFReplay_summary.txt, EKFReplay_states.csv, EKFReplay_updates.csv,
the two processes' console logs, and manifest.json with both repositories'
commits, every command line, the host, anything --environment adds, and the
outcome of each check. Exit status 0
when every check passes, 1 otherwise, with each failure named.

One more thing is measured but never fails a run: whether the flight
computer's arrival stamps (`host_time_s`) are a clock. scripts/
fit_arrival_clock.py fits them per stream against simulation time; the fit
lands in the manifest as `clock_fit`, and a stream outside --clock-max-scale-
error / --clock-max-rms-s (0.02 and 0.02 s) is a warning (`clock_ok` false,
`clock_flags`), printed as WARN. A host that cannot keep --rtf 1 -- the plant
FMU's step costing more than the step, on a slow runner -- produces exactly
that, and the sensor values are still right; only a filter that would use the
stamps as a clock needs to know. The host's own pacing report is in the
sidecar (`pacing_*`).

The same goes for whether the flight computer had brought the IMU, BMP390 and
MMC5983MA up before simulation time started (Hemerion/part_bring_up.h): the
host writes `bring_up_{imu,baro,mag}_ready` to the sidecar, the manifest
records them as `bring_up_ready`, `bring_up_flags` and `bring_up_ok`, and a part
that was not ready is a WARN, never a failure. Such a run's values are still
right; they are only not a function of the seed alone, so it does not
reproduce bit for bit.

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
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_arrival_clock  # noqa: E402  (beside this script)

REPO = Path(__file__).resolve().parent.parent
PIN_FILE = REPO / "cmake" / "aetherion_plant_version.cmake"
EXE = ".exe" if os.name == "nt" else ""

# The case. Rates are the sensor-fusion baseline (TODO-sensor-fusion.md, step 4):
# one IMU sample per 0.01 s step, a 10 Hz receiver whatever the step. case,
# seed, stop_s and the turbulence keys are the gate's values and can be
# overridden from the command line; the rest cannot.
CASE = {
    "case": "11",
    "seed": 11,
    "stop_s": 20.0,
    "step_s": 0.01,
    "gps_period_s": 0.1,
    "imu_rate_hz": 100.0,
    "rtf": 1.0,
    "replay_config": "nav-baro-gnss16",
    "turbulence_w20_mps": 0.0,
    "turbulence_seed": None,
}

# 20 s at 10 Hz is 200 epochs; case 11 keeps a fix for all of them. Fewer than
# three quarters of the epochs as 3D fixes (150 of the gate's 200) means the
# receiver or the link broke, not that the filter had little to work with.
MIN_3D_FIX_FRACTION = 0.75

# When the arrival stamps count as a usable clock; see fit_arrival_clock.py for where the numbers come from.
CLOCK_MAX_SCALE_ERROR = fit_arrival_clock.DEFAULT_MAX_SCALE_ERROR
CLOCK_MAX_RMS_S = fit_arrival_clock.DEFAULT_MAX_RMS_S

# The NESC check-cases this runner flies: the host that flies each, its reference window (the stop time when
# --stop is not given, except for the gate's case 11, which flies 20 s) and whether the receiver holds a fix.
CASES = {
    "11": {"host": "f16_trim_cosim", "window_s": 200.0, "gnss_fix": True},
    "12": {"host": "f16_trim_cosim", "window_s": 200.0, "gnss_fix": False},
    "13.1": {"host": "f16_autopilot_cosim", "window_s": 60.0, "gnss_fix": True},
    "13.2": {"host": "f16_autopilot_cosim", "window_s": 60.0, "gnss_fix": True},
    "13.3": {"host": "f16_autopilot_cosim", "window_s": 240.0, "gnss_fix": True},
    "13.4": {"host": "f16_autopilot_cosim", "window_s": 240.0, "gnss_fix": True},
}


def min_3d_fixes(case: dict) -> int:
    return int(MIN_3D_FIX_FRACTION * round(case["stop_s"] / case["gps_period_s"]))

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


def check_gnss_velocity(gps_csv: Path, failures: list[str], min_fixes: int, max_fixes: int | None = None,
                        min_epochs: int = 0) -> int:
    """Every 3D fix carries finite NED velocity and a positive speed accuracy; there are at least min_fixes 3D
    fixes (and at most max_fixes, when given) and at least min_epochs epochs in all. Returns the 3D fix count."""
    if not gps_csv.exists():
        failures.append(f"{gps_csv.name}: missing")
        return 0
    fixes = 0
    epochs = 0
    with gps_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [name.strip() for name in reader.fieldnames or []]
        missing = [name for name in GNSS_VELOCITY + ["fix_type"] if name not in reader.fieldnames]
        if missing:
            failures.append(f"{gps_csv.name}: no column(s) {', '.join(missing)}")
            return 0
        for line, row in enumerate(reader, start=2):
            epochs += 1
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
    if fixes < min_fixes:
        failures.append(f"{gps_csv.name}: {fixes} 3D fixes, expected at least {min_fixes}")
    if max_fixes is not None and fixes > max_fixes:
        failures.append(f"{gps_csv.name}: {fixes} 3D fixes, expected at most {max_fixes}")
    if epochs < min_epochs:
        failures.append(f"{gps_csv.name}: {epochs} epochs logged, expected at least {min_epochs}")
    return fixes


def assess_clock(out: Path, max_scale_error: float, max_rms_s: float) -> dict:
    """The arrival-stamp fit of a run directory, for the manifest: `clock_fit` (per stream), `clock_bounds`,
    `clock_flags` (each stream out of bounds, or unusable) and `clock_ok` (none flagged, and something to fit)."""
    fits = fit_arrival_clock.fit_run(out)
    flags = fit_arrival_clock.violations(fits, max_scale_error, max_rms_s)
    if not fits:
        flags = ["no flight-computer logs to fit"]
    return {
        "clock_fit": fits,
        "clock_bounds": {"max_scale_error": max_scale_error, "max_rms_s": max_rms_s},
        "clock_flags": flags,
        "clock_ok": not flags,
    }


BRING_UP_PARTS = ("imu", "baro", "mag")


def assess_bring_up(sidecar: Path) -> dict:
    """Whether each bus part was up at t = 0, from the host's sidecar, for the manifest: `bring_up_ready` (part ->
    True/False, None where the host wrote no key), `bring_up_flags` (each part not ready, or no keys at all) and
    `bring_up_ok` (every part ready; None when the host recorded nothing, as hosts before 2026-11-20 did not)."""
    keys: dict[str, str] = {}
    if sidecar.exists():
        for line in sidecar.read_text(encoding="utf-8", errors="replace").splitlines():
            key, sep, value = line.partition("=")
            if sep:
                keys[key.strip()] = value.strip()
    ready: dict[str, bool | None] = {}
    for part in BRING_UP_PARTS:
        value = keys.get(f"bring_up_{part}_ready")
        ready[part] = (value == "1") if value in ("0", "1") else None
    recorded = [part for part in BRING_UP_PARTS if ready[part] is not None]
    if not recorded:
        return {"bring_up_ready": ready, "bring_up_flags": ["the host recorded no bring-up keys"], "bring_up_ok": None}
    flags = [f"{part}: not brought up before t = 0" for part in BRING_UP_PARTS if ready[part] is False]
    flags += [f"{part}: no bring-up key" for part in BRING_UP_PARTS if ready[part] is None]
    return {"bring_up_ready": ready, "bring_up_flags": flags, "bring_up_ok": not flags}


def run(command: list[str], log: Path, timeout_s: float, cwd: Path) -> subprocess.CompletedProcess:
    with log.open("w", encoding="utf-8") as out:
        return subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, timeout=timeout_s, cwd=cwd, check=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", type=Path, required=True,
                        help="directory holding f16_trim_cosim and f16_flight_computer")
    parser.add_argument("--autopilot-bin", type=Path,
                        help="directory holding f16_autopilot_cosim, for --case 13.x "
                             "(default: f16_autopilot_ecos beside --bin)")
    parser.add_argument("--case", choices=list(CASES), default=CASE["case"],
                        help=f"NESC check-case (default {CASE['case']}, the gate's)")
    parser.add_argument("--ekfreplay", type=Path, required=True,
                        help="Aetherion's EKFReplay executable (still built as EkfReplay at the pinned commit)")
    parser.add_argument("--out", type=Path, required=True, help="output directory (created; must not hold logs)")
    parser.add_argument("--aetherion-header", type=Path, help="installed Aetherion/Estimation/LogContract.h")
    parser.add_argument("--aetherion-commit", default="",
                        help="the Aetherion commit EKFReplay and the plant FMU were built from")
    parser.add_argument("--seed", type=int, default=CASE["seed"],
                        help=f"sensor seed, --seed of f16_trim_cosim (default {CASE['seed']}, the gate's)")
    parser.add_argument("--stop", type=float,
                        help=f"simulated seconds (default {CASE['stop_s']:g}, the gate's, for case {CASE['case']}; "
                             "the case's reference window for any other)")
    parser.add_argument("--turbulence", type=float, default=CASE["turbulence_w20_mps"],
                        help="MIL-F-8785C W20 [m/s], --turbulence of f16_trim_cosim (default 0: calm, the gate's)")
    parser.add_argument("--turbulence-seed", type=int,
                        help="turbulence realisation (default: the --seed value; needs --turbulence)")
    parser.add_argument("--environment", type=Path,
                        help="JSON object recorded as the manifest's 'environment' (runner, compiler, CI run)")
    parser.add_argument("--timeout-s", type=float,
                        help="wall-clock cap per process (default 600 s, or 3 x --stop + 300 s if longer)")
    parser.add_argument("--clock-max-scale-error", type=float, default=CLOCK_MAX_SCALE_ERROR,
                        help=f"arrival-stamp clock: warn when |scale - 1| exceeds this "
                             f"(default {CLOCK_MAX_SCALE_ERROR:g}; a warning, never a failure)")
    parser.add_argument("--clock-max-rms-s", type=float, default=CLOCK_MAX_RMS_S,
                        help=f"arrival-stamp clock: warn when the fit residual exceeds this rms [s] "
                             f"(default {CLOCK_MAX_RMS_S:g}; a warning, never a failure)")
    args = parser.parse_args()
    if args.seed <= 0:
        parser.error("--seed must be positive: 0 makes every sensor a fresh, unreproducible draw")
    if args.stop is None:
        args.stop = CASE["stop_s"] if args.case == CASE["case"] else CASES[args.case]["window_s"]
    if args.stop <= 0.0 or args.turbulence < 0.0:
        parser.error("--stop must be positive and --turbulence not negative")
    if args.turbulence_seed is not None and args.turbulence == 0.0:
        parser.error("--turbulence-seed without --turbulence would name a realisation nobody flies")
    if args.timeout_s is None:
        args.timeout_s = max(600.0, 3.0 * args.stop + 300.0)
    case = dict(CASE, case=args.case, seed=args.seed, stop_s=args.stop, **CASES[args.case])
    if args.turbulence > 0.0:
        # One integer names a realisation: the turbulence seed is the sensor seed unless given. The sensors' own
        # seeds are hashed from --seed per part (derived_seed in cosim_host_main.cpp), so they do not coincide.
        case["turbulence_w20_mps"] = args.turbulence
        case["turbulence_seed"] = args.turbulence_seed if args.turbulence_seed is not None else args.seed
    # The child processes run in --out, so every path given relative to the caller's directory is resolved first.
    args.bin = args.bin.resolve()
    args.autopilot_bin = (args.autopilot_bin or args.bin.parent / "f16_autopilot_ecos").resolve()
    host_dir = args.autopilot_bin if case["host"] == "f16_autopilot_cosim" else args.bin
    args.ekfreplay = args.ekfreplay.resolve()
    if args.aetherion_header:
        args.aetherion_header = args.aetherion_header.resolve()

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.glob("*.csv")):
        parser.error(f"{out} already holds CSV logs; give an empty --out")

    pin = read_pin()
    manifest: dict = {
        "case": case,
        "hemerion_commit": os.environ.get("GITHUB_SHA") or git_head(REPO),
        "hemerion_head": git_head(REPO),
        "aetherion_commit": args.aetherion_commit,
        "aetherion_pin": pin,
        "host": {"platform": platform.platform(), "machine": platform.machine(),
                 "python": platform.python_version()},
        "environment": json.loads(args.environment.read_text(encoding="utf-8")) if args.environment else {},
        "commands": {},
        "checks": {},
        "failures": [],
    }
    failures: list[str] = manifest["failures"]

    flight_computer = [
        str(args.bin / f"f16_flight_computer{EXE}"),
        "--csv", str(out / "gps_fixes.csv"),
        "--imu-csv", str(out / "imu_samples.csv"),
        "--baro-csv", str(out / "baro_samples.csv"),
        "--mag-csv", str(out / "mag_samples.csv"),
        "--fix-period", str(case["gps_period_s"]),
        "--max-wall-s", str(int(args.timeout_s)),
    ]
    cosim = [
        str(host_dir / f"{case['host']}{EXE}"),
        "--case", case["case"],
        "--seed", str(case["seed"]),
        "--stop", str(case["stop_s"]),
        "--step", str(case["step_s"]),
        "--gps-period", str(case["gps_period_s"]),
        "--imu-rate", str(case["imu_rate_hz"]),
        "--rtf", str(case["rtf"]),
        "--csv", str(out / "f16_truth.csv"),
    ]
    if case["turbulence_w20_mps"] > 0.0:
        cosim += ["--turbulence", str(case["turbulence_w20_mps"]), "--turbulence-seed", str(case["turbulence_seed"])]
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
                failures.append(f"{case['host']} exited {host.returncode} (cosim.log)")
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

    # 3. GNSS velocity: values, not only names. Where the case holds no fix (12), the receiver and the link must
    # still have delivered the epochs, and none of them may be a 3D fix.
    if case["gnss_fix"]:
        fix_limits = {"min_fixes": min_3d_fixes(case)}
    else:
        fix_limits = {"min_fixes": 0, "max_fixes": 0, "min_epochs": min_3d_fixes(case)}
    manifest["checks"]["gnss_3d_fixes"] = check_gnss_velocity(out / "gps_fixes.csv", failures, **fix_limits)

    # 4. The reference filter over the logs.
    replay = [str(args.ekfreplay), "--logs", str(out), "--truth", str(out / "f16_truth.csv"), "--config",
              case["replay_config"], "--output", str(out / "EKFReplay")]
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

    # 6. The arrival stamps as a clock: recorded and flagged, never a failure (see the module docstring).
    clock = assess_clock(out, args.clock_max_scale_error, args.clock_max_rms_s)
    manifest.update(clock)
    # 7. The bus parts up before t = 0: recorded and flagged, never a failure (see the module docstring).
    bring_up = assess_bring_up(out / "f16_truth.config")
    manifest.update(bring_up)

    manifest["passed"] = not failures
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    if summary.exists():
        print(summary.read_text(encoding="utf-8", errors="replace").rstrip())
    if clock["clock_fit"]:
        print("arrival stamps against simulation time (scripts/fit_arrival_clock.py):")
        print(fit_arrival_clock.table(clock["clock_fit"]))
    for flag in clock["clock_flags"]:
        print(f"WARN: arrival stamps are not a clock for this run: {flag}")
    for flag in bring_up["bring_up_flags"]:
        print(f"WARN: sensor logs are not a function of the seed alone for this run: {flag}")
    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        return 1
    print(f"integration case {case['case']} seed {case['seed']}: all checks passed ({manifest['checks']['gnss_3d_fixes']} 3D fixes, "
          f"{manifest['checks']['states_rows']} state rows, {manifest['checks']['updates_rows']} updates)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
