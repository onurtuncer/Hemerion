#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""One table from many EKFReplay runs: a row per run, then statistics over them.

Each run is a directory written by scripts/run_integration_case.py: the logs,
the f16_truth.config sidecar, manifest.json and EKFReplay_summary.txt. Give
run directories, or directories to search for them (any depth); a directory
counts as a run when it holds manifest.json or EKFReplay_summary.txt, so a run
whose replay failed still gets a row, marked, rather than vanishing from the
count.

From the summary it reads the update counts ("duration:" line), the
second-half ANEES and NIS/dim per measurement ("second half:") and the
second-half RMS errors ("second half RMS:"). Measurements are read by name, so
one the replay adds later gets its own column without a change here. The seed
and turbulence come from the sidecar, which is what the co-simulation actually
flew, and from the manifest where the sidecar is missing.

Writes the table as Markdown (stdout, or --markdown; append it to
$GITHUB_STEP_SUMMARY in CI) and as CSV (--csv). The statistics are mean,
sample standard deviation, minimum and maximum over the runs that report each
figure; whether those figures are consistent is Aetherion's step 7, not this
script's. Exit status 0 when at least one run is found (with --strict, only
when every run passed and has a summary), 2 when none is.

Whether a run's arrival stamps are a clock gets its own columns: clock_scale
(the scale of the stream furthest from 1), clock_rms_s (the largest residual
of any stream) and clock_ok, from the manifest's `clock_fit` -- or, for a run
made before the runner recorded it, fitted here from the logs
(scripts/fit_arrival_clock.py, default bounds). pacing_max_lag_s and
pacing_rtf_honoured come from the sidecar where the host wrote them. A run
whose stamps are not a clock still has valid sensor values; the columns only
say which seeds a filter could use the stamps of.

Whether the flight computer had brought the bus parts up before simulation
time started (Hemerion/part_bring_up.h) gets columns too: bring_up_ok and
bring_up_{imu,baro,mag}_ready, from the manifest's `bring_up_ready` or else
the sidecar. A run made before the hosts recorded it (2026-11-20) shows them
empty, not "no". A "no" run's values are right but do not reproduce bit for
bit.

    python scripts/aggregate_replay_summaries.py step7-logs --csv step7.csv \\
        --markdown step7.md
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import re
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_arrival_clock  # noqa: E402  (beside this script)

SUMMARY = "EKFReplay_summary.txt"
MANIFEST = "manifest.json"
SIDECAR = "f16_truth.config"

# A number as a C++ stream prints it, or any spelling of a non-finite one (see run_integration_case.NON_FINITE).
NUMBER = r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?|nan(?:\(\w*\))?|inf(?:inity)?|\d\.#\w+)"

# Fixed names for the figures the pinned replay prints, so the columns keep their meaning; anything else is slugged.
COUNT_NAMES = {
    "IMU steps": "imu_steps",
    "GNSS position": "gnss_pos_updates",
    "GNSS velocity": "gnss_vel_updates",
    "barometer updates": "baro_updates",
    "fixes skipped": "fixes_skipped",
}
NIS_NAMES = {"GNSS pos": "nis_gnss_pos", "GNSS vel": "nis_gnss_vel", "baro": "nis_baro"}
# The update count behind each NIS. The replay prints NIS 0 for a measurement it never applied (case 12 has no GNSS
# fix); that is no figure at all, so it is dropped rather than averaged in as a perfect fit.
NIS_COUNTS = {"nis_gnss_pos": "gnss_pos_updates", "nis_gnss_vel": "gnss_vel_updates", "nis_baro": "baro_updates"}
UNIT_SLUGS = {"m": "m", "m/s": "mps", "deg": "deg", "rad": "rad", "rad/s": "radps"}

# Column order: identity, outcome, counts, consistency, accuracy. Unknown figures go after their group.
IDENTITY = ["run", "case", "seed","turbulence_w20_mps", "turbulence_seed", "stop_s", "passed", "summary"]
COUNTS = ["duration_s", "imu_steps", "gnss_pos_updates", "gnss_vel_updates", "baro_updates", "fixes_skipped"]
CONSISTENCY = ["anees_nav", "nis_gnss_pos", "nis_gnss_vel", "nis_baro"]
ACCURACY = ["rms_position_m", "rms_velocity_mps", "rms_tilt_deg", "rms_yaw_deg"]
CLOCK = ["clock_scale", "clock_rms_s", "clock_ok", "pacing_max_lag_s", "pacing_rtf_honoured"]
BRING_UP_PARTS = ("imu", "baro", "mag")
BRING_UP = ["bring_up_ok"] + [f"bring_up_{part}_ready" for part in BRING_UP_PARTS]
TRAILER = ["cosim_wall_s", "convention", "hemerion_commit", "aetherion_commit"]
NUMERIC_GROUPS = (COUNTS, CONSISTENCY, ACCURACY)


def slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def to_float(text: str) -> float:
    try:
        return float(text)
    except ValueError:
        # nan(ind), 1.#INF and the like: not a number anyone can average, but not a missing one either.
        return math.inf if "inf" in text.lower() else math.nan


def parse_summary(text: str) -> dict[str, object]:
    """The figures of one EKFReplay summary, keyed by column name. Lines it does not know are ignored."""
    figures: dict[str, object] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("convention:"):
            figures["convention"] = line.split(":", 1)[1].strip()
        elif line.startswith("duration:"):
            items = [item.strip() for item in line.split(":", 1)[1].split(",")]
            match = re.fullmatch(rf"({NUMBER}) s", items[0]) if items else None
            if match:
                figures["duration_s"] = to_float(match.group(1))
            for item in items[1:]:
                match = re.fullmatch(r"(\d+) (.+)", item)
                if match:
                    name = match.group(2).strip()
                    figures[COUNT_NAMES.get(name, "n_" + slug(name))] = int(match.group(1))
        elif line.startswith("second half RMS:"):
            for item in line.split(":", 1)[1].split(","):
                match = re.fullmatch(rf"\s*(\w[\w ]*?) ({NUMBER}) (\S+)\s*", item)
                if match:
                    unit = UNIT_SLUGS.get(match.group(3), slug(match.group(3)))
                    figures[f"rms_{slug(match.group(1))}_{unit}"] = to_float(match.group(2))
        elif line.startswith("second half:"):
            body = line.split(":", 1)[1]
            match = re.search(rf"ANEES \([^)]*\) ({NUMBER})", body)
            if match:
                figures["anees_nav"] = to_float(match.group(1))
            nis = body.split("NIS/dim", 1)
            if len(nis) == 2:
                for item in nis[1].split(","):
                    match = re.fullmatch(rf"\s*(.+?) ({NUMBER})\s*", item)
                    if match:
                        name = match.group(1).strip()
                        figures[NIS_NAMES.get(name, "nis_" + slug(name))] = to_float(match.group(2))
    for nis, count in NIS_COUNTS.items():
        if figures.get(count) == 0:
            figures.pop(nis, None)
    return figures


def read_sidecar(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            key, sep, value = line.partition("=")
            if sep:
                values[key.strip()] = value.strip()
    return values


def read_run(run: Path, root: Path) -> dict[str, object]:
    """One row: identity from the sidecar (else the manifest), outcome from the manifest, figures from the summary."""
    manifest = {}
    if (run / MANIFEST).exists():
        try:
            manifest = json.loads((run / MANIFEST).read_text(encoding="utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            manifest = {}  # a truncated manifest costs this row its outcome, not the table its other rows
    case = manifest.get("case", {})
    sidecar = read_sidecar(run / SIDECAR)

    name = run.relative_to(root).as_posix() if run != root else run.name
    row: dict[str, object] = {"run": name or run.name}
    seed = sidecar.get("seed", case.get("seed"))
    check_case = sidecar.get("check_case", case.get("case"))
    row["case"] = str(check_case) if check_case not in (None, "") else None
    row["seed"] = int(seed) if seed not in (None, "") else None
    turbulence = to_float(str(sidecar.get("turbulence_w20_mps", case.get("turbulence_w20_mps", 0.0)) or 0.0))
    row["turbulence_w20_mps"] = turbulence
    # The sidecar always names a turbulence seed (default 1); it only means something when there is turbulence.
    turbulence_seed = sidecar.get("turbulence_seed", case.get("turbulence_seed"))
    row["turbulence_seed"] = int(turbulence_seed) if turbulence > 0.0 and turbulence_seed not in (None, "") else None
    stop = sidecar.get("stop_s", case.get("stop_s"))
    row["stop_s"] = to_float(str(stop)) if stop not in (None, "") else None
    row["passed"] = manifest.get("passed") if manifest else None
    row["summary"] = (run / SUMMARY).exists()
    if row["summary"]:
        row.update(parse_summary((run / SUMMARY).read_text(encoding="utf-8", errors="replace")))
    row.update(clock_columns(run, manifest, sidecar))
    row.update(bring_up_columns(manifest, sidecar))
    row["cosim_wall_s"] = manifest.get("checks", {}).get("cosim_wall_s")
    row["hemerion_commit"] = manifest.get("hemerion_commit", "")
    row["aetherion_commit"] = manifest.get("aetherion_commit", "")
    return row


def clock_columns(run: Path, manifest: dict, sidecar: dict[str, str]) -> dict[str, object]:
    """clock_scale, clock_rms_s and clock_ok from the manifest's fit, or from the logs when it has none; the host's
    pacing keys from the sidecar. Each is None where there is nothing to read it from."""
    fits = manifest.get("clock_fit")
    ok = manifest.get("clock_ok")
    if fits is None:
        fits = fit_arrival_clock.fit_run(run)
        ok = None
        if fits:
            ok = not fit_arrival_clock.violations(fits, fit_arrival_clock.DEFAULT_MAX_SCALE_ERROR,
                                                  fit_arrival_clock.DEFAULT_MAX_RMS_S)
    usable = [fit for fit in (fits or {}).values() if "error" not in fit]
    columns: dict[str, object] = {"clock_scale": None, "clock_rms_s": None, "clock_ok": ok if fits else None}
    if usable:
        columns["clock_scale"] = max((fit["scale"] for fit in usable), key=lambda scale: abs(scale - 1.0))
        columns["clock_rms_s"] = max(fit["rms_s"] for fit in usable)
    lag = sidecar.get("pacing_max_lag_s")
    columns["pacing_max_lag_s"] = to_float(lag) if lag not in (None, "") else None
    honoured = sidecar.get("pacing_rtf_honoured")
    columns["pacing_rtf_honoured"] = (honoured == "1") if honoured in ("0", "1") else None
    return columns


def bring_up_columns(manifest: dict, sidecar: dict[str, str]) -> dict[str, object]:
    """bring_up_{part}_ready from the manifest's `bring_up_ready`, else the sidecar; bring_up_ok when every part says.
    None wherever nothing was recorded, so a run from before the hosts wrote the keys is empty, not "no"."""
    recorded = manifest.get("bring_up_ready") or {}
    columns: dict[str, object] = {}
    for part in BRING_UP_PARTS:
        value = recorded.get(part)
        if value is None:
            text = sidecar.get(f"bring_up_{part}_ready")
            value = (text == "1") if text in ("0", "1") else None
        columns[f"bring_up_{part}_ready"] = value
    parts = [columns[f"bring_up_{part}_ready"] for part in BRING_UP_PARTS]
    columns["bring_up_ok"] = None if any(value is None for value in parts) else all(parts)
    return columns


def find_runs(paths: list[Path]) -> list[tuple[Path, Path]]:
    """(run directory, the path it was found under), each run once, in a stable order."""
    found: dict[Path, Path] = {}
    for root in paths:
        root = root.resolve()
        candidates = [root] if root.is_dir() else []
        candidates += sorted(path for path in root.rglob("*") if path.is_dir()) if root.is_dir() else []
        for directory in candidates:
            if (directory / MANIFEST).exists() or (directory / SUMMARY).exists():
                found.setdefault(directory, root)
    return list(found.items())


def columns(rows: list[dict[str, object]]) -> list[str]:
    known = set(IDENTITY + CLOCK + BRING_UP + TRAILER).union(*NUMERIC_GROUPS)
    extra = sorted({key for row in rows for key in row} - known)
    ordered = list(IDENTITY)
    for group, prefix in zip(NUMERIC_GROUPS, ("n_", "nis_", "rms_")):
        ordered += group + [key for key in extra if key.startswith(prefix)]
    ordered += CLOCK
    ordered += BRING_UP
    ordered += [key for key in extra if not key.startswith(("n_", "nis_", "rms_"))]
    return ordered + TRAILER


def numeric_columns(names: list[str], rows: list[dict[str, object]]) -> list[str]:
    numeric = []
    for name in names:
        if name in IDENTITY or name in TRAILER:
            continue
        values = [row.get(name) for row in rows if row.get(name) is not None]
        if values and all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in values):
            numeric.append(name)
    return numeric


def statistics_over(rows: list[dict[str, object]], names: list[str]) -> dict[str, dict[str, float]]:
    """Mean, sample standard deviation, min, max and count of each column over the runs that report it finitely."""
    stats: dict[str, dict[str, float]] = {}
    for name in names:
        values = [float(row[name]) for row in rows if isinstance(row.get(name), (int, float))
                  and math.isfinite(float(row[name]))]
        if not values:
            continue
        stats[name] = {
            "n": len(values),
            "mean": statistics.fmean(values),
            "std": statistics.stdev(values) if len(values) > 1 else math.nan,
            "min": min(values),
            "max": max(values),
        }
    return stats


def cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return "nan" if math.isnan(value) else f"{value:.6g}"
    return str(value)


def write_csv(rows: list[dict[str, object]], names: list[str], out: io.TextIOBase) -> None:
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(names)
    for row in rows:
        writer.writerow([cell(row.get(name)) for name in names])


def markdown(rows: list[dict[str, object]], names: list[str], title: str) -> str:
    numeric = numeric_columns(names, rows)
    stats = statistics_over(rows, numeric)
    passed = sum(1 for row in rows if row.get("passed") is True)
    replayed = sum(1 for row in rows if row.get("summary"))
    lines = [f"## {title}", ""]
    lines.append(f"{len(rows)} run(s): {passed} passed every check, {replayed} have an EKFReplay summary.")
    for key, label in (("hemerion_commit", "Hemerion"), ("aetherion_commit", "Aetherion")):
        commits = sorted({str(row.get(key)) for row in rows if row.get(key)})
        lines.append(f"{label}: {', '.join(f'`{commit}`' for commit in commits) or 'not recorded'}.")
    lines += ["", "Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).", ""]

    shown = [name for name in names if name not in ("hemerion_commit", "aetherion_commit", "summary")]
    lines.append("| " + " | ".join(shown) + " |")
    lines.append("|" + "|".join("---:" if name in numeric else "---" for name in shown) + "|")
    for row in rows:
        lines.append("| " + " | ".join(cell(row.get(name)) for name in shown) + " |")

    if stats:
        lines += ["", "Over the runs that report each figure (finite values only; std is the sample one):", ""]
        lines.append("| figure | n | mean | std | min | max |")
        lines.append("|---|---:|---:|---:|---:|---:|")
        for name, s in stats.items():
            lines.append(f"| {name} | {s['n']} | {cell(s['mean'])} | {cell(s['std'])} | {cell(s['min'])} | "
                         f"{cell(s['max'])} |")
    return "\n".join(lines) + "\n"


def sort_key(row: dict[str, object]) -> tuple:
    seed = row.get("seed")
    return (str(row.get("case") or ""), seed is None, seed if isinstance(seed, int) else 0, str(row.get("run")))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", type=Path, nargs="+", help="run directories, or directories to search for them")
    parser.add_argument("--csv", type=Path, help="write the per-run table as CSV here")
    parser.add_argument("--markdown", type=Path, help="write the Markdown here (default: stdout)")
    parser.add_argument("--title", default="EKFReplay over seeds", help="Markdown heading")
    parser.add_argument("--strict", action="store_true",
                        help="exit 1 unless every run passed its checks and has a summary")
    args = parser.parse_args(argv)

    rows = sorted((read_run(run, root) for run, root in find_runs(args.paths)), key=sort_key)
    if not rows:
        print(f"no run directories ({MANIFEST} or {SUMMARY}) under {', '.join(map(str, args.paths))}",
              file=sys.stderr)
        return 2
    names = columns(rows)
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as out:
            write_csv(rows, names, out)
    text = markdown(rows, names, args.title)
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    if args.strict and not all(row.get("passed") is True and row.get("summary") for row in rows):
        print("not every run passed its checks and has a summary", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
