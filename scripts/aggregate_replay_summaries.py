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
UNIT_SLUGS = {"m": "m", "m/s": "mps", "deg": "deg", "rad": "rad", "rad/s": "radps"}

# Column order: identity, outcome, counts, consistency, accuracy. Unknown figures go after their group.
IDENTITY = ["run", "seed", "turbulence_w20_mps", "turbulence_seed", "stop_s", "passed", "summary"]
COUNTS = ["duration_s", "imu_steps", "gnss_pos_updates", "gnss_vel_updates", "baro_updates", "fixes_skipped"]
CONSISTENCY = ["anees_nav", "nis_gnss_pos", "nis_gnss_vel", "nis_baro"]
ACCURACY = ["rms_position_m", "rms_velocity_mps", "rms_tilt_deg", "rms_yaw_deg"]
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
    row["cosim_wall_s"] = manifest.get("checks", {}).get("cosim_wall_s")
    row["hemerion_commit"] = manifest.get("hemerion_commit", "")
    row["aetherion_commit"] = manifest.get("aetherion_commit", "")
    return row


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
    known = set(IDENTITY + TRAILER).union(*NUMERIC_GROUPS)
    extra = sorted({key for row in rows for key in row} - known)
    ordered = list(IDENTITY)
    for group, prefix in zip(NUMERIC_GROUPS, ("n_", "nis_", "rms_")):
        ordered += group + [key for key in extra if key.startswith(prefix)]
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
    return (seed is None, seed if isinstance(seed, int) else 0, str(row.get("run")))


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
