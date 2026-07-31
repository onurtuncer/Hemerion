# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Checks the co-simulation's rocket truth against the published Scenario 17 trajectory.

The sensor FMUs in this example are only as meaningful as the truth driving
them, so the plant has to be checked against something -- otherwise a
plausible-looking trajectory that is quietly flying the wrong mission profile
would still produce clean-looking GPS and IMU streams. NASA TM-2015-218675
Scenario 17 (Two-Stage Rocket to Orbit) publishes check-case simulator output
for exactly that purpose, and Aetherion ships it under
``data/Atmos_17_TwoStageRocketToOrbit/``:

    Atmos_17_sim_04.csv    0.01 s samples, 0..200 s
    Atmos_17_sim_05.csv    0.01 s samples, 0..200 s
    Atmos_17_sim_06.csv    0.1 s samples,  0..200 s

Those are three *independent* simulators run against the same case, and they do
not agree with each other: at t = 200 s sim_04 and sim_05 report 251.4 km while
sim_06 reports 234.5 km, a 7%% spread. Checking against any single one of them
would therefore be checking against that simulator's idiosyncrasies as much as
against the scenario. So the criterion here is the **envelope**: at each sample
the run must lie between the lowest and highest reference altitude, within a
tolerance. That is the strongest claim the published data actually supports.

Comparison is on altitude above MSL -- the one channel every simulator
tabulates, and the one the sensor models care about.

Usage (from the co-simulation working directory, after a run):

    python verify_trajectory.py --truth results/rocket_truth.csv \\
        --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_04.csv \\
        --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_05.csv \\
        --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_06.csv

Exits non-zero if any sample falls outside the envelope by more than
--tolerance, so this can run in CI once an Aetherion checkout is available to
it. With one --reference the envelope degenerates to that single trajectory and
the tolerance carries the whole comparison.

Only the standard library is required.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

FT_TO_M = 0.3048

# Mission events, from NASA TM-2015-218675 Scenario 17. The second is the one
# this script exists to catch: the TwoStageRocket FMU defaults to igniting
# stage 2 the moment stage 1 separates, which flies a different mission.
EVENTS = [
    (37.4, "stage 1 burnout / separation"),
    (131.8, "stage 2 ignition"),
    (193.0, "stage 2 burnout"),
]


def read_truth(path: Path) -> tuple[list[float], list[float]]:
    """Reads (time, altitude_m) out of Ecos csv_writer output."""
    with path.open(newline="") as f:
        rows = list(csv.reader(f))
    header = [h.strip() for h in rows[0]]
    names = [h.split("::")[-1].removeprefix("out.").split("[")[0] for h in header]
    time_index = names.index("time")
    alt_index = names.index("alt_m")
    times, altitudes = [], []
    for row in rows[1:]:
        if len(row) != len(names):
            continue
        times.append(float(row[time_index]))
        altitudes.append(float(row[alt_index]))
    return times, altitudes


def read_reference(path: Path) -> tuple[list[float], list[float]]:
    """Reads (time, altitude_m) out of a NASA check-case CSV (US customary)."""
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return ([float(r["time"]) for r in rows],
            [float(r["altitudeMsl_ft"]) * FT_TO_M for r in rows])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth", type=Path, default=Path("results/rocket_truth.csv"))
    parser.add_argument("--reference", type=Path, action="append", required=True,
                        help="NASA check-case CSV; repeat to build an envelope (recommended)")
    parser.add_argument("--tolerance", type=float, default=0.01,
                        help="allowed excursion outside the envelope, as a fraction of altitude (default 0.01)")
    parser.add_argument("--floor-m", type=float, default=100.0,
                        help="absolute allowance added to the fractional tolerance [m] (default 100)")
    args = parser.parse_args()

    times, altitudes = read_truth(args.truth)
    ours = dict(zip((round(t, 4) for t in times), altitudes))

    # Reference grids are uniform but not all the same rate, so index each by
    # rounded time and only compare where every source has a sample.
    references = {}
    for path in args.reference:
        ref_times, ref_altitudes = read_reference(path)
        references[path.name] = dict(zip((round(t, 4) for t in ref_times), ref_altitudes))

    common = set(ours)
    for table in references.values():
        common &= set(table)
    common = sorted(common)

    if not common:
        print(f"error: no overlapping samples between {args.truth} and the reference(s)", file=sys.stderr)
        return 2

    worst = (0.0, 0.0, 0.0, 0.0, 0.0)  # (excess, t, ours, envelope low, envelope high)
    spread = (0.0, 0.0)                # (widest reference disagreement, t)
    for t in common:
        band = [table[t] for table in references.values()]
        low, high = min(band), max(band)
        alt = ours[t]
        allowance = max(abs(low), abs(high)) * args.tolerance + args.floor_m
        excess = max(low - alt, alt - high) - allowance
        if excess > worst[0]:
            worst = (excess, t, alt, low, high)
        if high - low > spread[0]:
            spread = (high - low, t)

    print(f"compared {len(common)} samples of {args.truth.name} against "
          f"{len(references)} reference(s): {', '.join(references)}")
    if len(references) > 1:
        print(f"  references disagree with each other by up to {spread[0] / 1000:.2f} km "
              f"(at t = {spread[1]:.1f} s) -- the envelope is that wide")
    for event_time, label in EVENTS:
        t = round(event_time, 4)
        if t not in ours or t not in common:
            continue
        band = [table[t] for table in references.values()]
        print(f"  t={event_time:6.1f} s  {label:<30}  ours {ours[t] / 1000:8.2f} km   "
              f"reference {min(band) / 1000:8.2f}..{max(band) / 1000:.2f} km")

    if worst[0] > 0.0:
        excess, t, alt, low, high = worst
        print(f"FAIL: worst excursion at t={t:.2f} s -- ours {alt / 1000:.2f} km, envelope "
              f"{low / 1000:.2f}..{high / 1000:.2f} km, {excess / 1000:.2f} km beyond the "
              f"{args.tolerance * 100:.1f}% + {args.floor_m:.0f} m allowance")
        print("Check --stg2-ignition and the launch site: the FMU's default ignites stage 2 at separation, "
              "which is not this scenario.")
        return 1

    print(f"OK: every sample inside the reference envelope, within "
          f"{args.tolerance * 100:.1f}% + {args.floor_m:.0f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
