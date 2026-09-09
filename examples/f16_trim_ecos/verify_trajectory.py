# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Check an f16_trim_cosim run against NASA TM-2015-218675 check-case 11.

Two bars, deliberately kept apart, because they have very different resolution.

**Against the NESC references** (--reference, repeatable). These are the
published participant solutions, and on this check-case they disagree with each
other badly: at t = 200 s Atmos_11_sim_02 has drifted +50.8 ft and -0.84 deg of
heading while sim_04 and sim_05 hold altitude to a tenth of a foot and drift
+0.53 deg the *other way*. An open-loop trim flyout is a lightly damped system
integrated for 200 s, so participants' small differences in trim solution,
gravity model and integrator show up directly -- which is exactly what the
check-case is for, and also why a pass here is weak evidence. This script
reports the envelope width alongside the verdict so the number is never read as
tighter than it is.

**Against an Aetherion run** (--aetherion, optional). Aetherion's own standalone
F16SteadyFlight run of the same case, in SI units and radians. Same physics,
same library, different driver -- so this comparison has real resolution and a
correspondingly tight tolerance. It answers "does the co-simulation reproduce
the plant?", which is the question this example is actually responsible for;
the reference check answers "does the plant reproduce the check-case?", which is
Aetherion's.

Neither check looks only at altitude. On straight-and-level flight altitude is
nearly constant, so an altitude-only comparison passes for a vehicle that flew a
circle at the right height. Ground track and heading carry the information.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

# WGS-84 mean radius, adequate for turning a small angular difference into a
# ground distance for reporting purposes.
EARTH_RADIUS_M = 6371008.8
FT_PER_M = 1.0 / 0.3048


class Track:
    """A trajectory sampled on its own time grid: time, lat/lon [deg], alt [m], yaw [deg]."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.time: list[float] = []
        self.lat: list[float] = []
        self.lon: list[float] = []
        self.alt: list[float] = []
        self.yaw: list[float] = []

    def __len__(self) -> int:
        return len(self.time)

    def at(self, t: float) -> tuple[float, float, float, float] | None:
        """Nearest sample to `t`, or None if the track does not reach it.

        Nearest rather than interpolated: every producer here is on a 0.1 s or
        finer grid over the same 200 s window, so the nearest sample is within
        half a step, and interpolating would quietly paper over a track that
        stopped early.
        """
        if not self.time:
            return None
        best = min(range(len(self.time)), key=lambda i: abs(self.time[i] - t))
        if abs(self.time[best] - t) > 0.5:
            return None
        return self.lat[best], self.lon[best], self.alt[best], self.yaw[best]


def _column(header: list[str], *candidates: str) -> str:
    """Locate a column by any of several spellings, ignoring whitespace and case."""
    normalised = {name.strip().lower(): name for name in header}
    for candidate in candidates:
        if candidate in normalised:
            return normalised[candidate]
    raise SystemExit(f"error: none of {candidates} found among columns {sorted(normalised)}")


def read_truth(path: Path) -> Track:
    """Read f16_trim_cosim's own truth log (SI, radians for attitude)."""
    track = Track(path.name)
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"error: {path} is empty")
    header = list(rows[0].keys())
    c_t = _column(header, "time")
    c_lat = _column(header, "f16::out.lat_deg[real]")
    c_lon = _column(header, "f16::out.lon_deg[real]")
    c_alt = _column(header, "f16::out.alt_m[real]")
    c_yaw = _column(header, "f16::out.yaw_rad[real]")
    for row in rows:
        track.time.append(float(row[c_t]))
        track.lat.append(float(row[c_lat]))
        track.lon.append(float(row[c_lon]))
        track.alt.append(float(row[c_alt]))
        track.yaw.append(math.degrees(float(row[c_yaw])))
    return track


def read_reference(path: Path) -> Track:
    """Read a published NESC check-case CSV (feet, degrees)."""
    track = Track(path.name)
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"error: {path} is empty")
    header = list(rows[0].keys())
    c_t = _column(header, "time")
    c_lat = _column(header, "latitude_deg")
    c_lon = _column(header, "longitude_deg")
    c_alt = _column(header, "altitudemsl_ft")
    c_yaw = _column(header, "eulerangle_deg_yaw")
    for row in rows:
        track.time.append(float(row[c_t]))
        track.lat.append(float(row[c_lat]))
        track.lon.append(float(row[c_lon]))
        track.alt.append(float(row[c_alt]) * 0.3048)
        track.yaw.append(float(row[c_yaw]))
    return track


def read_aetherion(path: Path) -> Track:
    """Read an Aetherion standalone run (SI, radians)."""
    track = Track(path.name)
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"error: {path} is empty")
    header = list(rows[0].keys())
    c_t = _column(header, "time")
    c_lat = _column(header, "latitude_rad")
    c_lon = _column(header, "longitude_rad")
    c_alt = _column(header, "altitudemsl_m")
    c_yaw = _column(header, "eulerangle_rad_yaw")
    for row in rows:
        track.time.append(float(row[c_t]))
        track.lat.append(math.degrees(float(row[c_lat])))
        track.lon.append(math.degrees(float(row[c_lon])))
        track.alt.append(float(row[c_alt]))
        track.yaw.append(math.degrees(float(row[c_yaw])))
    return track


def ground_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    """Great-circle distance between two geodetic points, small-angle safe."""
    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)
    d_phi = phi_b - phi_a
    d_lam = math.radians(lon_b - lon_a)
    # haversine: stable for the sub-kilometre separations this script deals in
    h = math.sin(d_phi / 2.0) ** 2 + math.cos(phi_a) * math.cos(phi_b) * math.sin(d_lam / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def compare_to_envelope(truth: Track, references: list[Track], sample_times: list[float]) -> int:
    """Report our track against the min/max envelope of the references. Returns failures."""
    print(f"\nagainst {len(references)} NESC reference(s): " + ", ".join(r.name for r in references))

    # The participant solutions do not all cover the same window --
    # Atmos_11_sim_04 and sim_05 stop at 180 s where sim_02 runs to 200 s. An
    # earlier version of this script dropped any sample time that not every
    # reference reached, which silently left the last 20 s of the flight
    # unchecked while still printing OK. Each sample is therefore compared
    # against whatever references actually reach it, and the coverage is
    # reported so a thinning envelope is visible rather than invisible.
    coverage: dict[int, list[float]] = {}
    for t in sample_times:
        available = [r for r in references if r.at(t) is not None]
        coverage.setdefault(len(available), []).append(t)
    for count in sorted(coverage, reverse=True):
        times = coverage[count]
        span = f"t = {min(times):.0f}..{max(times):.0f} s"
        if count == 0:
            print(f"  NO reference covers {span} -- those samples are unchecked")
        else:
            print(f"  {count} of {len(references)} reference(s) cover {span}")

    # How far the references are from each other. Printed before any verdict:
    # it is the resolution of everything that follows.
    worst_spread_m = 0.0
    worst_spread_t = 0.0
    worst_yaw_spread = 0.0
    for t in sample_times:
        points = [p for p in (r.at(t) for r in references) if p is not None]
        for i, first in enumerate(points):
            for second in points[i + 1 :]:
                d = ground_distance_m(first[0], first[1], second[0], second[1])
                if d > worst_spread_m:
                    worst_spread_m, worst_spread_t = d, t
                worst_yaw_spread = max(worst_yaw_spread, abs(first[3] - second[3]))
    print(f"  the references disagree with each other by up to {worst_spread_m:.0f} m of ground track "
          f"(at t = {worst_spread_t:.0f} s) and {worst_yaw_spread:.2f} deg of heading -- they do not even "
          f"agree on the sign of the heading drift")
    print("  -- an open-loop trim flyout integrated for 200 s separates participants, so this check is a "
          "sanity bound, not a precision test")

    failures = 0
    worst_alt_excess_m = 0.0
    worst_yaw_excess_deg = 0.0
    for t in sample_times:
        ours = truth.at(t)
        points = [p for p in (r.at(t) for r in references) if p is not None]
        if ours is None or not points:
            continue
        # Inside the envelope means: no further from the *nearest* reference
        # than the references are from each other, plus a floor. Anything
        # tighter would fail on a spread this wide for reasons that say nothing
        # about our run.
        nearest = min(ground_distance_m(ours[0], ours[1], p[0], p[1]) for p in points)
        allowed = worst_spread_m + 500.0
        alt_lo = min(p[2] for p in points)
        alt_hi = max(p[2] for p in points)
        alt_slack = (alt_hi - alt_lo) + 30.0
        yaw_lo = min(p[3] for p in points)
        yaw_hi = max(p[3] for p in points)
        yaw_slack = (yaw_hi - yaw_lo) + 0.5
        if nearest > allowed:
            print(f"  FAIL t={t:6.1f} s  ground track {nearest / 1000.0:.3f} km from the nearest reference "
                  f"(allowed {allowed / 1000.0:.3f})")
            failures += 1
        # How far outside the participants' own spread we sit, before the
        # slack is applied. Tracked whether or not it fails, because "passed
        # with 13 m of margin left" and "passed comfortably" are different
        # facts and only one of them is true here.
        worst_alt_excess_m = max(worst_alt_excess_m, ours[2] - alt_hi, alt_lo - ours[2])
        worst_yaw_excess_deg = max(worst_yaw_excess_deg, ours[3] - yaw_hi, yaw_lo - ours[3])
        if not (alt_lo - alt_slack <= ours[2] <= alt_hi + alt_slack):
            print(f"  FAIL t={t:6.1f} s  altitude {ours[2]:.1f} m outside "
                  f"[{alt_lo - alt_slack:.1f}, {alt_hi + alt_slack:.1f}] m")
            failures += 1
        if not (yaw_lo - yaw_slack <= ours[3] <= yaw_hi + yaw_slack):
            print(f"  FAIL t={t:6.1f} s  heading {ours[3]:.3f} deg outside "
                  f"[{yaw_lo - yaw_slack:.3f}, {yaw_hi + yaw_slack:.3f}] deg")
            failures += 1
    if worst_alt_excess_m > 0.0 or worst_yaw_excess_deg > 0.0:
        print(f"  note: our run sits up to {max(0.0, worst_alt_excess_m):.1f} m in altitude and "
              f"{max(0.0, worst_yaw_excess_deg):.2f} deg in heading *beyond* the participants' own spread, "
              f"drifting further than any of them in the same direction as Atmos_11_sim_02")
    return failures


def compare_to_aetherion(truth: Track, aetherion: Track, sample_times: list[float],
                         position_tol_m: float, yaw_tol_deg: float, alt_tol_m: float) -> int:
    """Report our track against an Aetherion standalone run. Returns failures."""
    print(f"\nagainst Aetherion's own run of the same case: {aetherion.name}")
    failures = 0
    worst_pos = worst_alt = worst_yaw = 0.0
    worst_pos_t = worst_alt_t = worst_yaw_t = 0.0
    for t in sample_times:
        ours = truth.at(t)
        theirs = aetherion.at(t)
        if ours is None or theirs is None:
            continue
        d_pos = ground_distance_m(ours[0], ours[1], theirs[0], theirs[1])
        d_alt = abs(ours[2] - theirs[2])
        d_yaw = abs(ours[3] - theirs[3])
        if d_pos > worst_pos:
            worst_pos, worst_pos_t = d_pos, t
        if d_alt > worst_alt:
            worst_alt, worst_alt_t = d_alt, t
        if d_yaw > worst_yaw:
            worst_yaw, worst_yaw_t = d_yaw, t
    print(f"  worst ground-track difference {worst_pos:.2f} m (t = {worst_pos_t:.0f} s)")
    print(f"  worst altitude difference     {worst_alt:.2f} m (t = {worst_alt_t:.0f} s)")
    print(f"  worst heading difference      {worst_yaw:.2e} deg (t = {worst_yaw_t:.0f} s)")
    if worst_pos > position_tol_m:
        print(f"  FAIL ground track: {worst_pos:.2f} m > {position_tol_m:.2f} m")
        failures += 1
    if worst_alt > alt_tol_m:
        print(f"  FAIL altitude: {worst_alt:.2f} m > {alt_tol_m:.2f} m")
        failures += 1
    if worst_yaw > yaw_tol_deg:
        print(f"  FAIL heading: {worst_yaw:.3e} deg > {yaw_tol_deg:.3e} deg")
        failures += 1
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth", type=Path, default=Path("results/f16_truth.csv"),
                        help="f16_trim_cosim's truth log")
    parser.add_argument("--reference", type=Path, action="append", default=None,
                        help="a published NESC Atmos_11 CSV; repeat for each participant solution")
    parser.add_argument("--aetherion", type=Path, default=None,
                        help="an Aetherion standalone F16SteadyFlight CSV (SI, radians)")
    parser.add_argument("--position-tol-m", type=float, default=50.0,
                        help="ground-track tolerance against --aetherion (default 50 m over 34 km flown)")
    parser.add_argument("--alt-tol-m", type=float, default=5.0,
                        help="altitude tolerance against --aetherion (default 5 m; the phugoid amplitude is ~7 m, "
                             "so a small phase difference between two drivers of the same plant lands here)")
    parser.add_argument("--yaw-tol-deg", type=float, default=0.01,
                        help="heading tolerance against --aetherion (default 0.01 deg)")
    args = parser.parse_args()

    if not args.reference and not args.aetherion:
        parser.error("give at least one of --reference or --aetherion")

    truth = read_truth(args.truth)
    print(f"read {len(truth)} samples of {truth.name}")

    # Sample once per 10 s rather than at every communication point: the tracks
    # are smooth, and 21 comparisons that can each be printed beat 2001 that
    # cannot.
    end = min(200.0, truth.time[-1])
    sample_times = [round(0.1 * i, 1) for i in range(0, int(end * 10) + 1, 100)]

    failures = 0
    if args.reference:
        references = [read_reference(p) for p in args.reference]
        failures += compare_to_envelope(truth, references, sample_times)
    if args.aetherion:
        failures += compare_to_aetherion(truth, read_aetherion(args.aetherion), sample_times,
                                         args.position_tol_m, args.yaw_tol_deg, args.alt_tol_m)

    print()
    if failures:
        print(f"FAILED: {failures} check(s)")
        return 1
    print("OK: every check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
