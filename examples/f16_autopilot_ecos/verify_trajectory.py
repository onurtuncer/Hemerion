# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Check an f16_autopilot_cosim run against NASA TM-2015-218675 check-cases 13.1-13.4.

Two kinds of check, and unlike the trim-flyout verifier the second one is new:

**Against the NESC references** (--reference, repeatable): the published
participant solutions, compared on ground track, altitude and heading as an
envelope with the participants' own disagreement as the slack -- a sanity
bound, exactly as in examples/f16_trim_ecos.

**Against the command** (--case, required): the closed loop's actual job. The
references alone cannot carry this check -- their altitude spread plus a
sensible floor is wider than the 100 ft step that case 13.1 exists to
demonstrate, so a run that never climbed could sit inside the envelope. Each
case therefore asserts its own commanded response, with bounds taken from
what the participants themselves achieve:

  13.1  altitude settles at 10 113 ft (sim_02/04/05 end within 0.6 ft of it)
  13.2  KEAS ends between 275 and 285.5 kt (sim_02 reaches 277.0 by 20 s;
        sim_04/05 are still at ~283 kt at their windows' ends -- the
        deceleration authority separates participants, so "reached 277
        exactly" would fail two of the three published solutions)
  13.3  course settles at 60 deg (participants end 59.92-60.01)
  13.4  lateral deviation reaches 2000 ft right of the courseline
        (participants show 1934-1992 ft at t = 60 s), judged at t = 60 s:
        past that the flat-earth deviation formula itself drifts (sim_05
        reads 1817 ft at 239.9 s), and judging a run by a formula outside
        its validity would be measuring the formula

The hold quantities are asserted too: a heading case must hold altitude, an
altitude case must hold heading. That is what makes 13.x a *fusion-grade*
reference: several quantities are pinned at once.

KEAS is computed from the truth log's vt and rho with the standalone's own
constants (rho_SL = 1.225 kg/m^3, kt = 0.5144444 m/s), so the figure judged
here is the figure the controller was commanded in.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

EARTH_RADIUS_M = 6371008.8
FT_PER_M = 1.0 / 0.3048
KT_MPS = 0.5144444
RHO_SL_KG_M3 = 1.225
# The standalone's flat-earth radius for lateral deviation -- deliberately its
# value, not this script's EARTH_RADIUS_M, so the deviation judged here is the
# deviation the controller was fed.
LATDEV_RADIUS_M = 6371000.0


class Track:
    """A trajectory on its own time grid: lat/lon [deg], alt [m], yaw [deg], keas [kt]."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.time: list[float] = []
        self.lat: list[float] = []
        self.lon: list[float] = []
        self.alt: list[float] = []
        self.yaw: list[float] = []
        self.keas: list[float] = []  # empty when the source carries no density

    def __len__(self) -> int:
        return len(self.time)

    def index_at(self, t: float) -> int | None:
        if not self.time:
            return None
        best = min(range(len(self.time)), key=lambda i: abs(self.time[i] - t))
        if abs(self.time[best] - t) > 0.5:
            return None
        return best

    def at(self, t: float) -> tuple[float, float, float, float] | None:
        """Nearest (lat, lon, alt, yaw) to `t`, or None if the track stops short."""
        best = self.index_at(t)
        if best is None:
            return None
        return self.lat[best], self.lon[best], self.alt[best], self.yaw[best]

    def lateral_deviation_ft(self, t: float) -> float | None:
        """Deviation from the initial courseline at 45 deg, +right, the standalone's formula."""
        best = self.index_at(t)
        if best is None:
            return None
        lat0, lon0 = math.radians(self.lat[0]), math.radians(self.lon[0])
        dp_n = (math.radians(self.lat[best]) - lat0) * LATDEV_RADIUS_M
        dp_e = (math.radians(self.lon[best]) - lon0) * LATDEV_RADIUS_M * math.cos(lat0)
        psi0 = math.radians(45.0)
        return (-dp_n * math.sin(psi0) + dp_e * math.cos(psi0)) * FT_PER_M


def _column(header: list[str], *candidates: str) -> str:
    normalised = {name.strip().lower(): name for name in header}
    for candidate in candidates:
        if candidate in normalised:
            return normalised[candidate]
    raise SystemExit(f"error: none of {candidates} found among columns {sorted(normalised)}")


def read_truth(path: Path) -> Track:
    """Read f16_autopilot_cosim's truth log (SI, radians for attitude)."""
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
    c_vt = _column(header, "f16::out.vt_m_s[real]")
    c_rho = _column(header, "f16::out.rho_kg_m3[real]")
    for row in rows:
        track.time.append(float(row[c_t]))
        track.lat.append(float(row[c_lat]))
        track.lon.append(float(row[c_lon]))
        track.alt.append(float(row[c_alt]))
        track.yaw.append(math.degrees(float(row[c_yaw])))
        track.keas.append((float(row[c_vt]) / KT_MPS) * math.sqrt(float(row[c_rho]) / RHO_SL_KG_M3))
    return track


def read_reference(path: Path) -> Track:
    """Read a published NESC Atmos_13p* CSV (feet, degrees, slug/ft^3)."""
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
    c_vn = _column(header, "fevelocity_ft_s_x")
    c_ve = _column(header, "fevelocity_ft_s_y")
    c_vd = _column(header, "fevelocity_ft_s_z")
    c_rho = _column(header, "airdensity_slug_ft3")
    for row in rows:
        track.time.append(float(row[c_t]))
        track.lat.append(float(row[c_lat]))
        track.lon.append(float(row[c_lon]))
        track.alt.append(float(row[c_alt]) * 0.3048)
        track.yaw.append(float(row[c_yaw]))
        v_mps = (math.hypot(float(row[c_vn]), float(row[c_ve]), float(row[c_vd]))) * 0.3048
        rho = float(row[c_rho]) * 515.378818  # slug/ft^3 -> kg/m^3
        track.keas.append((v_mps / KT_MPS) * math.sqrt(rho / RHO_SL_KG_M3))
    return track


def ground_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)
    d_phi = phi_b - phi_a
    d_lam = math.radians(lon_b - lon_a)
    h = math.sin(d_phi / 2.0) ** 2 + math.cos(phi_a) * math.cos(phi_b) * math.sin(d_lam / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def compare_to_envelope(truth: Track, references: list[Track], sample_times: list[float]) -> int:
    """Our track against the min/max envelope of the references. Returns failures."""
    print(f"\nagainst {len(references)} NESC reference(s): " + ", ".join(r.name for r in references))

    # Participant windows are unequal (on these cases sim_02/sim_04 stop at
    # 20-60 s while sim_05 runs to 60-239.9 s), so each sample is compared
    # against whatever references reach it and the coverage is printed --
    # same reasoning as the trim verifier.
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

    worst_spread_m = 0.0
    worst_spread_t = 0.0
    worst_alt_spread_m = 0.0
    worst_yaw_spread = 0.0
    for t in sample_times:
        points = [point for r in references if (point := r.at(t)) is not None]
        for i, first in enumerate(points):
            for second in points[i + 1 :]:
                d = ground_distance_m(first[0], first[1], second[0], second[1])
                if d > worst_spread_m:
                    worst_spread_m, worst_spread_t = d, t
                worst_alt_spread_m = max(worst_alt_spread_m, abs(first[2] - second[2]))
                worst_yaw_spread = max(worst_yaw_spread, abs(first[3] - second[3]))
    print(f"  the references disagree with each other by up to {worst_spread_m:.0f} m of ground track "
          f"(at t = {worst_spread_t:.0f} s), {worst_alt_spread_m:.1f} m of altitude and "
          f"{worst_yaw_spread:.2f} deg of heading")
    print("  -- closed-loop responses separate participants less than open-loop flyouts, but the spread is "
          "still the resolution of this check; the commanded response is asserted separately below")

    # The participants' worst disagreement anywhere in the window, plus a
    # floor, applied at every sample -- one band for the whole run, as in the
    # trim verifier and for its reason (how long a participant integrated is
    # not a property of the run being checked).
    allowed_m = worst_spread_m + 500.0
    alt_slack_m = worst_alt_spread_m + 30.0
    yaw_slack_deg = worst_yaw_spread + 1.0

    failures = 0
    for t in sample_times:
        ours = truth.at(t)
        named = [(r.name, point) for r in references if (point := r.at(t)) is not None]
        if ours is None or not named:
            continue
        nearest = min(ground_distance_m(ours[0], ours[1], point[0], point[1]) for _, point in named)
        highest = max(named, key=lambda item: item[1][2])
        lowest = min(named, key=lambda item: item[1][2])
        largest_yaw = max(named, key=lambda item: item[1][3])
        smallest_yaw = min(named, key=lambda item: item[1][3])
        if nearest > allowed_m:
            print(f"  FAIL t={t:6.1f} s  ground track {nearest / 1000.0:.3f} km from the nearest reference "
                  f"(allowed {allowed_m / 1000.0:.3f})")
            failures += 1
        if not (lowest[1][2] - alt_slack_m <= ours[2] <= highest[1][2] + alt_slack_m):
            print(f"  FAIL t={t:6.1f} s  altitude {ours[2]:.1f} m outside "
                  f"[{lowest[1][2] - alt_slack_m:.1f}, {highest[1][2] + alt_slack_m:.1f}] m")
            failures += 1
        if not (smallest_yaw[1][3] - yaw_slack_deg <= ours[3] <= largest_yaw[1][3] + yaw_slack_deg):
            print(f"  FAIL t={t:6.1f} s  heading {ours[3]:.3f} deg outside "
                  f"[{smallest_yaw[1][3] - yaw_slack_deg:.3f}, {largest_yaw[1][3] + yaw_slack_deg:.3f}] deg")
            failures += 1
    return failures


def check_response(truth: Track, case: str) -> int:
    """Assert the commanded response actually happened. Returns failures."""
    print(f"\ncommanded response, case {case}:")
    failures = 0
    end = truth.time[-1]

    def judge(label: str, measured: float | None, low: float, high: float) -> None:
        nonlocal failures
        if measured is None:
            print(f"  FAIL {label}: no sample to judge")
            failures += 1
            return
        verdict = "ok  " if low <= measured <= high else "FAIL"
        if verdict == "FAIL":
            failures += 1
        print(f"  {verdict} {label}: {measured:.2f} (allowed {low:.2f}..{high:.2f})")

    final = truth.at(end)
    if final is None:
        print("  FAIL: empty truth track")
        return 1
    final_alt_ft = final[2] * FT_PER_M
    final_yaw = final[3]
    final_keas = truth.keas[truth.index_at(end)] if truth.keas else None

    if case == "13.1":
        # sim_02/04/05 end within 0.6 ft of the 10 113 ft command; +-10 ft
        # allows the co-simulation's delay-induced offset (measured +0.5 ft)
        # with an order of magnitude to spare below the step size.
        judge("final altitude [ft]", final_alt_ft, 10113.0 - 10.0, 10113.0 + 10.0)
        judge("held heading [deg]", final_yaw, 44.0, 46.0)
        judge("held KEAS [kt]", final_keas, 286.0, 290.0)
    elif case == "13.2":
        # sim_02 reaches 277.0 kt inside its 20 s window; sim_04/05 are still
        # at ~283 kt when theirs end. The band accepts anything from "settled
        # on the command" to "decelerating like the slowest participant" --
        # but not the trim KEAS of ~288, so a loop that ignored the command
        # fails.
        judge("final KEAS [kt]", final_keas, 275.0, 285.5)
        judge("held altitude [ft]", final_alt_ft, 10013.0 - 15.0, 10013.0 + 15.0)
        judge("held heading [deg]", final_yaw, 44.0, 46.0)
    elif case == "13.3":
        # Participants end 59.92-60.01 deg.
        judge("final course [deg]", final_yaw, 60.0 - 1.5, 60.0 + 1.5)
        judge("held altitude [ft]", final_alt_ft, 10013.0 - 15.0, 10013.0 + 15.0)
        judge("held KEAS [kt]", final_keas, 286.0, 290.0)
    elif case == "13.4":
        # Judged at t = 60 s, where the participants read 1934-1992 ft: past
        # that the flat-earth deviation formula itself drifts (sim_05 reads
        # 1817 ft at 239.9 s) and a longer run would be judged on the
        # formula's error, not the aircraft's.
        judge_time = min(60.0, end)
        latdev = truth.lateral_deviation_ft(judge_time)
        judge(f"lateral deviation at t={judge_time:.0f} s [ft]", latdev, 2000.0 - 150.0, 2000.0 + 150.0)
        judge("held course [deg]", final_yaw, 44.0, 46.0)
        judge("held altitude [ft]", final_alt_ft, 10013.0 - 15.0, 10013.0 + 15.0)
    else:
        print(f"  FAIL: unknown case {case}")
        return 1
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth", type=Path, default=Path("results/f16_truth.csv"),
                        help="f16_autopilot_cosim's truth log")
    parser.add_argument("--case", required=True, choices=["13.1", "13.2", "13.3", "13.4"],
                        help="which check-case the truth log flew -- selects the commanded-response assertions")
    parser.add_argument("--reference", type=Path, action="append", default=None,
                        help="a published NESC Atmos_13p* CSV; repeat for each participant solution")
    args = parser.parse_args()

    truth = read_truth(args.truth)
    print(f"read {len(truth)} samples of {truth.name}")

    # Once per 5 s: the 13.1/13.2 windows are only 60 s long, and the
    # transients this example exists for live in their first thirty.
    end = truth.time[-1]
    sample_times = [round(5.0 * i, 1) for i in range(0, int(end / 5.0) + 1)]

    failures = check_response(truth, args.case)
    if args.reference:
        references = [read_reference(p) for p in args.reference]
        failures += compare_to_envelope(truth, references, sample_times)

    print()
    if failures:
        print(f"FAILED: {failures} check(s)")
        return 1
    print("OK: every check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
