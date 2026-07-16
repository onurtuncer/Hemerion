# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Plots the rocket_gps_ecos co-simulation results for the Sphinx docs.

Reads the three CSVs the example produces (Ecos' rocket-truth log and the
flight computer's decoded-fix and decoded-IMU-sample logs), joins them on
simulation time, and renders the figures embedded in
doc/rocket_gps_ecos_cosim.rst:

    altitude_vs_time.png     truth altitude + decoded GPS fixes + staging/apogee
    ground_track.png         truth ground track + decoded GPS fixes
    velocity_vs_time.png     truth speed over ground + GPS-reported speed
    gps_error.png            horizontal/vertical decoded-fix error vs truth
    imu_specific_force.png   truth specific force + decoded accelerometer samples
    imu_body_rates.png       truth body rates + decoded gyroscope samples

Usage (from the co-simulation working directory, after a run):

    python plot_results.py [--truth results/rocket_truth.csv]
                           [--fixes results/gps_fixes.csv]
                           [--imu results/imu_samples.csv] [--out plots/]

Only matplotlib is required. The truth CSV is Ecos csv_writer output
(", "-separated, "name[TYPE]" headers); the fix and IMU CSVs are written by
gps_flight_computer.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Colors follow the repo docs' data-viz palette: categorical slot 1 (blue) for
# truth, slot 2 (aqua) for the flight computer's decoded fixes, neutral inks
# for text and annotations.
TRUTH = "#2a78d6"
GPS = "#1baf7a"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d9d8d4"
SURFACE = "#fcfcfb"

METERS_PER_DEG_LAT = 111_320.0


def read_truth(path: Path) -> dict[str, list[float]]:
    """Reads Ecos csv_writer output into {short_name: column}."""
    with path.open(newline="") as f:
        rows = list(csv.reader(f))
    header = [h.strip() for h in rows[0]]
    # "rocket::out.alt_m[REAL]" -> "alt_m"
    names = [h.split("::")[-1].removeprefix("out.").split("[")[0] for h in header]
    data: dict[str, list[float]] = {n: [] for n in names}
    for row in rows[1:]:
        if len(row) != len(names):
            continue
        for name, cell in zip(names, row):
            data[name].append(float(cell))
    return data


def read_fixes(path: Path) -> dict[str, list[float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return {k: [float(r[k]) for r in rows] for k in reader.fieldnames}


def style_axis(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.grid(True, color=GRID, linewidth=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)
    ax.title.set_color(INK)


def new_figure():
    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style_axis(ax)
    return fig, ax


def legend(ax) -> None:
    leg = ax.legend(loc="best", frameon=False, fontsize=9)
    for text in leg.get_texts():
        text.set_color(INK_2)


def staging_time(truth: dict[str, list[float]]) -> float | None:
    for t, staged in zip(truth["time"], truth["staged"]):
        if staged > 0.5:
            return t
    return None


def annotate_event(ax, x: float, label: str) -> None:
    ax.axvline(x, color=INK_2, linewidth=1.0, linestyle="--", alpha=0.7)
    ax.annotate(label, xy=(x, 1.0), xycoords=("data", "axes fraction"),
                xytext=(4, -4), textcoords="offset points",
                ha="left", va="top", fontsize=8, color=INK_2)


def plot_altitude(truth, fixes, out: Path) -> None:
    fig, ax = new_figure()
    # GPS dots first, truth line on top -- the two overlap almost everywhere,
    # and the line must stay visible.
    ax.plot(fixes["sim_time_s"], [a / 1000.0 for a in fixes["altitude_m"]],
            linestyle="none", marker=".", markersize=4, alpha=0.45, color=GPS,
            label="GPS fixes decoded by flight computer")
    ax.plot(truth["time"], [a / 1000.0 for a in truth["alt_m"]],
            color=TRUTH, linewidth=1.6, label="rocket truth (TwoStageRocket.fmu)")
    t_stage = staging_time(truth)
    if t_stage is not None:
        annotate_event(ax, t_stage, f"stage 1 separation\n t = {t_stage:.1f} s")
    apogee = max(truth["alt_m"])
    t_apogee = truth["time"][truth["alt_m"].index(apogee)]
    ax.annotate(f"apogee {apogee / 1000.0:.1f} km",
                xy=(t_apogee, apogee / 1000.0), xytext=(-8, 8),
                textcoords="offset points", ha="right", fontsize=8, color=INK_2)
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("altitude above MSL [km]")
    ax.set_title("Two-stage rocket ascent: truth vs. decoded GPS altitude")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "altitude_vs_time.png", facecolor=SURFACE)
    plt.close(fig)


def plot_ground_track(truth, fixes, out: Path) -> None:
    fig, ax = new_figure()
    lat = [math.degrees(v) for v in truth["lat_rad"]]
    lon = [math.degrees(v) for v in truth["lon_rad"]]
    ax.plot(fixes["longitude_deg"], fixes["latitude_deg"],
            linestyle="none", marker=".", markersize=4, alpha=0.45, color=GPS,
            label="GPS fixes")
    ax.plot(lon, lat, color=TRUTH, linewidth=1.6, label="rocket truth")
    ax.plot(lon[0], lat[0], marker="^", markersize=9, color=TRUTH)
    ax.annotate("launch (Wallops)", xy=(lon[0], lat[0]), xytext=(8, 4),
                textcoords="offset points", fontsize=8, color=INK_2)
    ax.set_xlabel("longitude [deg]")
    ax.set_ylabel("latitude [deg]")
    ax.set_title("Ground track, firing due east over the Atlantic")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "ground_track.png", facecolor=SURFACE)
    plt.close(fig)


def plot_velocity(truth, fixes, out: Path) -> None:
    fig, ax = new_figure()
    speed = [math.hypot(n, e) for n, e in zip(truth["v_north_m_s"], truth["v_east_m_s"])]
    ax.plot(fixes["sim_time_s"], fixes["ground_speed_mps"],
            linestyle="none", marker=".", markersize=4, alpha=0.45, color=GPS,
            label="GPS-reported speed over ground")
    ax.plot(truth["time"], speed, color=TRUTH, linewidth=1.6,
            label="truth speed over ground")
    t_stage = staging_time(truth)
    if t_stage is not None:
        annotate_event(ax, t_stage, "staging")
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("speed over ground [m/s]")
    ax.set_title("Speed over ground: truth vs. UBX-NAV-PVT gSpeed")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "velocity_vs_time.png", facecolor=SURFACE)
    plt.close(fig)


def plot_gps_error(truth, fixes, out: Path) -> None:
    # Truth rows and fixes share the 10 Hz grid (one NAV-PVT per step), so
    # align by rounded time rather than interpolating. The fix emitted at the
    # end of step k carries the truth sampled at the end of step k-1 -- the
    # Ecos master propagates connections between steps, one communication
    # step of transport delay -- so each fix is compared against the truth
    # one step earlier. Comparing at equal timestamps instead would show
    # speed x 0.1 s (hundreds of metres at rocket velocities), which is
    # latency, not receiver error.
    step = fixes["sim_time_s"][1] - fixes["sim_time_s"][0]
    truth_by_time = {round(t, 3): i for i, t in enumerate(truth["time"])}
    times, horizontal, vertical = [], [], []
    for i, t in enumerate(fixes["sim_time_s"]):
        j = truth_by_time.get(round(t - step, 3))
        if j is None:
            continue
        lat_t = math.degrees(truth["lat_rad"][j])
        lon_t = math.degrees(truth["lon_rad"][j])
        dlat_m = (fixes["latitude_deg"][i] - lat_t) * METERS_PER_DEG_LAT
        dlon_m = (fixes["longitude_deg"][i] - lon_t) * METERS_PER_DEG_LAT * math.cos(math.radians(lat_t))
        times.append(t)
        horizontal.append(math.hypot(dlat_m, dlon_m))
        vertical.append(abs(fixes["altitude_m"][i] - truth["alt_m"][j]))

    fig, ax = new_figure()
    ax.plot(times, horizontal, color=TRUTH, linewidth=0.9, alpha=0.9,
            label="horizontal error")
    ax.plot(times, vertical, color=GPS, linewidth=0.9, alpha=0.9,
            label="vertical error")
    ax.axhline(1.5, color=INK_2, linewidth=1.0, linestyle="--", alpha=0.7)
    ax.annotate("receiver-reported 1-sigma horizontal accuracy (1.5 m)",
                xy=(0.99, 1.5), xycoords=("axes fraction", "data"),
                xytext=(0, 4), textcoords="offset points",
                ha="right", fontsize=8, color=INK_2)
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("decoded fix error vs. truth [m]")
    ax.set_title("GPS error injected by GpsNoiseModel, recovered end-to-end")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "gps_error.png", facecolor=SURFACE)
    plt.close(fig)


def plot_imu_specific_force(truth, imu, out: Path) -> None:
    # The truth CSV's f_x/f_y/f_z columns are the imu:: input variables --
    # exactly the zero-order-held value the IMU FMU sampled (already one
    # communication step behind rocket truth, like every connection), so no
    # realignment is needed here.
    fig, ax = new_figure()
    ax.plot(imu["sim_time_s"], imu["accel_x_mps2"],
            linestyle="none", marker=".", markersize=2, alpha=0.25, color=GPS,
            label="accelerometer X samples decoded by flight computer")
    ax.plot(truth["time"], truth["f_x_mps2"], color=TRUTH, linewidth=1.4,
            label="true specific force X (thrust + aero) / mass")
    t_stage = staging_time(truth)
    if t_stage is not None:
        annotate_event(ax, t_stage, f"stage 1 separation\n t = {t_stage:.1f} s")
    ax.annotate("burnout: free fall reads 0 m/s²",
                xy=(0.98, 0.35), xycoords="axes fraction",
                ha="right", fontsize=8, color=INK_2)
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("body-X specific force [m/s²]")
    ax.set_title("Boost-phase acceleration: truth vs. decoded IMU counts")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "imu_specific_force.png", facecolor=SURFACE)
    plt.close(fig)


def plot_imu_body_rates(truth, imu, out: Path) -> None:
    fig, ax = new_figure()
    for axis, color, marker_alpha in (("x", TRUTH, 0.2), ("y", GPS, 0.2), ("z", INK_2, 0.2)):
        ax.plot(imu["sim_time_s"], imu[f"gyro_{axis}_rad_s"],
                linestyle="none", marker=".", markersize=2, alpha=marker_alpha, color=color,
                label=f"gyro {axis.upper()} samples")
    for name, axis, color in (("p_rad_s", "p (roll)", TRUTH), ("q_rad_s", "q (pitch)", GPS),
                              ("r_rad_s", "r (yaw)", INK_2)):
        ax.plot(truth["time"], truth[name], color=color, linewidth=1.2, label=f"truth {axis}")
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("body angular rate [rad/s]")
    ax.set_title("Body rates: truth p/q/r vs. decoded gyroscope samples")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "imu_body_rates.png", facecolor=SURFACE)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--truth", type=Path, default=Path("results/rocket_truth.csv"))
    parser.add_argument("--fixes", type=Path, default=Path("results/gps_fixes.csv"))
    parser.add_argument("--imu", type=Path, default=Path("results/imu_samples.csv"))
    parser.add_argument("--out", type=Path, default=Path("plots"))
    args = parser.parse_args()

    truth = read_truth(args.truth)
    fixes = read_fixes(args.fixes)
    args.out.mkdir(parents=True, exist_ok=True)

    plot_altitude(truth, fixes, args.out)
    plot_ground_track(truth, fixes, args.out)
    plot_velocity(truth, fixes, args.out)
    plot_gps_error(truth, fixes, args.out)

    figures = 4
    if args.imu.exists():
        imu = read_fixes(args.imu)  # same plain-CSV shape as the fix log
        plot_imu_specific_force(truth, imu, args.out)
        plot_imu_body_rates(truth, imu, args.out)
        figures += 2
    print(f"wrote {figures} figures to {args.out}/")


if __name__ == "__main__":
    main()
