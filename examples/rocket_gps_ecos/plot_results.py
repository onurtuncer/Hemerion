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

    altitude_vs_time.png     truth altitude + decoded GPS fixes + staging marker
    ground_track.png         truth ground track + decoded GPS fixes
    velocity_vs_time.png     truth speed over ground + GPS-reported speed
    gps_error.png            horizontal/vertical decoded-fix error vs truth
    gps_availability.png     why the receiver dropped out: truth vs. its envelope
    imu_specific_force.png   truth specific force + decoded accelerometer samples
    imu_body_rates.png       truth body rates + decoded gyroscope samples

Usage (from the co-simulation working directory, after a run):

    python plot_results.py [--truth results/rocket_truth.csv]
                           [--fixes results/gps_fixes.csv]
                           [--imu results/imu_samples.csv] [--out plots/]

With the receiver's dynamics envelope in force a launch vehicle can produce
*no* usable fixes at all, so the four fix-dependent figures are skipped rather
than drawn empty; gps_availability.png is the one that still says something,
and the IMU figures are unaffected. Run the co-simulation a second time with
``--dyn-model -1 --no-cocom`` to get a fix stream to compare against.

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
# Neutral wash for the epochs the receiver reported no solution for -- a
# region of absent data, so it must read as background, not as a series.
OUTAGE = "#eceae5"
# Threshold lines are annotation, not data: warm enough to read against the
# categorical blues without competing with them for attention.
LIMIT = "#b4632a"

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
    """Reads a flight-computer log, keeping only epochs that carry a fix.

    The GPS FMU's dynamics envelope (see doc/sensor_models.rst) drops the fix
    whenever the vehicle leaves the receiver's platform-model limits or the
    COCOM thresholds, so the log legitimately contains no-fix epochs whose
    position fields mean nothing. Plotting them as data would invent a
    trajectory the receiver never reported. The IMU log has no fix_type
    column and passes through unfiltered.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    fields = reader.fieldnames
    if "fix_type" in fields:
        rows = [r for r in rows if float(r["fix_type"]) > 0.0]
    return {k: [float(r[k]) for r in rows] for k in fields}


def read_outages(path: Path) -> list[tuple[float, float]]:
    """Finds the no-fix windows the receiver's dynamics envelope produced.

    Returned as (start, end) pairs in simulation time, merged across
    consecutive epochs. Every epoch is logged whether or not it carried a
    solution -- the receiver keeps emitting NAV-PVT throughout, it just stops
    setting gnssFixOK -- so the gaps are directly readable from fix_type.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if "fix_type" not in (reader.fieldnames or []):
            return []
        rows = list(reader)

    intervals: list[tuple[float, float]] = []
    start: float | None = None
    previous = 0.0
    step = 0.1
    times = [float(r["sim_time_s"]) for r in rows]
    if len(times) > 1:
        step = min(b - a for a, b in zip(times, times[1:]))
    for row, t in zip(rows, times):
        if float(row["fix_type"]) <= 0.0:
            if start is None:
                start = t
            previous = t
        elif start is not None:
            intervals.append((start, previous + step))
            start = None
    if start is not None:
        intervals.append((start, previous + step))
    return intervals


def shade_outages(ax, intervals: list[tuple[float, float]], label: bool = True) -> None:
    """Marks the no-fix windows behind a time-axis plot.

    `label` is off for the second panel of a shared-axis figure, where one
    legend entry already covers both.
    """
    for index, (start, end) in enumerate(intervals):
        ax.axvspan(start, end, color=OUTAGE, alpha=0.5, linewidth=0,
                   zorder=0, label="no fix (dynamics envelope)" if (label and index == 0) else None)


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


def plot_altitude(truth, fixes, outages, out: Path) -> None:
    fig, ax = new_figure()
    shade_outages(ax, outages)
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
    # Scenario 17 is a rocket *to orbit*: inside the 200 s reference window the
    # vehicle is still climbing at cut-off, so this is the highest altitude
    # reached and not an apogee. Labelled accordingly rather than asserting a
    # turning point the data does not contain.
    peak = max(truth["alt_m"])
    t_peak = truth["time"][truth["alt_m"].index(peak)]
    ax.annotate(f"{peak / 1000.0:.1f} km at cut-off",
                xy=(t_peak, peak / 1000.0), xytext=(-8, 8),
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
    ax.annotate("launch", xy=(lon[0], lat[0]), xytext=(8, 4),
                textcoords="offset points", fontsize=8, color=INK_2)
    ax.set_xlabel("longitude [deg]")
    ax.set_ylabel("latitude [deg]")
    ax.set_title("Ground track, firing due east over the Atlantic")
    legend(ax)
    fig.tight_layout()
    fig.savefig(out / "ground_track.png", facecolor=SURFACE)
    plt.close(fig)


def plot_velocity(truth, fixes, outages, out: Path) -> None:
    fig, ax = new_figure()
    shade_outages(ax, outages)
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


def plot_gps_availability(truth, outages, out: Path) -> None:
    """Shows the truth state against the limits that took the fix away.

    Two panels sharing a time axis, because the COCOM cut-off is an AND of
    altitude and speed and neither alone explains the dropout. The platform
    model's own limits are drawn alongside: for a launch vehicle they bite
    first, and they bite before the vehicle has left the pad by much.
    """
    fig, (ax_alt, ax_vel) = plt.subplots(2, 1, figsize=(8.0, 5.4), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, label in ((ax_alt, True), (ax_vel, False)):
        style_axis(ax)
        shade_outages(ax, outages, label)

    def threshold(ax, value, label, style):
        ax.axhline(value, color=LIMIT, linewidth=1.1, linestyle=style)
        ax.annotate(label, xy=(0.995, value), xycoords=("axes fraction", "data"),
                    xytext=(0, 3), textcoords="offset points", ha="right", fontsize=8, color=LIMIT)

    ax_alt.plot(truth["time"], [a / 1000.0 for a in truth["alt_m"]],
                color=TRUTH, linewidth=1.6, label="truth altitude")
    threshold(ax_alt, 18.0, "COCOM 18 km", "--")
    threshold(ax_alt, 80.0, "airborne <4 g platform max, 80 km", ":")
    # Log above the thresholds, linear through them: the interesting crossings
    # are three decades below apogee. The negative half of a symlog axis is
    # dead space here, so it is clipped away.
    ax_alt.set_yscale("symlog", linthresh=10.0)
    ax_alt.set_ylim(bottom=0.0)
    ax_alt.set_ylabel("altitude [km]")
    ax_alt.set_title("Why the receiver goes dark: truth against its own envelope")
    legend(ax_alt)

    speed_3d = [math.sqrt(n * n + e * e + d * d)
                for n, e, d in zip(truth["v_north_m_s"], truth["v_east_m_s"], truth["v_down_m_s"])]
    ax_vel.plot(truth["time"], speed_3d, color=TRUTH, linewidth=1.6, label="truth speed (3-D)")
    # The platform model's own 500 m/s horizontal cap sits 3% below the COCOM
    # threshold; drawing both would be two lines saying the same thing at this
    # scale, so only the export limit is marked.
    threshold(ax_vel, 515.0, "COCOM 515 m/s (platform cap 500 m/s just below)", "--")
    ax_vel.set_yscale("symlog", linthresh=100.0)
    ax_vel.set_ylim(bottom=0.0)
    ax_vel.set_xlabel("simulation time [s]")
    ax_vel.set_ylabel("speed [m/s]")
    legend(ax_vel)

    # Neither panel explains the *first* loss: the platform model's 4 g
    # acceleration limit trips on the second navigation epoch, long before
    # either threshold above is crossed. Say so rather than leaving the reader
    # to infer a cause the figure does not show.
    if outages:
        ax_alt.annotate(f"fix lost at t = {outages[0][0]:.1f} s\n(4 g platform acceleration limit)",
                        xy=(outages[0][0], 0.0), xycoords="data",
                        xytext=(46, 14), textcoords="offset points", fontsize=8, color=INK_2,
                        arrowprops=dict(arrowstyle="->", color=INK_2, linewidth=0.9))

    fig.tight_layout()
    fig.savefig(out / "gps_availability.png", facecolor=SURFACE)
    plt.close(fig)


def plot_gps_error(truth, fixes, step: float, out: Path) -> None:
    # Truth rows and fixes share the 10 Hz grid (one NAV-PVT per step), so
    # align by rounded time rather than interpolating. The fix emitted at the
    # end of step k carries the truth sampled at the end of step k-1 -- the
    # Ecos master propagates connections between steps, one communication
    # step of transport delay -- so each fix is compared against the truth
    # one step earlier. Comparing at equal timestamps instead would show
    # speed x 0.1 s (hundreds of metres at rocket velocities), which is
    # latency, not receiver error.
    # `step` is the communication step, taken from the truth log: the fix
    # stamps cannot supply it once the dynamics envelope drops whole runs of
    # epochs, and with a launch-vehicle envelope it may drop nearly all of them.
    stamps = fixes["sim_time_s"]
    truth_by_time = {round(t, 3): i for i, t in enumerate(truth["time"])}
    times, horizontal, vertical = [], [], []
    for i, t in enumerate(stamps):
        j = truth_by_time.get(round(t - step, 3))
        if j is None:
            continue
        # Break the line across a dropout rather than drawing a chord over it.
        if times and t - times[-1] > 1.5 * step:
            times.append(t - step)
            horizontal.append(float("nan"))
            vertical.append(float("nan"))
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
    outages = read_outages(args.fixes)
    args.out.mkdir(parents=True, exist_ok=True)

    times = truth["time"]
    step = min(b - a for a, b in zip(times, times[1:])) if len(times) > 1 else 0.1

    epochs = sum(1 for _ in args.fixes.open(newline="")) - 1  # minus the header
    valid = len(fixes["sim_time_s"])
    if valid < epochs:
        windows = ", ".join(f"{a:.1f}-{b:.1f} s" for a, b in outages)
        print(f"{epochs - valid} of {epochs} NAV-PVT epochs carried no fix "
              f"(receiver dynamics envelope: {windows}); plotting the {valid} valid ones")

    figures = 0
    plot_gps_availability(truth, outages, args.out)
    figures += 1

    # Two fixes is the least that says anything: one point draws no track and
    # the error figure needs a pair to align against truth. A launch-vehicle
    # envelope really can leave fewer than that, so skip rather than crash.
    if valid >= 2:
        plot_altitude(truth, fixes, outages, args.out)
        plot_ground_track(truth, fixes, args.out)
        plot_velocity(truth, fixes, outages, args.out)
        plot_gps_error(truth, fixes, step, args.out)
        figures += 4
    else:
        print(f"only {valid} epoch(s) carried a fix -- skipping the four fix-dependent figures. "
              f"Rerun the co-simulation with --dyn-model -1 --no-cocom for a receiver that keeps one.")

    if args.imu.exists():
        imu = read_fixes(args.imu)  # same plain-CSV shape as the fix log
        plot_imu_specific_force(truth, imu, args.out)
        plot_imu_body_rates(truth, imu, args.out)
        figures += 2
    print(f"wrote {figures} figures to {args.out}/")


if __name__ == "__main__":
    main()
