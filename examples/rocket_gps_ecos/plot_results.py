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

    trajectory_3d.png        the flight in space: navigated stub vs. dark remainder
    altitude_vs_time.png     truth altitude + decoded GPS fixes + staging marker
    velocity_vs_time.png     truth speed over ground + GPS-reported speed
    gps_error.png            horizontal/vertical decoded-fix error vs truth
    gps_availability.png     why the receiver dropped out: truth vs. its envelope
    imu_specific_force.png   truth specific force + decoded accelerometer samples
    imu_body_rates.png       truth body rates + decoded gyroscope samples

Usage (from the co-simulation working directory, after a run):

    python plot_results.py [--truth results/rocket_truth.csv]
                           [--fixes results/gps_fixes.csv]
                           [--imu results/imu_samples.csv] [--out plots/]

The default configuration (COCOM export limits, no platform model) leaves 312
of 2001 epochs with a usable fix, which is enough to draw all seven. A tighter
envelope can leave fewer: ``--dyn-model 8`` leaves *one*, because the airborne
<4 g model's acceleration limit trips on the second epoch of a launch. Below
two usable epochs the fix-dependent figures are skipped rather than drawn
empty; gps_availability.png and trajectory_3d.png still say something either
way (both are drawn from truth, with the fixes only overlaid), and the IMU
figures are unaffected.

Every figure is stamped at its foot with the receiver configuration that
produced it, read from the ``.config`` sidecar rocket_gps_cosim writes beside
the truth log. Run this against ``--no-cocom`` output and the plots are
identical in form while saying the opposite thing, so the stamp is the only
thing distinguishing them once a PNG leaves its caption behind.

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

# Decoded-sensor markers, sized so a dense cloud still reads as points laid
# over the truth line rather than as a second line. Sensor samples outnumber
# truth points 10:1 on the IMU plots, so those stay smaller.
MARKER_GPS = 5.0
MARKER_IMU = 2.5

METERS_PER_DEG_LAT = 111_320.0
# WGS-84 equatorial radius, for the local ENU frame the 3-D trajectory is drawn
# in. A sphere is accurate enough for a picture: over this flight the
# ellipsoidal correction is metres against hundreds of kilometres.
EARTH_RADIUS_M = 6_378_137.0


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


def read_run_config(truth_path: Path) -> dict[str, str]:
    """Reads the `<truth stem>.config` sidecar rocket_gps_cosim writes, if present."""
    path = truth_path.with_suffix(".config")
    if not path.exists():
        return {}
    config: dict[str, str] = {}
    for line in path.read_text().splitlines():
        key, _, value = line.partition("=")
        if key and value:
            config[key.strip()] = value.strip()
    return config


def config_caption(config: dict[str, str]) -> str:
    """One line describing the receiver configuration a run used.

    Every figure carries this. Without it the envelope-unlocked comparison run
    -- which reports a fix at every epoch -- looks like a contradiction of the
    COCOM result the default configuration produces, and there is nothing in
    the image to tell the two apart.
    """
    if not config:
        return "run configuration unknown (no .config sidecar beside the truth log)"

    platform = config.get("dynamic_platform", "?")
    cocom = config.get("cocom_limits_enabled", "?") == "1"
    if platform == "-1" and not cocom:
        return ("receiver dynamics envelope DISABLED (--dyn-model -1 --no-cocom): "
                "a waivered receiver, reporting through the whole flight")
    limits = []
    if platform != "-1":
        limits.append(f"u-blox dynModel {platform}")
    limits.append("COCOM limits in force" if cocom else "COCOM limits cleared")
    return "receiver dynamics envelope: " + ", ".join(limits)


def stamp(fig, caption: str) -> None:
    """Puts the run's provenance at the foot of a figure, out of the data's way."""
    fig.tight_layout(rect=(0.0, 0.045, 1.0, 1.0))
    fig.text(0.5, 0.012, caption, ha="center", va="bottom", fontsize=8, color=INK_2)


def rolling_rms(values: list[float], window: int = 101) -> list[float]:
    """Centred rolling RMS, for reading a noise level off a cloud of samples."""
    out: list[float] = []
    for i in range(len(values)):
        chunk = values[max(0, i - window // 2):min(len(values), i + window // 2 + 1)]
        out.append(math.sqrt(sum(v * v for v in chunk) / len(chunk)))
    return out


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


def to_enu_km(lat_rad: float, lon_rad: float, alt_m: float,
              origin: tuple[float, float, float]) -> tuple[float, float, float]:
    """Geodetic -> local east/north/up in km, on a sphere about the launch site."""
    lat0, lon0, alt0 = origin
    east = (lon_rad - lon0) * EARTH_RADIUS_M * math.cos(lat0)
    north = (lat_rad - lat0) * EARTH_RADIUS_M
    return east / 1000.0, north / 1000.0, (alt_m - alt0) / 1000.0


def style_axis_3d(ax) -> None:
    """The 2-D styling has no counterpart on a 3-D axes, which has panes instead."""
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1.0, 1.0, 1.0, 0.0))   # transparent panes: the box would dominate
        axis.line.set_color(GRID)
        axis._axinfo["grid"]["color"] = GRID
        axis._axinfo["grid"]["linewidth"] = 0.6
        axis.label.set_color(INK_2)
    ax.tick_params(colors=INK_2, labelsize=8)
    ax.title.set_color(INK)


def plot_trajectory_3d(truth, fixes, outages, out: Path, caption: str,
                       config: dict[str, str]) -> None:
    """The flight in space, and the fraction of it the receiver navigated.

    Every other GPS figure on this page plots against time, where the dropout
    reads as a shaded band of a certain width. In space it reads as what it
    actually costs: the receiver's entire contribution is a stub near the
    origin of a trajectory that runs 543 km downrange and 237 km up, and the
    rest of the ascent -- both staging events, all of stage 2 -- is flown with
    no GNSS at all.

    Hence two panels. On the overview the navigated portion is 4% of the track
    and cannot be drawn as a distinguishable segment -- it is about twenty
    pixels wide, and the fix cloud covers it -- so the truth line is one
    continuous curve there and the fixes mark their own extent. The zoom is
    where that stub is actually resolvable, and it is the panel that shows what
    the flight software had to work with.

    Scenario 17 launches due east from the equator with no cross-range
    manoeuvre, so the whole flight lies in one plane to within 0.1 m over
    543 km. The north axis is drawn anyway, and annotated, because the honest
    thing to do with an empty dimension is to show that it is empty rather than
    quietly project it away.
    """
    origin = (truth["lat_rad"][0], truth["lon_rad"][0], truth["alt_m"][0])
    east, north, up = zip(*[to_enu_km(la, lo, al, origin)
                            for la, lo, al in zip(truth["lat_rad"], truth["lon_rad"], truth["alt_m"])])
    times = truth["time"]
    t_lost = outages[0][0] if outages else None

    fix_east: tuple[float, ...] = ()
    fix_north: tuple[float, ...] = ()
    fix_up: tuple[float, ...] = ()
    if len(fixes["sim_time_s"]) >= 2:
        fix_east, fix_north, fix_up = zip(
            *[to_enu_km(math.radians(la), math.radians(lo), al, origin)
              for la, lo, al in zip(fixes["latitude_deg"], fixes["longitude_deg"], fixes["altitude_m"])])

    fig = plt.figure(figsize=(9.6, 5.0), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax_all = fig.add_subplot(1, 2, 1, projection="3d")
    ax_zoom = fig.add_subplot(1, 2, 2, projection="3d")

    def draw(ax, last: int, fix_count: int) -> None:
        """Draws the trajectory up to truth index `last` and `fix_count` fixes."""
        ax.set_facecolor(SURFACE)
        style_axis_3d(ax)
        # Ground track on the floor of the box: a 3-D curve with no shadow is
        # ambiguous about which of its dimensions is doing the climbing.
        ax.plot(east[:last], north[:last], [0.0] * last, color=INK_2, linewidth=0.9,
                linestyle=":", alpha=0.45, zorder=1, label="ground track (projected)")
        # Fixes under the truth line, not over it: at 1.5 m noise on a 21 km
        # track the two coincide, and whichever is drawn second hides the
        # other. The line is the thinner mark, so it goes on top.
        if fix_count:
            ax.plot(fix_east[:fix_count], fix_north[:fix_count], fix_up[:fix_count],
                    linestyle="none", marker="o", markersize=MARKER_GPS, alpha=0.55,
                    color=GPS, markeredgewidth=0, zorder=2,
                    label="GPS fixes decoded by flight computer")
        ax.plot(east[:last], north[:last], up[:last], color=TRUTH, linewidth=1.6,
                zorder=3, label="rocket truth (TwoStageRocket.fmu)")
        # An axis whose data spans centimetres cannot be auto-scaled: matplotlib
        # would zoom the numerical dust up to full width and invent a wander
        # that is not there. Give it a share of the downrange extent instead,
        # so the flight reads as the planar thing it is.
        span = max(east[:last]) * 0.16
        ax.set_xlim(0.0, max(east[:last]) * 1.02)
        ax.set_ylim(-span, span)
        ax.set_zlim(0.0, max(up[:last]) * 1.08)
        ax.set_yticks([-round(span, 1), 0.0, round(span, 1)])
        # Generous label padding: on a 3-D axes the tick labels splay out along
        # the projected axis, and a default-padded label lands on top of them.
        ax.set_xlabel("east of launch site [km]", labelpad=14)
        ax.set_ylabel("north [km]", labelpad=8)
        ax.set_zlabel("altitude [km]", labelpad=4)
        # A shallow box in the north direction: that axis carries 5 cm of data,
        # so giving it a third of the depth a cube would is both honest about
        # the flight being planar and buys the other two axes the room.
        ax.set_box_aspect((1.75, 0.5, 1.0))
        # Oblique rather than edge-on: edge-on would look like the 2-D altitude
        # plot and would hide the ground track under the trajectory.
        ax.view_init(elev=20.0, azim=-58.0)

    def mark(ax, index: int, text: str, offset: tuple[float, float], scale: float,
             ha: str = "left", va: str = "bottom") -> None:
        """Dot on the trajectory, leader line out to a label in clear space.

        3-D text takes data coordinates in all three axes; the offset stays in
        east and up only, so the label and its leader sit in the flight plane
        rather than floating out of it.
        """
        x, y, z = east[index], north[index], up[index]
        dx, dz = offset[0] * scale, offset[1] * scale
        ax.plot([x, x + dx], [y, y], [z, z + dz], color=INK_2, linewidth=0.7,
                alpha=0.5, zorder=4)
        ax.plot([x], [y], [z], linestyle="none", marker="o", markersize=4.0,
                color=INK_2, markeredgewidth=0, zorder=5)
        ax.text(x + dx, y, z + dz, text, fontsize=7.5, color=INK_2, ha=ha, va=va, zorder=6)

    # --- overview: the whole 200 s window ------------------------------------
    draw(ax_all, len(times), len(fix_east))
    scale = max(east) / 100.0
    if t_lost is not None:
        lost = next((i for i, t in enumerate(times) if t >= t_lost), len(times)) - 1
        # The count line only when there were fixes: an envelope tight enough to
        # leave none puts this marker on the launch pad, where "all 0 fixes are
        # in here" is a joke the figure should not be making.
        text = f"fix lost, t = {t_lost:.1f} s"
        if fix_east:
            text += f"\nall {len(fix_east)} fixes are in here"
        mark(ax_all, lost, text, (-2.0, 17.0), scale, ha="center")
    ignition = config.get("stg2_ignition_s")
    if ignition is not None:
        i = min(range(len(times)), key=lambda k: abs(times[k] - float(ignition)))
        mark(ax_all, i, f"stage 2 ignition\nt = {float(ignition):.1f} s",
             (7.0, -13.0), scale, va="top")
    mark(ax_all, len(times) - 1,
         f"t = {times[-1]:.0f} s, {up[-1]:.0f} km up\nand still climbing",
         (-11.0, 3.0), scale, ha="right")
    # Panel names inside the axes rather than as titles: a 3-D axes' title sits
    # at the top of its reserved region, which is a long way above the box, and
    # collides with the shared legend.
    ax_all.text2D(0.0, 0.98, "the whole flight", transform=ax_all.transAxes,
                  fontsize=10, color=INK, ha="left", va="top")

    # --- zoom: the part the receiver was alive for ---------------------------
    # A little past staging, so the zoom carries one event beyond the dropout
    # and the reader can see the flight continuing out of the navigated stub.
    t_stage = staging_time(truth)
    t_zoom = (t_stage + 4.0) if t_stage is not None else (times[-1] * 0.2)
    last = next((i for i, t in enumerate(times) if t > t_zoom), len(times))
    draw(ax_zoom, last, len(fix_east))
    scale = max(east[:last]) / 100.0
    if t_lost is not None and fix_east:
        lost = next((i for i, t in enumerate(times) if t >= t_lost), len(times)) - 1
        mark(ax_zoom, lost, f"fix lost, t = {t_lost:.1f} s\n{up[lost]:.0f} km up, "
                            f"{east[lost]:.0f} km downrange", (5.0, -15.0), scale, va="top")
    if t_stage is not None:
        i = min(range(len(times)), key=lambda k: abs(times[k] - t_stage))
        mark(ax_zoom, i, f"stage 1 separation\nt = {t_stage:.1f} s",
             (-13.0, 7.0), scale, ha="right")
    ax_zoom.text2D(0.0, 0.98,
                   f"the navigated stub (first {t_zoom:.0f} s), magnified" if fix_east
                   else f"the first {t_zoom:.0f} s, magnified — no fix in any of it",
                   transform=ax_zoom.transAxes, fontsize=10, color=INK, ha="left", va="top")

    handles, labels = ax_all.get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc="upper center", frameon=False, fontsize=9,
                     ncol=3, bbox_to_anchor=(0.5, 0.945))
    for text in leg.get_texts():
        text.set_color(INK_2)
    fig.suptitle(f"The flight in space: {t_lost:.0f} s navigated, {times[-1] - t_lost:.0f} s dark"
                 if t_lost is not None else "The flight in space",
                 fontsize=12, color=INK, x=0.5, y=0.985)

    cross_range_m = max(abs(n) for n in north) * 1000.0
    fig.text(0.5, 0.062, f"cross-range over the whole flight: {cross_range_m:.2f} m — "
                         f"Scenario 17 launches due east from the equator and never leaves that plane",
             ha="center", fontsize=8, color=INK_2)
    fig.tight_layout(rect=(0.01, 0.10, 0.99, 0.88))
    # A 3-D axes reserves a square region and then draws a box that, with this
    # box_aspect, occupies rather less than half of it. Overflowing each axes
    # past its allotted cell is what turns that margin back into figure.
    for ax in (ax_all, ax_zoom):
        box = ax.get_position()
        ax.set_position((box.x0 - box.width * 0.03, box.y0 - box.height * 0.10,
                         box.width * 1.06, box.height * 1.24))
    fig.text(0.5, 0.012, caption, ha="center", va="bottom", fontsize=8, color=INK_2)
    fig.savefig(out / "trajectory_3d.png", facecolor=SURFACE)
    plt.close(fig)


def plot_altitude(truth, fixes, outages, out: Path, caption: str) -> None:
    fig, ax = new_figure()
    shade_outages(ax, outages)
    # GPS dots first, truth line on top -- the two overlap almost everywhere,
    # and the line must stay visible. The markers are deliberately wider than
    # the line so the fix cloud reads as a distinct band around it rather than
    # disappearing under it; at 2001 fixes anything smaller is indistinguishable
    # from a slightly thicker line.
    ax.plot(fixes["sim_time_s"], [a / 1000.0 for a in fixes["altitude_m"]],
            linestyle="none", marker="o", markersize=MARKER_GPS, alpha=0.5, color=GPS,
            markeredgewidth=0, label="GPS fixes decoded by flight computer")
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
    stamp(fig, caption)
    fig.savefig(out / "altitude_vs_time.png", facecolor=SURFACE)
    plt.close(fig)


def plot_velocity(truth, fixes, outages, out: Path, caption: str) -> None:
    fig, ax = new_figure()
    shade_outages(ax, outages)
    speed = [math.hypot(n, e) for n, e in zip(truth["v_north_m_s"], truth["v_east_m_s"])]
    ax.plot(fixes["sim_time_s"], fixes["ground_speed_mps"],
            linestyle="none", marker="o", markersize=MARKER_GPS, alpha=0.5, color=GPS,
            markeredgewidth=0, label="GPS-reported speed over ground")
    ax.plot(truth["time"], speed, color=TRUTH, linewidth=1.6,
            label="truth speed over ground")
    t_stage = staging_time(truth)
    if t_stage is not None:
        annotate_event(ax, t_stage, "staging")
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("speed over ground [m/s]")
    ax.set_title("Speed over ground: truth vs. UBX-NAV-PVT gSpeed")
    legend(ax)
    stamp(fig, caption)
    fig.savefig(out / "velocity_vs_time.png", facecolor=SURFACE)
    plt.close(fig)


def plot_gps_availability(truth, outages, out: Path, caption: str, config: dict[str, str]) -> None:
    """Shows the truth state against the limits that took the fix away.

    Two panels sharing a time axis, because the COCOM cut-off is an AND of
    altitude and speed and neither alone explains the dropout.

    Which limits get drawn depends on which were in force. A platform model's
    thresholds are only annotated when one is configured -- drawing an 80 km
    ceiling that nothing is enforcing would invite the reader to attribute the
    dropout to it.
    """
    platform = config.get("dynamic_platform", "-1")
    platform_active = platform != "-1"

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
    if platform_active:
        threshold(ax_alt, 80.0, f"dynModel {platform} platform max, 80 km", ":")
    # Log above the thresholds, linear through them: the interesting crossings
    # are three decades below the final altitude. The negative half of a symlog
    # axis is dead space here, so it is clipped away.
    ax_alt.set_yscale("symlog", linthresh=10.0)
    ax_alt.set_ylim(bottom=0.0)
    ax_alt.set_ylabel("altitude [km]")
    ax_alt.set_title("Why the receiver goes dark: truth against its own envelope")
    legend(ax_alt)

    speed_3d = [math.sqrt(n * n + e * e + d * d)
                for n, e, d in zip(truth["v_north_m_s"], truth["v_east_m_s"], truth["v_down_m_s"])]
    ax_vel.plot(truth["time"], speed_3d, color=TRUTH, linewidth=1.6, label="truth speed (3-D)")
    speed_label = "COCOM 515 m/s"
    if platform_active:
        # A platform model's own horizontal cap sits just below the export
        # threshold; two lines would say the same thing at this scale.
        speed_label += " (platform cap 500 m/s just below)"
    threshold(ax_vel, 515.0, speed_label, "--")
    ax_vel.set_yscale("symlog", linthresh=100.0)
    ax_vel.set_ylim(bottom=0.0)
    ax_vel.set_xlabel("simulation time [s]")
    ax_vel.set_ylabel("speed [m/s]")
    legend(ax_vel)

    # Mark where the fix actually goes, and name the cause the panels support.
    # With COCOM alone that is the altitude crossing (speed passed 515 m/s long
    # before, and the cut-off is an AND); with a platform model configured it
    # is whichever of its limits bit first, which on a launch vehicle is the
    # acceleration one and is not visible on either axis.
    if outages:
        lost = outages[0][0]
        cause = ("platform-model limit, before either threshold above" if platform_active
                 else "18 km crossing; 515 m/s was passed at t = 10.1 s")
        ax_alt.annotate(f"fix lost at t = {lost:.1f} s\n({cause})",
                        xy=(lost, 18.0), xycoords="data",
                        xytext=(28, -34), textcoords="offset points", fontsize=8, color=INK_2,
                        arrowprops=dict(arrowstyle="->", color=INK_2, linewidth=0.9))

    stamp(fig, caption)
    fig.savefig(out / "gps_availability.png", facecolor=SURFACE)
    plt.close(fig)


def plot_gps_error(truth, fixes, step: float, out: Path, caption: str) -> None:
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
        lat_t = math.degrees(truth["lat_rad"][j])
        lon_t = math.degrees(truth["lon_rad"][j])
        dlat_m = (fixes["latitude_deg"][i] - lat_t) * METERS_PER_DEG_LAT
        dlon_m = (fixes["longitude_deg"][i] - lon_t) * METERS_PER_DEG_LAT * math.cos(math.radians(lat_t))
        times.append(t)
        horizontal.append(math.hypot(dlat_m, dlon_m))
        vertical.append(abs(fixes["altitude_m"][i] - truth["alt_m"][j]))

    # One panel per component, rather than two lines fighting over one axis:
    # they have different scales (1.5 m per horizontal axis against 3 m
    # vertical) and the one drawn second simply hid the other. Per-fix values
    # as a faint scatter, with a rolling RMS on top -- the RMS is what can
    # actually be compared against the configured sigma, since a cloud of
    # samples has no visible mean.
    fig, (ax_h, ax_v) = plt.subplots(2, 1, figsize=(8.0, 5.0), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax_h, ax_v):
        style_axis(ax)

    # Horizontal error is the magnitude of two independent axes, so its RMS is
    # sigma*sqrt(2), not sigma. Vertical is a single axis and lands on sigma.
    horizontal_sigma = 1.5
    vertical_sigma = 3.0
    for ax, series, color, expected, name in (
            (ax_h, horizontal, TRUTH, horizontal_sigma * math.sqrt(2.0), "horizontal"),
            (ax_v, vertical, GPS, vertical_sigma, "vertical")):
        ax.plot(times, series, linestyle="none", marker=".", markersize=2, alpha=0.25,
                color=color, label=f"{name} error, per fix")
        ax.plot(times, rolling_rms(series), color=color, linewidth=1.6,
                label=f"{name} RMS, 10 s window")
        ax.axhline(expected, color=INK_2, linewidth=1.0, linestyle="--", alpha=0.7)
        ax.annotate(f"expected RMS {expected:.2f} m", xy=(0.995, expected),
                    xycoords=("axes fraction", "data"), xytext=(0, 4),
                    textcoords="offset points", ha="right", fontsize=8, color=INK_2)
        ax.set_ylabel(f"{name} error [m]")
        ax.set_ylim(bottom=0.0)
        legend(ax)
    ax_h.set_title("GPS error injected by GpsNoiseModel, recovered end-to-end")
    ax_v.set_xlabel("simulation time [s]")
    stamp(fig, caption)
    fig.savefig(out / "gps_error.png", facecolor=SURFACE)
    plt.close(fig)


def plot_imu_specific_force(truth, imu, out: Path, caption: str) -> None:
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
    stamp(fig, caption)
    fig.savefig(out / "imu_specific_force.png", facecolor=SURFACE)
    plt.close(fig)


def plot_imu_body_rates(truth, imu, out: Path, caption: str) -> None:
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
    stamp(fig, caption)
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
    config = read_run_config(args.truth)
    caption = config_caption(config)
    args.out.mkdir(parents=True, exist_ok=True)
    print(caption)

    times = truth["time"]
    step = min(b - a for a, b in zip(times, times[1:])) if len(times) > 1 else 0.1

    epochs = sum(1 for _ in args.fixes.open(newline="")) - 1  # minus the header
    valid = len(fixes["sim_time_s"])
    if valid < epochs:
        windows = ", ".join(f"{a:.1f}-{b:.1f} s" for a, b in outages)
        print(f"{epochs - valid} of {epochs} NAV-PVT epochs carried no fix "
              f"(receiver dynamics envelope: {windows}); plotting the {valid} valid ones")

    figures = 0
    plot_trajectory_3d(truth, fixes, outages, args.out, caption, config)
    plot_gps_availability(truth, outages, args.out, caption, config)
    figures += 2

    # Two fixes is the least that says anything: one point draws no track and
    # the error figure needs a pair to align against truth. A launch-vehicle
    # envelope really can leave fewer than that, so skip rather than crash.
    if valid >= 2:
        plot_altitude(truth, fixes, outages, args.out, caption)
        plot_velocity(truth, fixes, outages, args.out, caption)
        plot_gps_error(truth, fixes, step, args.out, caption)
        figures += 3
    else:
        print(f"only {valid} epoch(s) carried a fix -- skipping the three fix-dependent figures. "
              f"A platform model on top of COCOM (--dyn-model 8) does this: its acceleration limit "
              f"trips on the second epoch of a launch.")

    if args.imu.exists():
        imu = read_fixes(args.imu)  # same plain-CSV shape as the fix log
        plot_imu_specific_force(truth, imu, args.out, caption)
        plot_imu_body_rates(truth, imu, args.out, caption)
        figures += 2
    print(f"wrote {figures} figures to {args.out}/")


if __name__ == "__main__":
    main()
