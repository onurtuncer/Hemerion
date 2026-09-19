# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Plots the f16_trim_ecos co-simulation results for the Sphinx docs.

Reads the six CSVs the example produces (the host's plant-truth log and the
flight computer's decoded GPS, IMU, barometer, magnetometer and radar-altimeter
logs), joins them on simulation time, and renders the figures embedded in
doc/f16_trim_ecos_cosim.rst:

    <case>_ground_track.png          the flyout in plan view, truth + decoded fixes
    <case>_altitude_consistency.png  four independent altitudes, and their residuals
    <case>_heading_consistency.png   truth yaw vs GPS course vs magnetic heading
    <case>_imu_specific_force.png    body specific force, truth + decoded counts
    <case>_imu_body_rates.png        body rates, against the gyroscope's resolution
    <case>_sensor_envelopes.png      each stack against the limit that bounds it
    <case>_nesc_envelope.png         drift against the published participant spread

``<case>`` is read from the ``.config`` sidecar ``f16_trim_cosim`` writes beside
the truth log, so a case-11 and a case-12 run write different files into the
same directory rather than overwriting each other. That prefix is not
decoration: the two cases produce figures identical in *form* and opposite in
*content* — on case 11 every stack sits inside its envelope for 200 s, on case
12 three of them are outside from the first epoch — and a PNG separated from
its caption has nothing else to tell you which run it came from. Every figure
is additionally stamped at the foot with the check-case, the flight condition
and the receiver configuration.

Usage (from the co-simulation working directory, after a run):

    python plot_results.py [--truth results/f16_truth.csv] [--fixes ...]
                           [--imu ...] [--baro ...] [--mag ...] [--radalt ...]
                           [--reference <NESC csv>]... [--out plots/]

**Pace the run.** The BMP390 and MMC5983MA are polled at the flight computer's
loop rate, not their programmed ODR, so an unpaced 200 s flight yields about 37
barometer conversions — enough to plot, not enough to read a trace against GPS
and radar altimetry. ``f16_trim_cosim --rtf 1`` gives roughly 23x that. The
altitude- and heading-consistency figures are the ones that suffer; the GPS,
IMU and radar-altimeter figures are unaffected, since their data queues in a
socket or a FIFO.

Only matplotlib is required. The truth CSV is written by the host's
``TruthLogger`` in Ecos ``csv_writer`` format (", "-separated, "name[TYPE]"
headers); the five sensor CSVs are written by ``f16_flight_computer``; the
optional NESC references are the published check-case files, in feet and
degrees.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Colors follow the repo docs' data-viz palette, and match plot_results.py in
# the rocket example so the two pages read as one system: categorical slot 1
# (blue) for truth, slot 2 (green) for what the flight computer decoded,
# neutral inks for text and annotation.
TRUTH = "#2a78d6"
GPS = "#1baf7a"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d9d8d4"
SURFACE = "#fcfcfb"
# A neutral wash for spans where a sensor reported nothing usable -- absent
# data, so it must read as background rather than as a series.
OUTAGE = "#eceae5"
# Threshold and rating lines are annotation, not data: warm enough to read
# against the categorical colors without competing with them.
LIMIT = "#b4632a"
# The third and fourth sensor series on the consistency figures. Four
# independent measurements of one quantity is the whole point of those plots,
# so they need four distinguishable inks rather than two plus reuse.
BARO = "#7b5bd6"
RADALT = "#c2185b"

# A quiet card behind an annotation that has to sit inside a dense cloud of
# sample markers, where plain text on the samples is unreadable.
ANNOTATION_CARD = dict(boxstyle="round,pad=0.35", facecolor=SURFACE, edgecolor=GRID,
                       linewidth=0.6, alpha=0.92)

MARKER_GPS = 5.0
MARKER_IMU = 2.0
MARKER_SENSOR = 3.0

FT_PER_M = 1.0 / 0.3048
# WGS-84 equatorial radius, for the local ENU frame the ground track is drawn
# in. A sphere is accurate enough for a picture: over 34 km of flight the
# ellipsoidal correction is centimetres.
EARTH_RADIUS_M = 6_378_137.0

# ---------------------------------------------------------------------------
# Part limits. Each is the bound the corresponding figure draws, and each is
# the reason one of the two check-cases exists.
# ---------------------------------------------------------------------------
# RadAltModel's maximum tracking range: beyond this the part reports a frame
# with no return rather than falling silent.
RADALT_MAX_RANGE_M = 6000.0
# BMP390 rated operating envelope (Bosch datasheet, and BaroNoiseModel's
# out-of-rating counters).
BMP390_P_MIN_PA = 30_000.0
BMP390_P_MAX_PA = 125_000.0
BMP390_T_MIN_C = -40.0
BMP390_T_MAX_C = 85.0
# u-blox dynamic platform model 8 ("airborne <4g"): the speed bound that takes
# the fix away on check-case 12 from the very first epoch.
UBLOX_DYNM8_SPEED_LIMIT_MS = 500.0
# COCOM export limits, applied as an AND.
COCOM_ALT_LIMIT_M = 18_000.0
COCOM_SPEED_LIMIT_MS = 515.0


# ---------------------------------------------------------------------------
# Readers
# ---------------------------------------------------------------------------
def read_truth(path: Path) -> dict[str, list[float]]:
    """Reads the host truth log into {short_name: column}.

    The header is Ecos ``csv_writer`` format, and the log carries variables
    from four instances, so the instance prefix is what distinguishes them:
    ``f16::out.alt_m`` is the plant's own altitude and ``radalt::h_agl_m`` is
    the height the radar altimeter was handed. Stripping to the last ``::``
    segment and dropping ``out.`` leaves those distinct (``alt_m`` and
    ``h_agl_m``) while giving every column a name short enough to index with.
    """
    with path.open(newline="") as f:
        rows = list(csv.reader(f))
    header = [h.strip() for h in rows[0]]
    names = [h.split("::")[-1].removeprefix("out.").split("[")[0] for h in header]
    data: dict[str, list[float]] = {n: [] for n in names}
    for row in rows[1:]:
        if len(row) != len(names):
            continue
        for name, cell in zip(names, row):
            data[name].append(float(cell))
    return data


def read_samples(path: Path) -> dict[str, list[float]]:
    """Reads a flight-computer log, keeping only rows that carry a measurement.

    Two of the five logs record epochs the part reported *nothing usable* for,
    and they record them deliberately: the GPS keeps emitting NAV-PVT with
    ``gnssFixOK`` clear when the vehicle leaves the receiver's envelope, and
    the radar altimeter keeps sending frames with the no-return flag set when
    the ground is out of range. Those rows carry no position and no range, so
    plotting them as data would invent measurements the parts never made --
    but dropping them from the *file* would erase the evidence that the parts
    were alive and talking, which on check-case 12 is the entire result.
    Hence: filtered here, kept there. ``read_gaps`` reads the same rows for
    the shading.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        # Read the header inside the `with`: on an empty file DictReader never
        # touches it, and reaching for fieldnames after the close raises.
        fields = reader.fieldnames or []
        rows = list(reader)
    # A run still in progress leaves a half-written final line, and a reader
    # that trips over it cannot be used to look at a long run while it flies.
    # Drop any row with a missing or empty cell rather than guessing at it.
    rows = [r for r in rows if all(r.get(k) not in (None, "") for k in fields)]
    if "fix_type" in fields:
        rows = [r for r in rows if float(r["fix_type"]) > 0.0]
    elif "valid" in fields:
        rows = [r for r in rows if float(r["valid"]) > 0.0]
    return {k: [float(r[k]) for r in rows] for k in fields}


def read_gaps(path: Path) -> list[tuple[float, float]]:
    """Finds the windows a part spent reporting no usable measurement.

    Returned as (start, end) pairs in simulation time, merged across
    consecutive epochs. Works for either kind of "nothing to say" column --
    the GPS's ``fix_type`` and the radar altimeter's ``valid`` -- because both
    log every epoch whether or not it carried data.
    """
    if not path.exists():
        return []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        column = "fix_type" if "fix_type" in fields else ("valid" if "valid" in fields else None)
        if column is None:
            return []
        rows = list(reader)
    if not rows:
        return []

    times = [float(r["sim_time_s"]) for r in rows]
    step = min((b - a for a, b in zip(times, times[1:])), default=0.1)
    intervals: list[tuple[float, float]] = []
    start: float | None = None
    previous = times[0]
    for row, t in zip(rows, times):
        if float(row[column]) <= 0.0:
            if start is None:
                start = t
            previous = t
        elif start is not None:
            intervals.append((start, previous + step))
            start = None
    if start is not None:
        intervals.append((start, previous + step))
    return intervals


def count_rows(path: Path) -> int:
    """Total logged epochs, including the ones carrying no measurement."""
    if not path.exists():
        return 0
    with path.open(newline="") as f:
        return max(0, sum(1 for _ in f) - 1)


def read_run_config(truth_path: Path) -> dict[str, str]:
    """Reads the ``<truth stem>.config`` sidecar f16_trim_cosim writes."""
    path = truth_path.with_suffix(".config")
    if not path.exists():
        return {}
    config: dict[str, str] = {}
    for line in path.read_text().splitlines():
        key, _, value = line.partition("=")
        if key and value:
            config[key.strip()] = value.strip()
    return config


def read_reference(path: Path) -> dict[str, list[float]]:
    """Reads a published NESC check-case CSV (feet and degrees) into SI.

    Column spellings are matched case-insensitively because the published
    files are not consistent about capitalisation between check-cases.
    """
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    lookup = {name.strip().lower(): name for name in rows[0]}

    def column(key: str) -> str:
        return lookup[key]

    return {
        "time": [float(r[column("time")]) for r in rows],
        "lat_deg": [float(r[column("latitude_deg")]) for r in rows],
        "lon_deg": [float(r[column("longitude_deg")]) for r in rows],
        "alt_m": [float(r[column("altitudemsl_ft")]) * 0.3048 for r in rows],
        "yaw_deg": [float(r[column("eulerangle_deg_yaw")]) for r in rows],
    }


# ---------------------------------------------------------------------------
# Provenance
# ---------------------------------------------------------------------------
def case_id(config: dict[str, str]) -> str:
    """The check-case a run flew, as a filename-safe prefix."""
    return "case" + config.get("check_case", "unknown")


def config_caption(config: dict[str, str]) -> str:
    """One line describing the run, stamped at the foot of every figure.

    The two check-cases share a wiring, an executable and a figure set; what
    separates them is an altitude and an airspeed. So the stamp leads with the
    flight condition, then the receiver configuration -- the one setting that
    changes which of these figures show data at all.
    """
    if not config:
        return "run configuration unknown (no .config sidecar beside the truth log)"

    check_case = config.get("check_case", "?")
    altitude_ft = config.get("alt0_ft", "?")
    speed_fps = config.get("vt0_fps", "?")
    condition = f"NESC check-case {check_case}: {altitude_ft} ft, {speed_fps} ft/s TAS"

    platform = config.get("dynamic_platform", "?")
    cocom = config.get("cocom_limits_enabled", "?") == "1"
    if platform == "-1" and not cocom:
        receiver = "receiver dynamics envelope DISABLED (--dyn-model -1 --no-cocom)"
    else:
        limits = []
        if platform != "-1":
            limits.append(f"u-blox dynModel {platform}")
        else:
            limits.append("no platform model")
        limits.append("COCOM limits in force" if cocom else "COCOM limits cleared")
        receiver = "receiver: " + ", ".join(limits)

    rtf = config.get("realtime_factor", "")
    pacing = "paced --rtf 1" if rtf == "1" else ("unpaced" if rtf else "")
    parts = [condition, receiver] + ([pacing] if pacing else [])
    return " | ".join(parts)


# ---------------------------------------------------------------------------
# Styling
# ---------------------------------------------------------------------------
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


def new_figure(rows: int = 1, height: float = 4.2, sharex: bool = True):
    fig, axes = plt.subplots(rows, 1, figsize=(8.0, height), dpi=150, sharex=sharex)
    fig.patch.set_facecolor(SURFACE)
    axes = [axes] if rows == 1 else list(axes)
    for ax in axes:
        style_axis(ax)
    return fig, axes


def legend(ax, loc: str = "best", framed: bool = False) -> None:
    """Legend in the house style.

    ``framed`` puts an opaque surface-coloured card behind it. Several of
    these figures are dense clouds of sample markers with no empty corner --
    a residual panel where the samples fill the axes is the normal case, not
    the exception -- and an unframed legend laid over one is unreadable.
    """
    leg = ax.legend(loc=loc, frameon=framed, fontsize=8)
    if framed:
        leg.get_frame().set_facecolor(SURFACE)
        leg.get_frame().set_edgecolor(GRID)
        leg.get_frame().set_linewidth(0.6)
        leg.get_frame().set_alpha(0.92)
    for text in leg.get_texts():
        text.set_color(INK_2)


def add_headroom(ax, fraction: float = 0.32) -> None:
    """Expands the y-range upward to make room for a legend.

    These panels are dense clouds of sample markers with no empty corner, so a
    legend has to be given space rather than found space: laid over the data it
    hides samples, and a reader cannot tell a hidden sample from an absent one.
    Expanding the axis is the honest way to buy the room -- nothing is cropped
    and the data keeps its own scale.
    """
    low, high = ax.get_ylim()
    if high > low:
        ax.set_ylim(low, low + (high - low) * (1.0 + fraction))


def stamp(fig, caption: str) -> None:
    """Puts the run's provenance at the foot of a figure, out of the data's way."""
    fig.tight_layout(rect=(0.0, 0.05, 1.0, 1.0))
    fig.text(0.5, 0.012, caption, ha="center", va="bottom", fontsize=7.5, color=INK_2)


def shade_gaps(ax, intervals: list[tuple[float, float]], label: str | None) -> None:
    for index, (start, end) in enumerate(intervals):
        ax.axvspan(start, end, color=OUTAGE, alpha=0.6, linewidth=0, zorder=0,
                   label=label if index == 0 else None)


def limit_line(ax, value: float, text: str, side: str = "right") -> None:
    """Draws a part limit and labels it without colliding with the title.

    The label goes *below* the line when the line sits in the top fifth of the
    axes and above it otherwise. That case is not rare: a panel drawn to show
    a limit not being reached puts the limit at the top of the y-range by
    construction, which is exactly where the title is.
    """
    ax.axhline(value, color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.9)
    low, high = ax.get_ylim()
    below = high > low and (value - low) / (high - low) > 0.8
    ax.annotate(text, xy=(0.99 if side == "right" else 0.01, value),
                xycoords=("axes fraction", "data"),
                xytext=(0, -4 if below else 3), textcoords="offset points",
                ha="right" if side == "right" else "left",
                va="top" if below else "bottom", fontsize=7.5, color=LIMIT)


def save(fig, out: Path, prefix: str, name: str, caption: str) -> str:
    stamp(fig, caption)
    path = out / f"{prefix}_{name}.png"
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    return path.name


# ---------------------------------------------------------------------------
# Geometry and atmosphere -- reimplemented here rather than imported, because
# a plotting script should need no build products.
# ---------------------------------------------------------------------------
def to_enu_km(lat_deg: float, lon_deg: float, origin: tuple[float, float]) -> tuple[float, float]:
    """Geodetic -> local east/north in km, on a sphere about the trim point."""
    lat0, lon0 = origin
    east = math.radians(lon_deg - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat_deg - lat0) * EARTH_RADIUS_M
    return east / 1000.0, north / 1000.0


_ISA_P0 = 101325.0
_ISA_T0 = 288.15
_ISA_LAPSE = 0.0065
_ISA_H_TROPO = 11000.0
_ISA_T_TROPO = 216.65
_ISA_G = 9.80665
_ISA_R = 287.05287
_ISA_P_TROPO = _ISA_P0 * (_ISA_T_TROPO / _ISA_T0) ** (_ISA_G / (_ISA_LAPSE * _ISA_R))


def isa_pressure_pa(altitude_m: float) -> float:
    if altitude_m <= _ISA_H_TROPO:
        return _ISA_P0 * (1.0 - _ISA_LAPSE * altitude_m / _ISA_T0) ** (_ISA_G / (_ISA_LAPSE * _ISA_R))
    return _ISA_P_TROPO * math.exp(-_ISA_G * (altitude_m - _ISA_H_TROPO) / (_ISA_R * _ISA_T_TROPO))


def isa_altitude_m(pressure_pa: float) -> float:
    """Inverse of isa_pressure_pa -- the pressure altimeter a flight computer runs."""
    if pressure_pa >= _ISA_P_TROPO:
        return (_ISA_T0 / _ISA_LAPSE) * (1.0 - (pressure_pa / _ISA_P0) ** (_ISA_LAPSE * _ISA_R / _ISA_G))
    return _ISA_H_TROPO - (_ISA_R * _ISA_T_TROPO / _ISA_G) * math.log(pressure_pa / _ISA_P_TROPO)


# The centred tilted dipole the host drives the magnetometer FMU with, ported
# from geomagnetic_field.hpp. It is here for one number: the declination at the
# trim point, which is what turns a magnetic heading into a comparison against
# GPS course. Taking it from the same model the truth field came from is the
# point -- a WMM declination would be a *better* number for Kitty Hawk and the
# wrong one for this figure, because the field the part measured is this
# model's, not the WMM's, and the residual would then carry the model
# difference rather than the sensor chain's error.
_DIPOLE_REFERENCE_RADIUS_M = 6_371_200.0
_DIPOLE_EQUATORIAL_FIELD_UT = 31.2
_DIPOLE_POLE_LAT_DEG = 80.65
_DIPOLE_POLE_LON_DEG = -72.68


def dipole_field_ned(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    radius = _DIPOLE_REFERENCE_RADIUS_M + alt_m
    scale = _DIPOLE_EQUATORIAL_FIELD_UT * (_DIPOLE_REFERENCE_RADIUS_M / radius) ** 3

    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    cos_lat, sin_lat = math.cos(lat), math.sin(lat)
    cos_lon, sin_lon = math.cos(lon), math.sin(lon)
    rx, ry, rz = cos_lat * cos_lon, cos_lat * sin_lon, sin_lat

    pole_lat, pole_lon = math.radians(_DIPOLE_POLE_LAT_DEG), math.radians(_DIPOLE_POLE_LON_DEG)
    mx = -math.cos(pole_lat) * math.cos(pole_lon)
    my = -math.cos(pole_lat) * math.sin(pole_lon)
    mz = -math.sin(pole_lat)

    m_dot_r = mx * rx + my * ry + mz * rz
    bx = scale * (3.0 * rx * m_dot_r - mx)
    by = scale * (3.0 * ry * m_dot_r - my)
    bz = scale * (3.0 * rz * m_dot_r - mz)

    north = -sin_lat * cos_lon * bx - sin_lat * sin_lon * by + cos_lat * bz
    east = -sin_lon * bx + cos_lon * by
    down = -cos_lat * cos_lon * bx - cos_lat * sin_lon * by - sin_lat * bz
    return north, east, down


def declination_deg(lat_deg: float, lon_deg: float, alt_m: float) -> float:
    north, east, _ = dipole_field_ned(lat_deg, lon_deg, alt_m)
    return math.degrees(math.atan2(east, north))


def wrap_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def wrap_360(angle_deg: float) -> float:
    return angle_deg % 360.0


def interpolate(times: list[float], values: list[float], t: float) -> float:
    """Linear interpolation onto an arbitrary time, clamped at both ends.

    The truth log is on the 0.1 s communication grid while the sensor logs are
    on their own -- 100 Hz for the IMU, 25 Hz for the radar altimeter,
    whatever the flight computer's loop achieved for the two I2C parts -- so
    every residual on these figures needs truth resampled onto a sensor's
    stamps. Nearest-sample would quantise a sub-metre residual against a
    0.1 s grid at 172 m/s, which is 17 m of ground track.
    """
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    low, high = 0, len(times) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if times[mid] <= t:
            low = mid
        else:
            high = mid
    span = times[high] - times[low]
    if span <= 0.0:
        return values[low]
    weight = (t - times[low]) / span
    return values[low] * (1.0 - weight) + values[high] * weight


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def plot_ground_track(truth, fixes, gps_gaps, out: Path, prefix: str, caption: str) -> str:
    """The flyout in plan view, and the departure the plan view cannot show.

    Two panels, because one cannot carry both facts at an honest scale. To the
    left, the track at true aspect ratio: 34.5 km on a 45 degree heading, and
    it is a straight line. Drawing the decoded fixes on top of it would be a
    lie of resolution -- 2001 fixes with 1.5 m of injected noise sit inside the
    width of the line itself -- so the left panel is truth alone, and the fixes
    appear where they are actually resolvable.

    That is the right panel: cross-track departure from the initial heading,
    in metres against kilometres flown. The check-case asks the aircraft to
    hold 45 degrees, the phugoid slowly turns it, and the resulting few hundred
    metres of bend is the quantity the published participants disagree about
    (see the participant-envelope figure). At this scale the GPS noise is
    finally visible as what it is -- a metre-scale scatter about a track that
    departs by hundreds.
    """
    origin = (truth["lat_deg"][0], truth["lon_deg"][0])
    east, north = zip(*[to_enu_km(la, lo, origin) for la, lo in zip(truth["lat_deg"], truth["lon_deg"])])

    fig, (ax_plan, ax_cross) = plt.subplots(1, 2, figsize=(10.0, 4.4), dpi=150,
                                            gridspec_kw={"width_ratios": [1.0, 1.35]})
    fig.patch.set_facecolor(SURFACE)
    style_axis(ax_plan)
    style_axis(ax_cross)

    ax_plan.plot(east, north, color=TRUTH, linewidth=1.8, label="plant truth", zorder=3)
    ax_plan.plot([east[0]], [north[0]], marker="o", markersize=5, color=INK_2, linestyle="none", zorder=5)
    ax_plan.annotate("trim point", xy=(east[0], north[0]), xytext=(7, -11), textcoords="offset points",
                     fontsize=8, color=INK_2)
    distance_km = math.hypot(east[-1], north[-1])
    heading0_deg = wrap_360(math.degrees(truth["yaw_rad"][0]))
    ax_plan.set_title(f"{distance_km:.1f} km in {truth['time'][-1]:.0f} s on a {heading0_deg:.0f}° heading",
                      fontsize=10)
    ax_plan.set_xlabel("east of trim point [km]")
    ax_plan.set_ylabel("north of trim point [km]")
    ax_plan.set_aspect("equal", adjustable="datalim")

    # Along/cross track about the initial heading. Both series go through the
    # same rotation, so a GPS fix and the truth point it should match land on
    # comparable axes without either being privileged.
    heading0 = truth["yaw_rad"][0]
    sin0, cos0 = math.sin(heading0), math.cos(heading0)

    def along_cross(e: float, n: float) -> tuple[float, float]:
        return e * sin0 + n * cos0, e * cos0 - n * sin0

    along, cross = zip(*[along_cross(e, n) for e, n in zip(east, north)])
    ax_cross.plot(along, [c * 1000.0 for c in cross], color=TRUTH, linewidth=1.6,
                  label="plant truth", zorder=3)
    if len(fixes["sim_time_s"]) >= 2:
        fix_along, fix_cross = zip(*[along_cross(*to_enu_km(la, lo, origin))
                                     for la, lo in zip(fixes["latitude_deg"], fixes["longitude_deg"])])
        ax_cross.plot(fix_along, [c * 1000.0 for c in fix_cross], linestyle="none", marker=".",
                      markersize=MARKER_SENSOR, alpha=0.35, color=GPS,
                      label=f"decoded GPS fixes ({len(fix_along)})", zorder=2)
    else:
        ax_cross.annotate("the receiver reported no fix at any epoch on this flight",
                          xy=(0.5, 0.5), xycoords="axes fraction", ha="center", fontsize=9, color=LIMIT)
    ax_cross.axhline(0.0, color=INK_2, linewidth=0.8, alpha=0.5)
    ax_cross.annotate(f"{cross[-1] * 1000.0:+.0f} m at cut-off",
                      xy=(along[-1], cross[-1] * 1000.0), xytext=(-6, 10), textcoords="offset points",
                      ha="right", fontsize=8, color=INK_2)
    ax_cross.set_xlabel("along track [km]")
    ax_cross.set_ylabel("cross track [m]")
    ax_cross.set_title("Departure from the initial heading", fontsize=10)
    legend(ax_cross, framed=True)
    return save(fig, out, prefix, "ground_track", caption)


def plot_altitude_consistency(truth, fixes, baro, radalt, gps_gaps, radalt_gaps,
                              out: Path, prefix: str, caption: str) -> str:
    """Four independent altitudes of one aircraft, and their disagreement.

    This is the figure check-case 11 exists for. Barometric altitude, GNSS
    altitude and radar height are three physically unrelated measurements —
    a pressure, a set of ranging solutions, and a time of flight to the ground
    — and a fusion filter that trusts their agreement has to be shown that
    agreement first, on a flight where all three are valid. The lower panel is
    the actual content: each stack's residual against plant truth, on one
    scale, so their very different error characters are comparable. The
    barometer's offset is bias (the BMP390 model's per-run turn-on pressure
    offset, read through the ISA), the GPS's is noise, and the radar
    altimeter's is quantisation.

    On check-case 12 the same figure draws the opposite result, which is why it
    is not gated on the case: the GPS trace disappears into a full-width no-fix
    wash, the radar altimeter's does the same past its tracking range, and the
    barometer runs on below the pressure its part is rated for.
    """
    fig, (ax, ax_err) = new_figure(2, height=6.4)

    shade_gaps(ax, gps_gaps, "GPS: no fix")
    ax.plot(truth["time"], truth["alt_m"], color=TRUTH, linewidth=1.5, label="plant truth (MSL)", zorder=4)
    if fixes["sim_time_s"]:
        ax.plot(fixes["sim_time_s"], fixes["altitude_m"], linestyle="none", marker=".",
                markersize=MARKER_GPS, alpha=0.5, color=GPS, label="GPS altitude (decoded UBX)")
    if baro["sim_time_s"]:
        ax.plot(baro["sim_time_s"], [isa_altitude_m(p) for p in baro["pressure_pa"]],
                linestyle="none", marker=".", markersize=MARKER_SENSOR, alpha=0.6, color=BARO,
                label="pressure altitude (BMP390, ISA inverted)")
    if radalt["sim_time_s"]:
        ax.plot(radalt["sim_time_s"], radalt["range_m"], linestyle="none", marker=".",
                markersize=2.0, alpha=0.4, color=RADALT, label="radar height AGL (no terrain model)")
    ax.set_ylabel("altitude [m]")
    # The title counts the sources that actually produced data. On case 11 that
    # is four and the figure is a consistency check; on case 12 it is one, and a
    # title still promising four would be describing the wiring instead of the
    # flight.
    surviving = 1 + sum(1 for series in (fixes, baro, radalt) if series["sim_time_s"])
    ax.set_title("Four independent altitudes of the same aircraft" if surviving == 4
                 else f"Only {surviving} of the four altitude sources produced data on this flight",
                 fontsize=10)
    add_headroom(ax, 0.34)
    legend(ax, loc="upper left", framed=True)

    # Residuals. Truth is resampled onto each sensor's own stamps rather than
    # the reverse: the sensors are the irregular series here, and snapping
    # them to the 0.1 s communication grid would fold 17 m of travel into a
    # residual whose interesting values are metres.
    any_residual = False
    if fixes["sim_time_s"]:
        ax_err.plot(fixes["sim_time_s"],
                    [a - interpolate(truth["time"], truth["alt_m"], t)
                     for t, a in zip(fixes["sim_time_s"], fixes["altitude_m"])],
                    linestyle="none", marker=".", markersize=MARKER_SENSOR, alpha=0.5, color=GPS,
                    label="GPS - truth")
        any_residual = True
    if baro["sim_time_s"]:
        ax_err.plot(baro["sim_time_s"],
                    [isa_altitude_m(p) - interpolate(truth["time"], truth["alt_m"], t)
                     for t, p in zip(baro["sim_time_s"], baro["pressure_pa"])],
                    linestyle="none", marker=".", markersize=MARKER_SENSOR, alpha=0.6, color=BARO,
                    label="pressure altitude - truth")
        any_residual = True
    if radalt["sim_time_s"]:
        ax_err.plot(radalt["sim_time_s"],
                    [r - interpolate(truth["time"], truth["alt_m"], t)
                     for t, r in zip(radalt["sim_time_s"], radalt["range_m"])],
                    linestyle="none", marker=".", markersize=2.0, alpha=0.4, color=RADALT,
                    label="radar height - truth")
        any_residual = True
    if not any_residual:
        ax_err.annotate("no stack produced a usable altitude on this flight",
                        xy=(0.5, 0.5), xycoords="axes fraction", ha="center", fontsize=9, color=LIMIT)
    ax_err.axhline(0.0, color=INK_2, linewidth=0.8, alpha=0.5)
    ax_err.set_xlabel("simulation time [s]")
    ax_err.set_ylabel("residual against truth [m]")
    ax_err.set_title("What each stack disagrees with truth by, on one scale", fontsize=10)
    if any_residual:
        add_headroom(ax_err, 0.34)
        legend(ax_err, loc="upper left", framed=True)
    return save(fig, out, prefix, "altitude_consistency", caption)


def plot_heading_consistency(truth, fixes, mag, gps_gaps, out: Path, prefix: str, caption: str) -> str:
    """Magnetic heading against GNSS course against truth yaw.

    The second half of the cross-sensor claim, and the harder half: a
    magnetometer measures a field in body axes, so recovering a heading from
    it needs the aircraft's own tilt and the local declination. Both are taken
    from the simulation here — truth roll and pitch, and the declination of
    the same centred dipole the host drove the FMU with — because the question
    this figure answers is whether the *sensor chain* (18-bit registers, I2C
    transactions, bridge-offset calibration, unit conversion) preserves
    heading, not whether an attitude filter can be built. The residual is the
    chain's error with the geometry taken out.

    GNSS course is not the same quantity as yaw and the figure does not
    pretend otherwise: course is the direction of the velocity vector over the
    ground, yaw is where the nose points, and on this flight they differ by
    the sideslip and the wind-free drift the phugoid produces.

    **The magnetic residual is a bias, not noise, and it is supposed to be.**
    The MMC5983MA model draws a hard-iron offset once per run (1 uT 1-sigma
    per axis) on top of the bridge offset. The bridge offset is what a
    SET/RESET pair cancels, and the driver's bring-up calibration duly removes
    it; hard iron is a real field the installation adds, so SET/RESET cannot
    touch it and only a magnetic calibration flown through attitudes can. A
    couple of microtesla of it against a ~22 uT horizontal field is a few
    degrees of heading, which is exactly what the lower panel shows. The
    figure measures that offset from the run rather than asserting it -- the
    mean of (decoded - truth field) over the flight -- and prints the heading
    error it predicts beside the one observed, so the two can be checked
    against each other instead of taken on trust.
    """
    fig, (ax, ax_err) = new_figure(2, height=6.4)
    shade_gaps(ax, gps_gaps, "GPS: no fix")

    truth_yaw = [wrap_360(math.degrees(y)) for y in truth["yaw_rad"]]
    ax.plot(truth["time"], truth_yaw, color=TRUTH, linewidth=1.5, label="plant truth yaw", zorder=4)
    if fixes["sim_time_s"]:
        ax.plot(fixes["sim_time_s"], [wrap_360(c) for c in fixes["course_deg"]],
                linestyle="none", marker=".", markersize=MARKER_GPS, alpha=0.5, color=GPS,
                label="GPS course over ground")

    mag_heading: list[float] = []
    if mag["sim_time_s"]:
        for t, bx, by, bz in zip(mag["sim_time_s"], mag["mag_x_ut"], mag["mag_y_ut"], mag["mag_z_ut"]):
            roll = interpolate(truth["time"], truth["roll_rad"], t)
            pitch = interpolate(truth["time"], truth["pitch_rad"], t)
            # Tilt compensation: rotate the body field back through roll then
            # pitch to get its horizontal components.
            bx_h = (bx * math.cos(pitch)
                    + by * math.sin(roll) * math.sin(pitch)
                    + bz * math.cos(roll) * math.sin(pitch))
            by_h = by * math.cos(roll) - bz * math.sin(roll)
            lat = interpolate(truth["time"], truth["lat_deg"], t)
            lon = interpolate(truth["time"], truth["lon_deg"], t)
            alt = interpolate(truth["time"], truth["alt_m"], t)
            magnetic = math.degrees(math.atan2(-by_h, bx_h))
            mag_heading.append(wrap_360(magnetic + declination_deg(lat, lon, alt)))
        ax.plot(mag["sim_time_s"], mag_heading, linestyle="none", marker=".",
                markersize=MARKER_SENSOR, alpha=0.65, color=BARO,
                label="magnetic heading (MMC5983MA, tilt- and declination-corrected)")
        declination = declination_deg(truth["lat_deg"][0], truth["lon_deg"][0], truth["alt_m"][0])
        ax.annotate(f"dipole declination at the trim point: {declination:+.2f}°",
                    xy=(0.99, 0.05), xycoords="axes fraction", ha="right", fontsize=8, color=INK_2)

    ax.set_ylabel("heading [deg]")
    ax.set_title("Heading from three sources: the plant, the receiver and the magnetometer",
                 fontsize=10)
    add_headroom(ax, 0.30)
    legend(ax, loc="upper left", framed=True)

    if fixes["sim_time_s"]:
        ax_err.plot(fixes["sim_time_s"],
                    [wrap_180(c - interpolate(truth["time"], truth_yaw, t))
                     for t, c in zip(fixes["sim_time_s"], fixes["course_deg"])],
                    linestyle="none", marker=".", markersize=MARKER_SENSOR, alpha=0.5, color=GPS,
                    label="GPS course - truth yaw (course is not yaw: sideslip and drift)")
    if mag_heading:
        ax_err.plot(mag["sim_time_s"],
                    [wrap_180(h - interpolate(truth["time"], truth_yaw, t))
                     for t, h in zip(mag["sim_time_s"], mag_heading)],
                    linestyle="none", marker=".", markersize=MARKER_SENSOR, alpha=0.65, color=BARO,
                    label="magnetic heading - truth yaw (the sensor chain's own error)")
    ax_err.axhline(0.0, color=INK_2, linewidth=0.8, alpha=0.5)

    # Measure the offset the magnetometer chain carries, and check it against
    # the heading error it should produce. Both numbers come from this run:
    # the offset from (decoded - truth field), the horizontal field from the
    # truth field itself. If they disagree, the tilt compensation above is
    # wrong and the residual is the figure's, not the part's.
    if mag_heading and "b_x_ut" in truth:
        offsets = []
        for component in ("x", "y", "z"):
            deltas = [value - interpolate(truth["time"], truth[f"b_{component}_ut"], t)
                      for t, value in zip(mag["sim_time_s"], mag[f"mag_{component}_ut"])]
            offsets.append(sum(deltas) / len(deltas))
        horizontal_ut = math.hypot(*dipole_field_ned(truth["lat_deg"][0], truth["lon_deg"][0],
                                                     truth["alt_m"][0])[:2])
        predicted_deg = math.degrees(math.atan2(math.hypot(offsets[0], offsets[1]), horizontal_ut))
        residuals = [wrap_180(h - interpolate(truth["time"], truth_yaw, t))
                     for t, h in zip(mag["sim_time_s"], mag_heading)]
        observed_deg = sum(residuals) / len(residuals)
        ax_err.annotate(
            f"hard iron measured from this run: {offsets[0]:+.2f} / {offsets[1]:+.2f} / {offsets[2]:+.2f} µT\n"
            f"predicts {predicted_deg:.2f}° of heading error against a {horizontal_ut:.1f} µT "
            f"horizontal field; observed {abs(observed_deg):.2f}°",
            xy=(0.99, 0.06), xycoords="axes fraction", ha="right", fontsize=8, color=INK_2,
            bbox=ANNOTATION_CARD)

    ax_err.set_xlabel("simulation time [s]")
    ax_err.set_ylabel("heading residual [deg]")
    ax_err.set_title("Residual against truth yaw: the magnetometer's is a bias, the receiver's is noise",
                     fontsize=10)
    if fixes["sim_time_s"] or mag_heading:
        add_headroom(ax_err, 0.38)
        legend(ax_err, loc="upper left", framed=True)
    else:
        ax_err.annotate("neither stack produced a heading on this flight",
                        xy=(0.5, 0.5), xycoords="axes fraction", ha="center", fontsize=9, color=LIMIT)
    return save(fig, out, prefix, "heading_consistency", caption)


def smallest_step(values: list[float]) -> float:
    """The quantisation step a decoded series lands on, measured from it.

    Taken from the data rather than from the part's sensitivity constant on
    purpose: what this says is what actually survived the register counts, the
    packet format and the driver's conversion, which is the whole span the
    example exists to exercise. A series with fewer than two distinct levels
    returns 0.0.
    """
    levels = sorted({round(v, 9) for v in values})
    gaps = [b - a for a, b in zip(levels, levels[1:]) if b - a > 1e-12]
    return min(gaps) if gaps else 0.0


def plot_imu_specific_force(truth, imu, out: Path, prefix: str, caption: str) -> str:
    """Body specific force, truth against decoded counts, on two scales.

    The truth CSV's ``imu::f_*`` columns are the FMU's *input* variables — the
    zero-order-held value the part actually sampled, already one communication
    step behind the plant like every connection — so the decoded samples are
    compared against what the part was given rather than against plant truth
    at the same instant, and no realignment is needed.

    Body Z sits near -9.8 m/s^2 holding the aircraft up while X and Y live
    within a few hundredths of zero, so one axis cannot show both: drawn
    together, the lateral and forward channels collapse onto a line. Hence the
    split, with the X/Y panel scaled to its own data. What that panel then
    shows is the honest result -- on a trim flyout the *entire* variation in
    forward specific force is smaller than one accelerometer count.
    """
    fig, (ax_xy, ax_z) = new_figure(2, height=6.2)

    for axis, color, label in (("x", TRUTH, "X"), ("y", GPS, "Y")):
        ax_xy.plot(imu["sim_time_s"], imu[f"accel_{axis}_mps2"], linestyle="none", marker=".",
                   markersize=MARKER_IMU, alpha=0.25, color=color, label=f"decoded accelerometer {label}")
        ax_xy.plot(truth["time"], truth[f"f_{axis}_mps2"], color=color, linewidth=1.4,
                   label=f"truth specific force {label}")
    step = smallest_step(imu["accel_x_mps2"])
    span = max(truth["f_x_mps2"]) - min(truth["f_x_mps2"])
    if step > 0.0:
        ax_xy.annotate(f"one accelerometer count = {step:.4f} m/s²;\n"
                       f"the whole body-X excursion is {span:.4f} m/s² ({span / step:.2f} counts)",
                       xy=(0.99, 0.06), xycoords="axes fraction", ha="right", fontsize=8, color=INK_2,
                       bbox=ANNOTATION_CARD)
    ax_xy.set_ylabel("specific force [m/s²]")
    ax_xy.set_title("Body X and Y: the whole excursion is smaller than one count", fontsize=10)
    add_headroom(ax_xy, 0.34)
    legend(ax_xy, loc="upper left", framed=True)

    ax_z.plot(imu["sim_time_s"], imu["accel_z_mps2"], linestyle="none", marker=".",
              markersize=MARKER_IMU, alpha=0.25, color=BARO, label="decoded accelerometer Z")
    ax_z.plot(truth["time"], truth["f_z_mps2"], color=BARO, linewidth=1.4, label="truth specific force Z")
    ax_z.set_xlabel("simulation time [s]")
    ax_z.set_ylabel("specific force [m/s²]")
    ax_z.set_title("Body Z: the phugoid is resolvable here, at about nine counts", fontsize=10)
    add_headroom(ax_z, 0.30)
    legend(ax_z, loc="upper left", framed=True)
    return save(fig, out, prefix, "imu_specific_force", caption)


def plot_imu_body_rates(truth, imu, out: Path, prefix: str, caption: str) -> str:
    """Body rates: a phugoid that fits inside one gyroscope count.

    This figure looks broken and is not. The decoded samples lie on discrete
    horizontal bands because the gyroscope is a 16-bit part at +/-2000 deg/s,
    and on an open-loop trim flyout every body rate the aircraft actually has
    is *smaller than one of those counts*. The truth traces run through the
    middle of the zero band; the banding above and below it is the model's
    noise being quantised, not motion.

    It is worth a figure of its own because of what it implies downstream. A
    filter cannot integrate these rates into an attitude on this flight -- the
    signal is below the sensor's resolution, so dead reckoning has nothing to
    work with, and the heading must come from the magnetometer and the
    receiver instead. That is the case for the cross-sensor figures, made by
    the sensor that fails to make it.
    """
    fig, (ax,) = new_figure(1, height=4.6)
    for axis, color, label in (("x", TRUTH, "X"), ("y", GPS, "Y"), ("z", BARO, "Z")):
        ax.plot(imu["sim_time_s"], imu[f"gyro_{axis}_rad_s"], linestyle="none", marker=".",
                markersize=MARKER_IMU, alpha=0.28, color=color, label=f"decoded gyro {label}")
    for name, label, color in (("p_rad_s", "p (roll)", TRUTH), ("q_rad_s", "q (pitch)", GPS),
                               ("r_rad_s", "r (yaw)", BARO)):
        ax.plot(truth["time"], truth[name], color=color, linewidth=1.5, label=f"truth {label}")

    step = smallest_step(imu["gyro_x_rad_s"])
    if step > 0.0:
        peak = max(max(abs(v) for v in truth[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s"))
        ax.annotate(f"one gyro count = {step:.6f} rad/s ({math.degrees(step):.4f}°/s)\n"
                    f"largest truth rate on this flight = {math.degrees(peak):.4f}°/s "
                    f"({peak / step:.2f} counts)",
                    xy=(0.99, 0.04), xycoords="axes fraction", ha="right", fontsize=8, color=INK_2,
                    bbox=ANNOTATION_CARD)
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("body angular rate [rad/s]")
    ax.set_title("Body rates: the phugoid lives inside one gyroscope count")
    add_headroom(ax, 0.30)
    legend(ax, loc="upper left", framed=True)
    return save(fig, out, prefix, "imu_body_rates", caption)


def plot_sensor_envelopes(truth, fixes, baro, radalt, gps_gaps, radalt_gaps,
                          fix_epochs, radalt_epochs, out: Path, prefix: str, caption: str) -> str:
    """Each environment-dependent stack against the limit that bounds it.

    One figure, four panels, and the whole difference between the two
    check-cases: on case 11 every trace sits clear of its line for 200 s; on
    case 12 three of the four are on the wrong side of it from the first
    epoch, while every part keeps talking. That is the distinction the example
    is built to make — an envelope violation is not a silent bus.

    The GPS panel plots speed and altitude together on purpose. COCOM is an
    **AND**, and check-case 12 is the one flight in the NESC set that can tell
    that apart from an OR: at Mach 2 and 9.1 km the speed threshold is
    exceeded and the altitude one is not, so an OR receiver would have lost
    the fix to COCOM and this one does not. What takes the fix instead is the
    airborne platform model's own 500 m/s bound, which is a different limit
    with a different reason, drawn separately.
    """
    fig, axes = plt.subplots(4, 1, figsize=(8.4, 11.6), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in axes:
        style_axis(ax)
    ax_gps, ax_radalt, ax_press, ax_temp = axes

    # --- GPS ---------------------------------------------------------------
    # Two thresholds 15 m/s apart on a 500 m/s axis cannot carry a label each
    # without overprinting, so they share one: the lines are drawn separately
    # (they are different limits, from different authorities, with different
    # consequences) and named together in a single annotation.
    speed = [math.hypot(vn, ve) for vn, ve in zip(truth["v_north_m_s"], truth["v_east_m_s"])]
    shade_gaps(ax_gps, gps_gaps, "no fix reported")
    ax_gps.plot(truth["time"], speed, color=TRUTH, linewidth=1.4, label="true ground speed")
    ax_gps.axhline(UBLOX_DYNM8_SPEED_LIMIT_MS, color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.9)
    ax_gps.axhline(COCOM_SPEED_LIMIT_MS, color=LIMIT, linewidth=1.0, linestyle=":", alpha=0.9)
    ax_gps.set_ylim(0.0, max(COCOM_SPEED_LIMIT_MS * 1.18, max(speed) * 1.15))
    ax_gps.annotate("u-blox airborne <4g platform limit 500 m/s (dashed)\n"
                    "COCOM speed threshold 515 m/s (dotted), AND 18 000 m",
                    xy=(0.99, UBLOX_DYNM8_SPEED_LIMIT_MS), xycoords=("axes fraction", "data"),
                    xytext=(0, -6), textcoords="offset points", ha="right", va="top",
                    fontsize=7.5, color=LIMIT)
    ax_gps.set_ylabel("ground speed [m/s]")
    valid = len(fixes["sim_time_s"])
    ax_gps.set_title(f"GPS: {valid} of {fix_epochs} NAV-PVT epochs carried a fix", fontsize=10)
    legend(ax_gps, loc="center left", framed=True)

    # Altitude rides on a second scale because COCOM is an AND and the reader
    # has to be able to see both halves of it fail or hold at once. It is the
    # quiet series here -- grey, dotted, its own axis labelled as such -- so it
    # reads as the second condition rather than as a competing measurement.
    ax_alt = ax_gps.twinx()
    ax_alt.plot(truth["time"], truth["alt_m"], color=INK_2, linewidth=1.1, alpha=0.65, linestyle=":")
    ax_alt.axhline(COCOM_ALT_LIMIT_M, color=INK_2, linewidth=0.8, linestyle=":", alpha=0.45)
    ax_alt.set_ylim(0.0, COCOM_ALT_LIMIT_M * 1.25)
    ax_alt.set_ylabel("altitude [m] — grey dotted", fontsize=8, color=INK_2)
    ax_alt.tick_params(colors=INK_2, labelsize=8)
    ax_alt.spines["top"].set_visible(False)
    ax_alt.annotate(f"altitude holds {max(truth['alt_m']):.0f} m, "
                    f"{'below' if max(truth['alt_m']) < COCOM_ALT_LIMIT_M else 'above'} the 18 000 m half "
                    f"of the AND",
                    xy=(0.99, 0.04), xycoords="axes fraction", ha="right", va="bottom",
                    fontsize=7.5, color=INK_2)

    # --- Radar altimeter ---------------------------------------------------
    shade_gaps(ax_radalt, radalt_gaps, "no ground return")
    ax_radalt.plot(truth["time"], truth["h_agl_m"], color=TRUTH, linewidth=1.4,
                   label="height the part was driven with")
    if radalt["sim_time_s"]:
        ax_radalt.plot(radalt["sim_time_s"], radalt["range_m"], linestyle="none", marker=".",
                       markersize=2.0, alpha=0.4, color=RADALT, label="decoded range")
    ax_radalt.set_ylim(0.0, max(RADALT_MAX_RANGE_M * 1.15, max(truth["h_agl_m"]) * 1.15))
    limit_line(ax_radalt, RADALT_MAX_RANGE_M, "maximum tracking range, 6000 m")
    ax_radalt.set_ylabel("height AGL [m]")
    returns = len(radalt["sim_time_s"])
    ax_radalt.set_title(f"Radar altimeter: {returns} of {radalt_epochs} returns found the ground",
                        fontsize=10)
    legend(ax_radalt, loc="lower left", framed=True)

    # --- Barometer ---------------------------------------------------------
    if baro["sim_time_s"]:
        ax_press.plot(baro["sim_time_s"], baro["pressure_pa"], linestyle="none", marker=".",
                      markersize=MARKER_SENSOR, alpha=0.6, color=BARO, label="BMP390 compensated pressure")
    ax_press.plot(truth["time"], [isa_pressure_pa(h) for h in truth["alt_m"]], color=TRUTH,
                  linewidth=1.2, alpha=0.8, label="ISA pressure at true altitude")
    ax_press.set_ylim(min(BMP390_P_MIN_PA * 0.9, min(baro["pressure_pa"], default=BMP390_P_MIN_PA) * 0.95),
                      max(baro["pressure_pa"], default=BMP390_P_MAX_PA) * 1.08)
    limit_line(ax_press, BMP390_P_MIN_PA, "rated pressure floor, 300 hPa")
    below = sum(1 for p in baro["pressure_pa"] if p < BMP390_P_MIN_PA)
    ax_press.set_ylabel("static pressure [Pa]")
    ax_press.set_title(f"BMP390 pressure: {below} of {len(baro['pressure_pa'])} conversions "
                       f"below the rated floor", fontsize=10)
    legend(ax_press, loc="lower left", framed=True)

    # --- Die temperature ---------------------------------------------------
    if baro["sim_time_s"]:
        ax_temp.plot(baro["sim_time_s"], baro["temperature_c"], linestyle="none", marker=".",
                     markersize=MARKER_SENSOR, alpha=0.6, color=BARO, label="BMP390 die temperature")
    ax_temp.set_ylim(min(BMP390_T_MIN_C * 1.2, min(baro["temperature_c"], default=BMP390_T_MIN_C) - 5.0),
                     max(baro["temperature_c"], default=BMP390_T_MAX_C) + 5.0)
    limit_line(ax_temp, BMP390_T_MIN_C, "rated temperature floor, −40 °C")
    cold = sum(1 for t in baro["temperature_c"] if t < BMP390_T_MIN_C)
    ax_temp.set_xlabel("simulation time [s]")
    ax_temp.set_ylabel("die temperature [°C]")
    ax_temp.set_title(f"BMP390 die: {cold} of {len(baro['temperature_c'])} conversions below the rated floor",
                      fontsize=10)
    ax_temp.annotate("the FMU drives die temperature with ambient air",
                     xy=(0.99, 0.08), xycoords="axes fraction", ha="right", fontsize=7.5, color=INK_2)
    legend(ax_temp, loc="lower left", framed=True)

    return save(fig, out, prefix, "sensor_envelopes", caption)


def plot_nesc_envelope(truth, references: dict[str, dict[str, list[float]]],
                       out: Path, prefix: str, caption: str) -> str:
    """This run's drift against the published participants' own disagreement.

    The bar here is a sanity bound, not a precision test, and the figure is
    drawn to make that unmistakable: the shaded band is the *spread between
    the published solutions*, which on these check-cases is wide, and its
    width is the honest resolution of the comparison. Where the band is
    narrow the participants agree and a deviation means something; where it
    is wide they do not and it does not.

    The band ends before the plot does on purpose. ``sim_04`` and ``sim_05``
    stop at t = 180 s while ``sim_02`` runs to 200; a verifier that only
    compares times every reference covers silently leaves the last 20 s
    unchecked, so this figure shows the coverage instead of hiding it.
    """
    fig, (ax_alt, ax_yaw) = new_figure(2, height=6.4)
    truth_yaw = [wrap_360(math.degrees(y)) for y in truth["yaw_rad"]]

    for ax, truth_series, refs_key, ylabel in (
            (ax_alt, truth["alt_m"], "alt_m", "altitude [m]"),
            (ax_yaw, truth_yaw, "yaw_deg", "yaw [deg]")):
        # The participants' envelope, evaluated on this run's time grid and
        # only where at least two of them reach.
        times, lows, highs = [], [], []
        for t in truth["time"]:
            values = [interpolate(ref["time"], ref[refs_key], t)
                      for ref in references.values() if ref["time"][-1] >= t - 0.05]
            if len(values) >= 2:
                times.append(t)
                lows.append(min(values))
                highs.append(max(values))
        if times:
            ax.fill_between(times, lows, highs, color=OUTAGE, alpha=0.9, linewidth=0,
                            label="spread between published participants", zorder=1)
            if times[-1] < truth["time"][-1] - 0.5:
                ax.axvline(times[-1], color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.8)
                ax.annotate(f"only one reference reaches past t = {times[-1]:.0f} s",
                            xy=(times[-1], 0.97), xycoords=("data", "axes fraction"),
                            xytext=(-5, 0), textcoords="offset points", ha="right", va="top",
                            fontsize=7.5, color=LIMIT)
        # Distinct dash patterns per participant: they disagree with each other
        # by more than this run disagrees with any of them, so which line is
        # which is part of the content rather than a legend formality.
        dashes = [(4, 2), (1, 1.6), (6, 2, 1, 2)]
        for index, (name, ref) in enumerate(sorted(references.items())):
            ax.plot(ref["time"], ref[refs_key], linewidth=1.0, alpha=0.8, color=INK_2,
                    dashes=dashes[index % len(dashes)],
                    label=name if ax is ax_alt else None, zorder=2)
        ax.plot(truth["time"], truth_series, color=TRUTH, linewidth=1.6,
                label="this co-simulation" if ax is ax_alt else None, zorder=3)
        ax.set_ylabel(ylabel)

    ax_alt.set_title("Trim drift against the published NESC solutions", fontsize=10)
    # Whether the participants disagree about the *direction* of the heading
    # drift is a per-case fact, not a standing one: they do on check-case 11 and
    # do not on 12. Read it off the references rather than asserting it, or the
    # title is right on one figure and wrong on the other.
    drifts = [ref["yaw_deg"][-1] - ref["yaw_deg"][0] for ref in references.values() if len(ref["yaw_deg"]) > 1]
    split = any(a * b < 0.0 for a in drifts for b in drifts)
    ax_yaw.set_title("The participants disagree on the sign of the heading drift" if split
                     else "Heading drift, and the spread the participants leave", fontsize=10)
    ax_yaw.set_xlabel("simulation time [s]")
    add_headroom(ax_alt, 0.30)
    # Enough room for the coverage marker's label to clear the topmost
    # participant, which on both cases runs along the top of this panel.
    add_headroom(ax_yaw, 0.12)
    legend(ax_alt, loc="upper left", framed=True)
    return save(fig, out, prefix, "nesc_envelope", caption)


# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth", type=Path, default=Path("results/f16_truth.csv"))
    parser.add_argument("--fixes", type=Path, default=Path("results/gps_fixes.csv"))
    parser.add_argument("--imu", type=Path, default=Path("results/imu_samples.csv"))
    parser.add_argument("--baro", type=Path, default=Path("results/baro_samples.csv"))
    parser.add_argument("--mag", type=Path, default=Path("results/mag_samples.csv"))
    parser.add_argument("--radalt", type=Path, default=Path("results/radalt_samples.csv"))
    parser.add_argument("--reference", type=Path, action="append", default=[],
                        help="published NESC check-case CSV; repeatable, draws the participant envelope")
    parser.add_argument("--out", type=Path, default=Path("plots"))
    args = parser.parse_args()

    truth = read_truth(args.truth)
    config = read_run_config(args.truth)
    caption = config_caption(config)
    prefix = case_id(config)
    args.out.mkdir(parents=True, exist_ok=True)
    print(caption)

    empty: dict[str, list[float]] = {}
    fixes = read_samples(args.fixes) if args.fixes.exists() else empty
    baro = read_samples(args.baro) if args.baro.exists() else empty
    mag = read_samples(args.mag) if args.mag.exists() else empty
    radalt = read_samples(args.radalt) if args.radalt.exists() else empty
    imu = read_samples(args.imu) if args.imu.exists() else empty
    for series in (fixes, baro, mag, radalt, imu):
        series.setdefault("sim_time_s", [])

    gps_gaps = read_gaps(args.fixes)
    radalt_gaps = read_gaps(args.radalt)
    fix_epochs = count_rows(args.fixes)
    radalt_epochs = count_rows(args.radalt)

    if fix_epochs and not fixes["sim_time_s"]:
        print(f"none of the {fix_epochs} NAV-PVT epochs carried a fix "
              f"(the receiver reported throughout; its envelope refused every solution)")
    elif fix_epochs > len(fixes["sim_time_s"]):
        print(f"{fix_epochs - len(fixes['sim_time_s'])} of {fix_epochs} NAV-PVT epochs carried no fix")
    if radalt_epochs > len(radalt["sim_time_s"]):
        print(f"{radalt_epochs - len(radalt['sim_time_s'])} of {radalt_epochs} radar returns found no ground")

    written = [
        plot_ground_track(truth, fixes, gps_gaps, args.out, prefix, caption),
        plot_altitude_consistency(truth, fixes, baro, radalt, gps_gaps, radalt_gaps,
                                  args.out, prefix, caption),
        plot_heading_consistency(truth, fixes, mag, gps_gaps, args.out, prefix, caption),
        plot_sensor_envelopes(truth, fixes, baro, radalt, gps_gaps, radalt_gaps,
                              fix_epochs, radalt_epochs, args.out, prefix, caption),
    ]
    if imu["sim_time_s"]:
        written.append(plot_imu_specific_force(truth, imu, args.out, prefix, caption))
        written.append(plot_imu_body_rates(truth, imu, args.out, prefix, caption))

    references = {p.stem: read_reference(p) for p in args.reference if p.exists()}
    missing = [p for p in args.reference if not p.exists()]
    for p in missing:
        print(f"reference not found, skipping: {p}")
    if references:
        written.append(plot_nesc_envelope(truth, references, args.out, prefix, caption))
    else:
        print("no --reference given: skipping the participant-envelope figure")

    print(f"wrote {len(written)} figures to {args.out}/")
    for name in written:
        print(f"  {name}")


if __name__ == "__main__":
    main()
