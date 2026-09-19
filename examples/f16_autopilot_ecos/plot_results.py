# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Plots the f16_autopilot_ecos co-simulation results for the Sphinx docs.

Unlike the trim example's script, this one reads **several runs at once**: the
four closed-loop check-cases differ only in which command is stepped, so the
figures that say anything are the ones that put them side by side. Point it at
one ``--run`` directory per case:

    python plot_results.py --run results/case13p1 --run results/case13p2 \\
                           --run results/case13p3 --run results/case13p4 \\
                           --nesc-data <aetherion>/data --out plots/

Each run directory is one ``f16_autopilot_cosim`` output: the truth log, its
``.config`` sidecar (which carries the commanded values and their step times,
so a response figure can draw its own cause) and the five decoded-sensor logs
``f16_flight_computer`` wrote beside it. ``--nesc-data`` is the root holding
``Atmos_13p1_SubsonicAltitudeChangeF16/`` and its three siblings; the
participant solutions are located from the check-case, so there is no
per-case ``--reference`` list to keep in step with the run list.

Figures written:

    case_responses.png        the four commanded quantities, one panel each
    hold_quantities.png       what each case had to hold while it manoeuvred
    case13p4_lateral_offset.png  the one signal in the loop that is feedback
    sensor_cadence.png        the two-rate sensor scheme, measured

Only matplotlib is required. Truth logs are Ecos ``csv_writer`` format
(", "-separated, "name[TYPE]" headers); the sensor logs are plain CSV; the
NESC references are the published check-case files, in feet, degrees and
slug/ft^3.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# The rocket and trim pages' palette, unchanged, so the three example pages
# read as one system.
TRUTH = "#2a78d6"
GPS = "#1baf7a"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d9d8d4"
SURFACE = "#fcfcfb"
OUTAGE = "#eceae5"
LIMIT = "#b4632a"
BARO = "#7b5bd6"
RADALT = "#c2185b"
# The magnetometer needs an ink of its own on the cadence figure, where it sits
# directly under the barometer and the two would otherwise read as one stack.
MAG = "#0f7b8a"

FT_PER_M = 1.0 / 0.3048
KT_MPS = 0.5144444
RHO_SL_KG_M3 = 1.225
# The standalone's flat-earth radius for lateral deviation -- deliberately its
# value rather than a better one, because the quantity plotted here is the one
# the controller was actually fed. verify_trajectory.py uses the same constant
# for the same reason.
LATDEV_RADIUS_M = 6_371_000.0

# What each case commands, and which quantity carries the answer. The labels
# are the figure's axis titles, so they name the *measured* quantity rather
# than the command.
CASES = {
    "13.1": {"dir": "Atmos_13p1_SubsonicAltitudeChangeF16", "stem": "Atmos_13p1",
             "quantity": "altitude", "unit": "ft", "title": "13.1 — altitude step"},
    "13.2": {"dir": "Atmos_13p2_SubsonicAirspeedChangeF16", "stem": "Atmos_13p2",
             "quantity": "keas", "unit": "kt", "title": "13.2 — airspeed step"},
    "13.3": {"dir": "Atmos_13p3_SubsonicHeadingChangeF16", "stem": "Atmos_13p3",
             "quantity": "heading", "unit": "deg", "title": "13.3 — heading step"},
    "13.4": {"dir": "Atmos_13p4_SubsonicLateralSideStepF16", "stem": "Atmos_13p4",
             "quantity": "latdev", "unit": "ft", "title": "13.4 — lateral side-step"},
}


# ---------------------------------------------------------------------------
# Readers
# ---------------------------------------------------------------------------
def read_truth(path: Path) -> dict[str, list[float]]:
    """Reads the host truth log into {short_name: column}.

    Same Ecos ``csv_writer`` shape as the trim example, and the same reason for
    stripping to the last ``::`` segment: the log carries variables from four
    instances, and only the prefix tells ``f16::out.alt_m`` from
    ``radalt::h_agl_m``. Here it also carries the ``ap::`` loop signals, so
    ``cmd.altCmd_ft`` and ``ctrl.el_deg`` arrive under their own names.
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
    """Reads a flight-computer log, dropping rows that carry no measurement."""
    if not path.exists():
        return {"sim_time_s": []}
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
    out = {k: [float(r[k]) for r in rows] for k in fields}
    out.setdefault("sim_time_s", [])
    return out


def read_config(truth_path: Path) -> dict[str, str]:
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
    """Reads a published NESC Atmos_13p* CSV into the quantities plotted here.

    KEAS is built from the file's own density and velocity with the
    standalone's constants (rho_SL = 1.225 kg/m^3, kt = 0.5144444 m/s), which
    is what ``verify_trajectory.py`` does, so the participant curve on the
    13.2 panel is the same number the verifier judges against.
    """
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    lookup = {name.strip().lower(): name for name in rows[0]}

    def col(key: str) -> str:
        return lookup[key]

    out: dict[str, list[float]] = {"time": [], "alt_ft": [], "yaw_deg": [], "keas": [], "latdev_ft": []}
    lat0 = lon0 = None
    for row in rows:
        lat = float(row[col("latitude_deg")])
        lon = float(row[col("longitude_deg")])
        if lat0 is None:
            lat0, lon0 = math.radians(lat), math.radians(lon)
        vn = float(row[col("fevelocity_ft_s_x")])
        ve = float(row[col("fevelocity_ft_s_y")])
        vd = float(row[col("fevelocity_ft_s_z")])
        rho = float(row[col("airdensity_slug_ft3")]) * 515.378818
        speed_mps = math.hypot(vn, ve, vd) * 0.3048

        out["time"].append(float(row[col("time")]))
        out["alt_ft"].append(float(row[col("altitudemsl_ft")]))
        out["yaw_deg"].append(float(row[col("eulerangle_deg_yaw")]))
        out["keas"].append((speed_mps / KT_MPS) * math.sqrt(rho / RHO_SL_KG_M3))
        dp_n = (math.radians(lat) - lat0) * LATDEV_RADIUS_M
        dp_e = (math.radians(lon) - lon0) * LATDEV_RADIUS_M * math.cos(lat0)
        psi0 = math.radians(45.0)
        out["latdev_ft"].append((-dp_n * math.sin(psi0) + dp_e * math.cos(psi0)) * FT_PER_M)
    return out


# ---------------------------------------------------------------------------
# Derived quantities -- one definition, used for both our run and the figures'
# annotations, so a panel cannot compare two different definitions of a word.
# ---------------------------------------------------------------------------
def truth_series(truth: dict[str, list[float]], quantity: str) -> list[float]:
    if quantity == "altitude":
        return [a * FT_PER_M for a in truth["alt_m"]]
    if quantity == "keas":
        return [(v / KT_MPS) * math.sqrt(r / RHO_SL_KG_M3)
                for v, r in zip(truth["vt_m_s"], truth["rho_kg_m3"])]
    if quantity == "heading":
        # Euler yaw, not course over ground. The command is a *course* command
        # (``cmd.baseChiCmd_deg``), and on this flight the two differ by the
        # sideslip -- about 0.03 deg at cut-off on 13.4. Yaw is what
        # verify_trajectory.py judges and what this page's tables quote, so
        # using it here keeps the figure, the assertion and the prose measuring
        # one quantity instead of three nearly-equal ones.
        return [math.degrees(y) % 360.0 for y in truth["yaw_rad"]]
    if quantity == "latdev":
        lat0 = math.radians(truth["lat_deg"][0])
        lon0 = math.radians(truth["lon_deg"][0])
        psi0 = math.radians(45.0)
        out = []
        for lat, lon in zip(truth["lat_deg"], truth["lon_deg"]):
            dp_n = (math.radians(lat) - lat0) * LATDEV_RADIUS_M
            dp_e = (math.radians(lon) - lon0) * LATDEV_RADIUS_M * math.cos(lat0)
            out.append((-dp_n * math.sin(psi0) + dp_e * math.cos(psi0)) * FT_PER_M)
        return out
    raise ValueError(quantity)


REFERENCE_COLUMN = {"altitude": "alt_ft", "keas": "keas", "heading": "yaw_deg", "latdev": "latdev_ft"}


def reference_series(ref: dict[str, list[float]], quantity: str) -> list[float]:
    """The participants' version of the quantity a case is judged on."""
    return ref[REFERENCE_COLUMN[quantity]]


def commanded(config: dict[str, str], quantity: str) -> tuple[float, float] | None:
    """The commanded value and the time it is stepped, from the run's sidecar.

    Returns None where the case does not command that quantity, which is what
    makes the hold panels drawable from the same table: a case's *hold*
    quantities are exactly the ones it does not command.
    """
    try:
        if quantity == "altitude":
            return float(config["alt_cmd_ft"]), float(config["alt_step_time_s"])
        if quantity == "keas":
            return float(config["keas_cmd_kt"]), float(config["keas_step_time_s"])
        if quantity == "heading":
            return float(config["chi_cmd_deg"]), float(config["chi_step_time_s"])
        if quantity == "latdev":
            return float(config["lat_step_ft"]), float(config["lat_step_time_s"])
    except (KeyError, ValueError):
        return None
    return None


def participant_label(stem: str) -> str:
    """"Atmos_13p1_sim_02" -> "sim_02".

    The check-case is already the panel title, so repeating it in three legend
    entries spends the legend's width on the one thing every entry shares.
    """
    return "sim_" + stem.rsplit("_", 1)[-1]


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


def legend(ax, loc: str = "best", framed: bool = True, fontsize: float = 7.5) -> None:
    leg = ax.legend(loc=loc, frameon=framed, fontsize=fontsize)
    if framed:
        leg.get_frame().set_facecolor(SURFACE)
        leg.get_frame().set_edgecolor(GRID)
        leg.get_frame().set_linewidth(0.6)
        leg.get_frame().set_alpha(0.92)
    for text in leg.get_texts():
        text.set_color(INK_2)


def add_headroom(ax, fraction: float = 0.3) -> None:
    low, high = ax.get_ylim()
    if high > low:
        ax.set_ylim(low, low + (high - low) * (1.0 + fraction))


def stamp(fig, caption: str) -> None:
    fig.tight_layout(rect=(0.0, 0.045, 1.0, 1.0))
    fig.text(0.5, 0.010, caption, ha="center", va="bottom", fontsize=7.5, color=INK_2)


def save(fig, out: Path, name: str, caption: str) -> str:
    stamp(fig, caption)
    fig.savefig(out / name, facecolor=SURFACE)
    plt.close(fig)
    return name


def run_caption(runs: dict[str, dict]) -> str:
    """One line covering every run on the figure.

    All four cases share a configuration by construction -- same trim point,
    same receiver settings, same loop rate -- so the stamp states it once and
    says so, rather than repeating four identical clauses. If a run ever
    differs, this notices and says that instead of quietly averaging it away.
    """
    if not runs:
        return "no runs"
    steps = {r["config"].get("step_s", "?") for r in runs.values()}
    rtfs = {r["config"].get("realtime_factor", "?") for r in runs.values()}
    platforms = {r["config"].get("dynamic_platform", "?") for r in runs.values()}
    parts = [f"NESC check-cases {', '.join(sorted(runs))}"]
    parts.append(f"{1.0 / float(next(iter(steps))):.0f} Hz closed loop, one-step transport delay"
                 if len(steps) == 1 and next(iter(steps)) not in ("?", "") else "mixed loop rates")
    if len(platforms) == 1:
        parts.append(f"u-blox dynModel {next(iter(platforms))}")
    if len(rtfs) == 1:
        parts.append("paced --rtf 1" if next(iter(rtfs)) == "1" else "unpaced")
    else:
        parts.append("MIXED PACING")
    return " | ".join(parts)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def plot_case_responses(runs: dict[str, dict], out: Path, caption: str) -> str:
    """The four commanded responses, one panel each, against the participants.

    This is the page's results table drawn rather than tabulated, and the
    reason it is one figure instead of four is that the four cases are one
    experiment: the same aircraft, the same controller, the same loop, with
    exactly one command moved. Reading them together is what shows that.

    Each panel carries its own cause. The commanded value and the time it
    steps come from the run's ``.config`` sidecar, so the dashed step is the
    signal the host actually wrote rather than a number retyped from the
    check-case description, and a panel whose response chases the wrong line
    would show it.

    The participants are drawn individually rather than as an envelope. On
    13.2 they disagree by 6 kt at their windows' ends -- the deceleration
    authority separates them -- and a shaded band would present that as one
    fuzzy answer instead of three sharp ones that differ.
    """
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)

    for ax, case in zip(axes.flat, sorted(CASES)):
        style_axis(ax)
        spec = CASES[case]
        if case not in runs:
            ax.annotate(f"no run supplied for check-case {case}", xy=(0.5, 0.5),
                        xycoords="axes fraction", ha="center", fontsize=9, color=LIMIT)
            ax.set_title(spec["title"], fontsize=10)
            continue

        run = runs[case]
        truth = run["truth"]
        series = truth_series(truth, spec["quantity"])

        # Ours goes down first and the participants over it. On 13.1 and 13.4
        # they agree to within the width of a line, and whichever is drawn last
        # is the only one visible -- so the thin dashes belong on top, where
        # agreement reads as dashes riding the trace rather than as two of the
        # four panels having lost their references.
        ax.plot(truth["time"], series, color=TRUTH, linewidth=2.0, label="this co-simulation", zorder=2)
        for index, (name, ref) in enumerate(sorted(run["references"].items())):
            ax.plot(ref["time"], reference_series(ref, spec["quantity"]),
                    color=INK_2, linewidth=0.9, alpha=0.85,
                    dashes=[(4, 2), (1, 1.6), (6, 2, 1, 2)][index % 3],
                    label=participant_label(name), zorder=3)

        step = commanded(run["config"], spec["quantity"])
        if step is not None:
            value, step_time = step
            ax.axhline(value, color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.9)
            ax.axvline(step_time, color=LIMIT, linewidth=0.8, linestyle=":", alpha=0.7)
            ax.annotate(f"commanded {value:g} {spec['unit']} at t = {step_time:g} s",
                        xy=(0.98, 0.06), xycoords="axes fraction", ha="right", fontsize=7.5, color=LIMIT)
        ax.annotate(f"{series[-1]:.1f} {spec['unit']} at cut-off",
                    xy=(0.98, 0.16), xycoords="axes fraction", ha="right", fontsize=8, color=INK_2)

        ax.set_title(spec["title"], fontsize=10)
        ax.set_xlabel("simulation time [s]")
        ax.set_ylabel(f"{spec['quantity']} [{spec['unit']}]")
        add_headroom(ax, 0.34)
        legend(ax, loc="upper left")

    return save(fig, out, "case_responses.png", caption)


def plot_hold_quantities(runs: dict[str, dict], out: Path, caption: str) -> str:
    """What each case had to hold while it manoeuvred.

    The commanded response is only half of a closed-loop check-case: an
    altitude step flown by rolling into a turn would sit on the commanded line
    and still be wrong. So each panel plots the two quantities that case does
    *not* command, as deviations from where they started, on one axis scaled
    to whichever moved most.

    Which quantities those are is not hard-coded -- it is read off the same
    sidecar the response figure draws its steps from, as "everything this run
    did not step". That keeps the two figures from disagreeing about what a
    case commanded.
    """
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    colors = {"altitude": TRUTH, "keas": GPS, "heading": BARO, "latdev": RADALT}

    for ax, case in zip(axes.flat, sorted(CASES)):
        style_axis(ax)
        spec = CASES[case]
        ax.set_title(spec["title"], fontsize=10)
        if case not in runs:
            ax.annotate(f"no run supplied for check-case {case}", xy=(0.5, 0.5),
                        xycoords="axes fraction", ha="center", fontsize=9, color=LIMIT)
            continue

        run = runs[case]
        truth = run["truth"]
        commanded_quantity = spec["quantity"]
        # Everything this case did not step. For 13.4 that includes course,
        # which matters: a side-step flown by turning and staying turned would
        # reach the commanded offset and still be wrong, so verify_trajectory.py
        # judges "held course" there too. Keeping the rule uniform keeps this
        # figure and the verifier asserting the same set.
        held = [q for q in ("altitude", "keas", "heading") if q != commanded_quantity]

        for quantity in held:
            series = truth_series(truth, quantity)
            start = series[0]
            unit = {"altitude": "ft", "keas": "kt", "heading": "deg"}[quantity]
            worst = max(series, key=lambda v: abs(v - start)) - start
            ax.plot(truth["time"], [v - start for v in series], color=colors[quantity], linewidth=1.4,
                    label=f"{quantity} − its trim value [{unit}]   (worst {worst:+.1f})")
        ax.axhline(0.0, color=INK_2, linewidth=0.8, alpha=0.5)

        step = commanded(run["config"], commanded_quantity)
        if step is not None:
            ax.axvline(step[1], color=LIMIT, linewidth=0.8, linestyle=":", alpha=0.7)
            ax.annotate(f"{commanded_quantity} stepped here", xy=(step[1], 0.02),
                        xycoords=("data", "axes fraction"), xytext=(4, 0), textcoords="offset points",
                        fontsize=7.5, color=LIMIT)
        ax.set_xlabel("simulation time [s]")
        ax.set_ylabel("departure from trim (units per series)")
        add_headroom(ax, 0.34)
        legend(ax, loc="upper left")

    return save(fig, out, "hold_quantities.png", caption)


def plot_lateral_offset(run: dict, out: Path, caption: str) -> str:
    """Check-case 13.4's ``cmd.latOffset_ft``: the one signal that is feedback.

    Every other command in this example is a setpoint the host writes and
    forgets. This one the host has to *compute every step*, from where the
    aircraft is now and where it began, because the controller is fed
    ``deviation − commanded_step`` rather than a target: the plant cannot
    publish it, since it depends on the flight's own starting point.

    The figure separates the three things that are easy to conflate. The
    aircraft's actual deviation from the original courseline is what the
    manoeuvre achieves; the commanded step is what it was asked for; and
    ``cmd.latOffset_ft`` — read back from the truth log, not recomputed — is
    what the controller was handed, which is the difference of the two. That
    the third is the difference of the first two is the check that the host's
    feedback path is wired the way the standalone's is.
    """
    truth = run["truth"]
    deviation = truth_series(truth, "latdev")
    step = commanded(run["config"], "latdev")

    fig, (ax, ax_fb) = plt.subplots(2, 1, figsize=(9.0, 6.6), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    style_axis(ax)
    style_axis(ax_fb)

    ax.plot(truth["time"], deviation, color=TRUTH, linewidth=2.0, label="this co-simulation", zorder=2)
    for index, (name, ref) in enumerate(sorted(run["references"].items())):
        ax.plot(ref["time"], ref["latdev_ft"], color=INK_2, linewidth=0.9, alpha=0.85,
                dashes=[(4, 2), (1, 1.6), (6, 2, 1, 2)][index % 3],
                label=participant_label(name), zorder=3)
    if step is not None:
        value, step_time = step
        ax.axhline(value, color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.9)
        ax.axvline(step_time, color=LIMIT, linewidth=0.8, linestyle=":", alpha=0.7)
        # 60 s is where the verifier judges this case, and why: past it the
        # flat-earth formula drifts enough that it measures itself.
        judge = 60.0
        if truth["time"][-1] >= judge:
            at_judge = min(zip(truth["time"], deviation), key=lambda p: abs(p[0] - judge))[1]
            ax.plot([judge], [at_judge], marker="o", markersize=5, color=LIMIT, linestyle="none", zorder=5)
            ax.annotate(f"{at_judge:.0f} ft at t = 60 s, where the verifier judges it\n"
                        f"(participants read 1934–1992 ft here)",
                        xy=(judge, at_judge), xytext=(10, -26), textcoords="offset points",
                        fontsize=7.5, color=LIMIT)
    ax.set_ylabel("deviation from the original courseline [ft]")
    ax.set_title("Where the aircraft actually went", fontsize=10)
    add_headroom(ax, 0.30)
    legend(ax, loc="upper left")

    ax_fb.plot(truth["time"], truth["cmd.latOffset_ft"], color=RADALT, linewidth=1.5,
               label="cmd.latOffset_ft — what the controller was handed", zorder=3)
    if step is not None:
        value, step_time = step
        ax_fb.plot(truth["time"],
                   [d - (value if t >= step_time else 0.0) for t, d in zip(truth["time"], deviation)],
                   color=INK_2, linewidth=2.6, alpha=0.35,
                   label="deviation − commanded step, recomputed here", zorder=2)
    ax_fb.axhline(0.0, color=INK_2, linewidth=0.8, alpha=0.5)
    ax_fb.set_xlabel("simulation time [s]")
    ax_fb.set_ylabel("controller input [ft]")
    ax_fb.set_title("The feedback the host computes every step — the two traces should coincide",
                    fontsize=10)
    add_headroom(ax_fb, 0.30)
    legend(ax_fb, loc="upper left")
    return save(fig, out, "case13p4_lateral_offset.png", caption)


def plot_sensor_cadence(run: dict, case: str, out: Path, caption: str) -> str:
    """The two-rate sensor scheme, measured rather than asserted.

    The plant and the autopilot run at 100 Hz because that is what the control
    loop needs; the sensors do not follow them down, and Ecos' per-instance
    step-size hint is how they are held apart. This figure is the evidence
    that the hints did what they were supposed to: sample interval against
    time, one row per stack, with the intended period drawn across it.

    The two I2C parts are the interesting rows. They carry no hint at all —
    their rate is whatever the flight computer's poll loop achieves, because a
    BMP390 data register is not a FIFO — so their intervals are a scatter
    rather than a line, and the scatter is the reason this page's other
    figures come from paced runs.
    """
    stacks = [
        ("GPS (UBX-NAV-PVT)", run["gps"], GPS, 0.1, "0.1 s hint → 10 Hz"),
        ("radar altimeter", run["radalt"], RADALT, 1.0 / 25.0, "1/25 s hint → 25 Hz"),
        ("IMU (SPI burst)", run["imu"], TRUTH, 0.01, "base step → 100 Hz"),
        ("BMP390 (I2C poll)", run["baro"], BARO, None, "no hint: the flight computer's loop rate"),
        ("MMC5983MA (I2C poll)", run["mag"], MAG, None, "no hint: the flight computer's loop rate"),
    ]
    fig, axes = plt.subplots(len(stacks), 1, figsize=(8.6, 9.4), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)

    for ax, (name, samples, color, period, note) in zip(axes, stacks):
        style_axis(ax)
        times = samples["sim_time_s"]
        if len(times) < 2:
            ax.annotate(f"{name}: fewer than two samples", xy=(0.5, 0.5), xycoords="axes fraction",
                        ha="center", fontsize=9, color=LIMIT)
            continue
        intervals = [(b - a) * 1000.0 for a, b in zip(times, times[1:])]
        ax.plot(times[1:], intervals, linestyle="none", marker=".", markersize=2.0,
                alpha=0.4, color=color)
        measured = sum(intervals) / len(intervals)
        if period is not None:
            ax.axhline(period * 1000.0, color=LIMIT, linewidth=1.0, linestyle="--", alpha=0.9)
        ax.set_ylabel("interval [ms]", fontsize=8)
        ax.set_title(f"{name}: {len(times)} samples, mean interval {measured:.2f} ms — {note}",
                     fontsize=9)
        # A rate that is exactly its hint draws a flat line, and a flat line
        # auto-scaled fills the panel with numerical dust. Pin the scale to
        # something the eye can read as "constant" instead.
        spread = max(intervals) - min(intervals)
        if spread < 0.05 * measured:
            ax.set_ylim(measured * 0.8, measured * 1.2)
    axes[-1].set_xlabel("simulation time [s]")
    return save(fig, out, "sensor_cadence.png", caption)


# ---------------------------------------------------------------------------
def load_run(directory: Path, nesc_data: Path | None) -> tuple[str, dict] | None:
    truth_path = directory / "f16_truth.csv"
    if not truth_path.exists():
        print(f"no f16_truth.csv in {directory}, skipping")
        return None
    config = read_config(truth_path)
    case = config.get("check_case")
    if case not in CASES:
        print(f"{directory}: unrecognised check_case {case!r} in the .config sidecar, skipping")
        return None

    references: dict[str, dict[str, list[float]]] = {}
    if nesc_data is not None:
        folder = nesc_data / CASES[case]["dir"]
        for participant in ("02", "04", "05"):
            path = folder / f"{CASES[case]['stem']}_sim_{participant}.csv"
            if path.exists():
                references[path.stem] = read_reference(path)
        if not references:
            print(f"case {case}: no participant CSVs under {folder}")

    return case, {
        "truth": read_truth(truth_path),
        "config": config,
        "references": references,
        "gps": read_samples(directory / "gps_fixes.csv"),
        "imu": read_samples(directory / "imu_samples.csv"),
        "baro": read_samples(directory / "baro_samples.csv"),
        "mag": read_samples(directory / "mag_samples.csv"),
        "radalt": read_samples(directory / "radalt_samples.csv"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", type=Path, action="append", default=[],
                        help="a run directory (truth log, .config sidecar and sensor logs); repeatable")
    parser.add_argument("--nesc-data", type=Path, default=None,
                        help="root holding the Atmos_13p*/ published participant solutions")
    parser.add_argument("--out", type=Path, default=Path("plots"))
    args = parser.parse_args()

    if not args.run:
        raise SystemExit("error: at least one --run directory is required")

    runs: dict[str, dict] = {}
    for directory in args.run:
        loaded = load_run(directory, args.nesc_data)
        if loaded is not None:
            case, data = loaded
            runs[case] = data
    if not runs:
        raise SystemExit("error: no usable runs")

    args.out.mkdir(parents=True, exist_ok=True)
    caption = run_caption(runs)
    print(caption)
    missing = [c for c in CASES if c not in runs]
    if missing:
        print(f"no run for check-case(s) {', '.join(missing)}; those panels will say so")

    written = [
        plot_case_responses(runs, args.out, caption),
        plot_hold_quantities(runs, args.out, caption),
    ]
    if "13.4" in runs:
        written.append(plot_lateral_offset(runs["13.4"], args.out, caption))
    else:
        print("no 13.4 run: skipping the lateral-offset figure, which is that case's alone")

    # The cadence figure needs one run, and wants the longest: a 60 s window
    # shows the same constant intervals in a quarter of the samples.
    cadence_case = max(runs, key=lambda c: runs[c]["truth"]["time"][-1])
    written.append(plot_sensor_cadence(runs[cadence_case], cadence_case, args.out,
                                       caption + f" | cadence from check-case {cadence_case}"))

    print(f"wrote {len(written)} figures to {args.out}/")
    for name in written:
        print(f"  {name}")


if __name__ == "__main__":
    main()
