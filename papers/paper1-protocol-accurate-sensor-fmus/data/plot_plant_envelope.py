"""Figure: the reference run's altitude against the three NASA Scenario 17 check-case trajectories.

    python plot_plant_envelope.py --truth <results>/rocket_truth.csv \
        --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_0{4,5,6}.csv \
        --out ../figures

The comparison is on altitude above MSL, the one channel every simulator tabulates, with the
same alignment as the example's verify_trajectory.py (rounded time, only where all sources
have a sample).
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FT_TO_M = 0.3048
EVENTS = [(37.4, "stage 1 burnout"), (131.8, "stage 2 ignition"), (193.0, "stage 2 burnout")]


def read_truth(path: Path) -> dict[float, float]:
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    names = [n.strip() for n in rows[0]]
    it = names.index("time")
    ia = next(i for i, n in enumerate(names) if n.endswith("alt_m[REAL]") or n == "alt_m")
    return {round(float(r[it]), 4): float(r[ia]) for r in rows[1:] if len(r) == len(names)}


def read_reference(path: Path) -> dict[float, float]:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {round(float(r["time"]), 4): float(r["altitudeMsl_ft"]) * FT_TO_M for r in rows}


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--truth", type=Path, required=True)
    p.add_argument("--reference", type=Path, action="append", required=True)
    p.add_argument("--out", type=Path, default=Path("."))
    a = p.parse_args()

    ours = read_truth(a.truth)
    refs = {r.name.replace("Atmos_17_", "").replace(".csv", ""): read_reference(r) for r in a.reference}
    times = sorted(t for t in ours if all(t in r for r in refs.values()))

    fig, (ax, ax_d) = plt.subplots(2, 1, figsize=(6.4, 5.2), dpi=200, sharex=True,
                                   gridspec_kw={"height_ratios": [2.2, 1]})
    for name, r in refs.items():
        ax.plot(times, [r[t] / 1e3 for t in times], linewidth=1.0, alpha=0.9, label=f"NASA check case {name}")
    ax.plot(times, [ours[t] / 1e3 for t in times], color="black", linewidth=1.2, linestyle="--",
            label="this run (TwoStageRocket.fmu)")
    ax.set_ylabel("altitude above MSL [km]")
    for t_ev, label in EVENTS:
        for axis in (ax, ax_d):
            axis.axvline(t_ev, color="0.7", linewidth=0.8, linestyle=":")
        ax.annotate(label, xy=(t_ev, 5), xytext=(t_ev + 2, 5), fontsize=7, color="0.35")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(alpha=0.3)

    for name, r in refs.items():
        ax_d.plot(times, [(ours[t] - r[t]) / 1e3 for t in times], linewidth=1.0, label=f"minus {name}")
    ax_d.axhline(0.0, color="black", linewidth=0.6)
    ax_d.set_ylabel("this run minus\nreference [km]")
    ax_d.set_xlabel("time [s]")
    ax_d.grid(alpha=0.3)
    ax_d.legend(fontsize=7, loc="lower left", ncol=3)

    fig.tight_layout()
    a.out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(a.out / f"plant_envelope.{ext}")
    print("wrote", a.out / "plant_envelope.pdf")


if __name__ == "__main__":
    main()
