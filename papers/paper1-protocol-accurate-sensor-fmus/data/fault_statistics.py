"""Scores the three injected driver faults against the truth log.

    python fault_statistics.py <results_dir> [<results_dir> ...]

For each directory: gyroscope and accelerometer scale against truth (what `imu-range`
corrupts), and the angle between the decoded magnetic field and the field the FMU was
handed (what `mag-skip-conditioning` and `mag-leave-reset` corrupt).
"""
import csv, math, sys
from pathlib import Path

H = 0.1


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    head = [h.strip() for h in rows[0]]
    cols = {h: [] for h in head}
    for r in rows[1:]:
        if len(r) != len(head):
            continue
        for h, v in zip(head, r):
            v = v.strip()
            cols[h].append(float(v) if v != "" else float("nan"))
    return cols


def median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


for arg in sys.argv[1:]:
    R = Path(arg)
    truth = load(R / "rocket_truth.csv")
    T = {k.split("::")[-1].split("[")[0]: v for k, v in truth.items()}
    t = T["time"]
    print(f"\n== {R.name}")

    m = load(R / "imu_samples.csv")
    gt = {"x": "out.p_rad_s", "y": "out.q_rad_s", "z": "out.r_rad_s"}
    # Scale factor by least squares over samples where the truth signal is large enough to carry it.
    for kind, cols, thresh in (("gyro", [f"gyro_{a}_rad_s" for a in "xyz"], 0.02),
                               ("accel", [f"accel_{a}_mps2" for a in "xyz"], 20.0)):
        num = den = 0.0
        n = 0
        for k in range(len(m["sample_index"])):
            j = int(math.ceil(m["part_time_s"][k] / H - 1e-6)) - 1
            if j < 0 or j >= len(t):
                continue
            for a, c in zip("xyz", cols):
                tv = T[gt[a]][j] if kind == "gyro" else T[f"f_{a}_mps2"][j]
                if abs(tv) >= thresh:
                    num += m[c][k] * tv
                    den += tv * tv
                    n += 1
        print(f"  {kind}: decoded / truth scale = {num / den:.4f} over {n} axis-samples with |truth| >= {thresh}")

    g2 = load(R / "mag_samples.csv")
    angles, mags, tm = [], [], []
    for k in range(len(g2["sample_index"])):
        j = int(math.ceil(g2["imu_part_time_s"][k] / H - 1e-6)) - 1
        if j < 1 or j >= len(t):
            continue
        d = [g2[f"mag_{a}_ut"][k] for a in "xyz"]
        u = [T[f"b_{a}_ut"][j] for a in "xyz"]
        nd, nu = math.sqrt(sum(x * x for x in d)), math.sqrt(sum(x * x for x in u))
        if nd == 0 or nu == 0:
            continue
        c = sum(x * y for x, y in zip(d, u)) / (nd * nu)
        angles.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
        mags.append(nd)
        tm.append(nu)
    print(f"  mag: {len(angles)} samples; angle decoded-vs-truth median {median(angles):.1f} deg "
          f"(min {min(angles):.1f}, max {max(angles):.1f}); |B| decoded {min(mags):.1f}..{max(mags):.1f} uT, "
          f"truth {min(tm):.1f}..{max(tm):.1f} uT")
    cfg = R / "mag_samples.config"
    if cfg.exists():
        print("  " + " ".join(l.strip() for l in open(cfg) if l.startswith("bridge")))
