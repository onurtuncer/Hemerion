"""Numbers for the paper, from one results directory. Mirrors plot_results.py's alignment."""
import csv, math, sys
from pathlib import Path

R = Path(sys.argv[1])
H = 0.1


def load(name):
    with open(R / name, newline="") as f:
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


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def mean(xs):
    return sum(xs) / len(xs)


def std(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


truth = load("rocket_truth.csv")
T = {k.split("::")[-1].split("[")[0]: v for k, v in truth.items()}
t = T["time"]
print(f"truth rows {len(t)}  t_end {t[-1]:.1f}  max alt {max(T['out.alt_m']):.0f} m")
staged = next(tt for tt, s in zip(t, T["out.staged"]) if s > 0.5)
print(f"staging at t={staged:.1f}")
speed = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(T["out.v_north_m_s"], T["out.v_east_m_s"], T["out.v_down_m_s"])]
t515 = next(tt for tt, s in zip(t, speed) if s > 515.0)
i18 = next(i for i, a in enumerate(T["out.alt_m"]) if a > 18000.0)
print(f"speed>515 at t={t515:.1f} (alt {T['out.alt_m'][t.index(t515)]:.0f} m);  alt>18km at t={t[i18]:.1f} (speed {speed[i18]:.0f} m/s)")
print(f"downrange lon at end {T['out.lon_deg'][-1]:.3f} deg  -> {T['out.lon_deg'][-1] * 111.32:.0f} km east")

# ---- GPS ----
g = load("gps_fixes.csv")
n_epochs = len(g["fix_index"])
valid = [i for i in range(n_epochs) if not math.isnan(g["latitude_deg"][i])]
print(f"\nGPS epochs {n_epochs}  with fix {len(valid)}  without {n_epochs - len(valid)}")
print(f"last fix t={g['nominal_time_s'][valid[-1]]:.1f}  first no-fix t={g['nominal_time_s'][valid[-1] + 1]:.1f}")
by_time = {round(tt, 3): i for i, tt in enumerate(t)}
MPD = 111_319.49
hor, ver, dn, de = [], [], [], []
for i in valid:
    j = by_time.get(round(g["nominal_time_s"][i] - H, 3))
    if j is None:
        continue
    lat_t, lon_t = T["out.lat_deg"][j], T["out.lon_deg"][j]
    n = (g["latitude_deg"][i] - lat_t) * MPD
    e = (g["longitude_deg"][i] - lon_t) * MPD * math.cos(math.radians(lat_t))
    dn.append(n); de.append(e)
    hor.append(math.hypot(n, e))
    ver.append(g["altitude_m"][i] - T["out.alt_m"][j])
N = len(hor)
print(f"compared {N} fixes: horizontal RMS {rms(hor):.3f} m (expect 2.121), vertical RMS {rms(ver):.3f} m (expect 3.0)")
print(f"  north RMS {rms(dn):.3f}  east RMS {rms(de):.3f}  (expect 1.5 each);  SE of an RMS at N={N}: {100 / math.sqrt(2 * N):.1f} %")
print(f"  max decoded alt {max(g['altitude_m'][i] for i in valid):.1f} m  max speed {max(g['ground_speed_mps'][i] for i in valid):.1f} m/s")

# ---- IMU ----
m = load("imu_samples.csv")
n = len(m["sample_index"])
print(f"\nIMU samples {n}  first part_time {m['part_time_s'][0]:.3f}  last {m['part_time_s'][-1]:.3f}")
res = {a: [] for a in "xyz"}
gres = {a: [] for a in "xyz"}
gt = {"x": "out.p_rad_s", "y": "out.q_rad_s", "z": "out.r_rad_s"}
for k in range(n):
    j = int(math.ceil(m["part_time_s"][k] / H - 1e-6)) - 1
    if j < 0 or j >= len(t):
        continue
    for a in "xyz":
        res[a].append(m[f"accel_{a}_mps2"][k] - T[f"f_{a}_mps2"][j])
    # gyro truth: the rate held over the step is the plant output at t_j
    for a in "xyz":
        gres[a].append(m[f"gyro_{a}_rad_s"][k] - T[gt[a]][j])
for a in "xyz":
    print(f"  accel {a}: bias {mean(res[a]):+.4f}  std {std(res[a]):.4f} m/s2 (inject 0.05 white, 0.02 bias 1-sigma)")
for a in "xyz":
    print(f"  gyro  {a}: bias {mean(gres[a]):+.5f}  std {std(gres[a]):.5f} rad/s (inject 0.002 white, 0.001 bias 1-sigma)")
fx = T["f_x_mps2"]
def at(tt):
    return fx[by_time[round(tt, 3)]]
print(f"  truth f_x: t=0.1 {at(0.1):.1f}  max stage1 {max(fx[: by_time[round(staged, 3)]]):.1f}  coast t=80 {at(80.0):.3f}  t=132.5 {at(132.5):.1f}  max {max(fx):.1f}  t=199 {at(199.0):.3f}")
print(f"  decoded max |f| {max(math.sqrt(x*x+y*y+z*z) for x,y,z in zip(m['accel_x_mps2'],m['accel_y_mps2'],m['accel_z_mps2'])):.2f}")

# ---- BARO ----
b = load("baro_samples.csv")
nb = len(b["sample_index"])
p = b["pressure_pa"]
print(f"\nBARO conversions {nb}  first p {p[0]:.1f} Pa at part_time {b['part_time_s'][0]:.2f}  min p {min(p):.1f}  first T {b['temperature_c'][0]:.2f}  min T {min(b['temperature_c']):.2f}")
floor = min(p)
tf = next(tt for tt, pp in zip(b["part_time_s"], p) if pp < floor + 5.0)
print(f"  reaches floor (within 5 Pa) at part_time {tf:.1f} s")
def isa_alt(pp):
    if pp > 22632.06:
        return 44330.77 * (1.0 - (pp / 101325.0) ** 0.190263)
    return 11000.0 + 6341.62 * math.log(22632.06 / pp)
print(f"  ISA altitude at floor {isa_alt(floor) / 1000:.2f} km")
errs = []
for tt, pp in zip(b["part_time_s"], p):
    j = int(math.ceil(tt / H - 1e-6)) - 1
    if 0 <= j < len(t) and T["out.alt_m"][j] < 15000.0:
        errs.append(isa_alt(pp) - T["out.alt_m"][j])
if errs:
    print(f"  pressure-altitude minus truth below 15 km: mean {mean(errs):+.1f} m  std {std(errs):.1f} m  (n={len(errs)})")

# ---- MAG ----
g2 = load("mag_samples.csv")
nm = len(g2["sample_index"])
cfg = dict(l.strip().split("=") for l in open(R / "mag_samples.config") if "=" in l)
S = float(cfg["lsb_per_microtesla"])
off = [float(cfg[f"bridge_offset_{a}_lsb"]) / S for a in "xyz"]
print(f"\nMAG measurements {nm}  bridge offset LSB {cfg['bridge_offset_x_lsb']}/{cfg['bridge_offset_y_lsb']}/{cfg['bridge_offset_z_lsb']}  = {off[0]:+.2f}/{off[1]:+.2f}/{off[2]:+.2f} uT  |offset| {math.sqrt(sum(o*o for o in off)):.1f}")
mags = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(g2["mag_x_ut"], g2["mag_y_ut"], g2["mag_z_ut"])]
print(f"  decoded |B|: first {mags[0]:.2f}  last {mags[-1]:.2f}  min {min(mags):.2f}  max {max(mags):.2f}")
tm = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(T["b_x_ut"], T["b_y_ut"], T["b_z_ut"])]
print(f"  truth  |B|: t=0.1 {tm[1]:.2f}  end {tm[-1]:.2f}   truth body at t=0.1: {T['b_x_ut'][1]:+.2f} {T['b_y_ut'][1]:+.2f} {T['b_z_ut'][1]:+.2f}  end: {T['b_x_ut'][-1]:+.2f} {T['b_y_ut'][-1]:+.2f} {T['b_z_ut'][-1]:+.2f}")
gap = {a: [] for a in "xyz"}
for k in range(nm):
    j = int(math.ceil(g2["imu_part_time_s"][k] / H - 1e-6)) - 1
    if 0 <= j < len(t):
        for a in "xyz":
            gap[a].append(g2[f"mag_{a}_ut"][k] - T[f"b_{a}_ut"][j])
for a in "xyz":
    print(f"  decoded - truth {a}: mean {mean(gap[a]):+.3f}  std {std(gap[a]):.3f} uT  (hard iron 1 uT 1-sigma; white 0.04)")
angles = []
for x, y, z in zip(g2["mag_x_ut"], g2["mag_y_ut"], g2["mag_z_ut"]):
    ux, uy, uz = x + off[0], y + off[1], z + off[2]
    c = (x * ux + y * uy + z * uz) / (math.sqrt(x * x + y * y + z * z) * math.sqrt(ux * ux + uy * uy + uz * uz))
    angles.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
um = [math.sqrt((x + off[0]) ** 2 + (y + off[1]) ** 2 + (z + off[2]) ** 2) for x, y, z in zip(g2["mag_x_ut"], g2["mag_y_ut"], g2["mag_z_ut"])]
print(f"  uncalibrated counterfactual: direction error {min(angles):.1f}..{max(angles):.1f} deg (first {angles[0]:.1f}, last {angles[-1]:.1f});  |B| would read {min(um):.1f}..{max(um):.1f} uT")
