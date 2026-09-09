# examples/f16_trim_ecos — F-16 trim flyout → five sensor FMUs → STM32 flight software, co-simulated with Ecos

NASA TM-2015-218675 **atmospheric check-case 11**: an F-16 trimmed for subsonic straight-and-level flight over
Kitty Hawk, NC at 10 013 ft and 565.685 ft/s true airspeed (335 KTAS) on a 45° heading, flown open-loop for
200 s.

```
┌──────────────────────┐   FMI 2.0 variables    ┌──────────────────────┐   UBX-NAV-PVT     ┌─────────────────────────┐
│    F16Plant.fmu      │  (Ecos connections)    │ hemerion_gps_fmu.fmu │   over UDP        │   f16_flight_computer   │
│  (Aetherion 6-DoF    ├───────────────────────>│ (u-blox M9N sim)     ├──────────────────>│  GpsDriver + UbxParser  │
│   plant, Radau IIA   │  lat, lon, alt,        └──────────────────────┘  127.0.0.1:5762   │                         │
│   on SE(3), with its │  NED velocity          ┌──────────────────────┐  SPI transfers    │  ImuSpiDriver + …       │
│   own trim solver)   │  p/q/r, specific force │ hemerion_imu_fmu.fmu │<──────────────────┤                         │
│                      ├───────────────────────>│ (MEMS IMU sim)       ├─── shared mem ───>│  Bmp390Driver + …       │
│                      │  altitude              ┌──────────────────────┐  I2C transactions │                         │
│                      ├───────────────────────>│ hemerion_bmp390_fmu  │<─── shared mem ──>│  Mmc5983maDriver + …    │
│                      │  position + attitude   ┌──────────────────────┐  I2C transactions │                         │
│                      ├───────────────────────>│ hemerion_mmc5983ma   │<─── shared mem ──>│  RadAltPacketParser +   │
│                      │  (host-computed)       └──────────────────────┘                   │  convert_raw_to_si      │
│                      │  altitude as AGL       ┌──────────────────────┐  raw-sample frames│                         │
│                      ├───────────────────────>│ hemerion_radalt_fmu  ├──────────────────>│  — the same modules/    │
└──────────────────────┘                        └──────────────────────┘  UDP 5765         │  sensors code the STM32 │
        │                                                                                  │  H743 firmware runs     │
        └────────────── f16_trim_cosim (Ecos master, fixed-step 10 Hz) ────────────────────┴─────────────────────────┘
```

## Why this check-case

It is the only scenario in the NESC set where **all four environment-dependent sensor stacks are valid at the
same time**:

| stack | why it works here | what the rocket scenario does |
|---|---|---|
| GPS | 172 m/s and 3.05 km clear every envelope limit; **2001 of 2001 epochs carry a fix** | loses the fix at t = 31 s |
| barometer | 10 013 ft is squarely in band | leaves the atmosphere |
| magnetometer | 36° N gives a strong, near-constant field (49.3–49.5 µT) | field magnitude swings 10% with altitude |
| radar altimeter | 3052 m AGL is inside the part's 6000 m range | out of range within seconds of the pad |

That is what makes this run the **cross-sensor consistency reference** — baro altitude against GPS altitude
against radar height, magnetic heading against GPS course, all on one trajectory. It is the case the EKF in
`modules/gnc` will be judged on.

`F16Plant.fmu` runs its own trim solver during initialisation and seeds the control deflections from it, so
nothing here closes a loop. The slow drift that follows is the check-case's content, not an error.

## The radar altimeter measures height above *ground*

There is no terrain model. `radalt::h_agl_m` is fed the plant's MSL altitude, correct to within the terrain
elevation at Kitty Hawk — a few metres of coastal North Carolina. Over mountains this connection would be
wrong, and it is wired bare rather than through a nominal offset so the assumption is visible in one line.

## Running

Two terminals, both in `build/examples-native/examples/f16_trim_ecos/`:

```
# terminal 1 — the "STM32" side
./f16_flight_computer

# terminal 2 — the co-simulation
./f16_trim_cosim
```

**Pace it if you care about the I2C parts.** The BMP390 and MMC5983MA are *polled*, and their sample rate is
the flight computer's loop rate, not the ODR programmed into their registers. Unpaced, the co-simulation
finishes far faster than wall clock and starves them: 200 s of flight yields about **37 baro conversions**.
With `./f16_trim_cosim --rtf 1` the same 20 s of flight yields **83**, a ~23× higher rate. The GPS, IMU and
radar altimeter are unaffected — their data queues in a socket or a FIFO.

## Verifying

`verify_trajectory.py` keeps two bars apart, because they have very different resolution:

```
python verify_trajectory.py \
  --truth results/f16_truth.csv \
  --reference <aetherion>/data/Atmos_11_TrimCheckSubsonicF16/Atmos_11_sim_02.csv \
  --reference <aetherion>/data/Atmos_11_TrimCheckSubsonicF16/Atmos_11_sim_04.csv \
  --reference <aetherion>/data/Atmos_11_TrimCheckSubsonicF16/Atmos_11_sim_05.csv \
  --aetherion <aetherion>/f16_s11_dt0.1.csv
```

**Against the NESC references** — a sanity bound, not a precision test. The published participant solutions
disagree with each other badly on this case: at t = 180 s `sim_02` has drifted −0.66° of heading and +44.7 ft
while `sim_04`/`sim_05` drift **+0.53° the other way** and hold altitude to a tenth of a foot. Two traps in
that data set are worth knowing:

* `sim_04` and `sim_05` stop at **t = 180 s**; only `sim_02` reaches 200 s. A verifier that skips any sample
  time not covered by *every* reference silently leaves the last 20 s unchecked and still prints OK. This one
  compares against whatever references reach each time and prints the coverage.
* Comparing the three files' *last rows* compares t = 200 against t = 180 and invents a ~2.4 km disagreement.
  At equal times the spread is ~181 m of ground track.

**Against an Aetherion run** — this one has resolution, and it is the question this example is actually
responsible for: does the co-simulation reproduce the plant? Current figures over 34 km flown:

```
worst ground-track difference 5.97 m (t = 200 s)
worst altitude difference     2.16 m (t = 200 s)
worst heading difference      5.52e-04 deg (t = 200 s)
```

Neither check looks only at altitude. On straight-and-level flight altitude is nearly constant, so an
altitude-only comparison passes for a vehicle that flew a circle at the right height.

### Known: this run drifts more than any published participant

Up to ~16 m in altitude and 0.44° in heading beyond the participants' own spread, in `sim_02`'s direction.
That is **not** the co-simulation's doing — it is a difference between Aetherion's FMU and Aetherion's own
standalone example, and it starts at t = 0:

| source | initial pitch (= trim angle of attack) |
|---|---|
| `F16Plant.fmu` | 2.656087° |
| Aetherion `F16SteadyFlight` | 2.654236° |
| NESC published initial condition | **2.643331°** |

The FMU computes the weight it trims against from **J2 gravity at the trim position**; the standalone example
uses a constant 9.80665 m/s². At 36° N that is a 0.073% difference in weight, and the resulting 1.85×10⁻³ deg
of alpha is 0.07% — the same figure. An open-loop trim flyout is a lightly damped phugoid (~70 s period, ~7 m
amplitude here), so a trim difference that small grows into metres over 200 s.

It is not integration error on either side: a 10× finer communication step (`--step 0.01`) changes those
figures *not at all*, and Aetherion's own step-size study is flat from 0.01 to 0.1 s. Note also that **neither**
Aetherion path reproduces the NESC trim point — both sit ~0.012° away, an order of magnitude further than they
are from each other. Resolving this needs an Aetherion-side change; it cannot be configured from here, because
the FMU derives its trim gravity internally from `lat0`/`lon0`/`alt0` and exposes no override.

## Building

Needs the `examples-native` preset and an **Aetherion ≥ 0.14.0** install for `F16Plant.fmu` (set
`AETHERION_ROOT` if it is not in a default location; `--f16` overrides the path at runtime).

```
cmake --preset examples-native
cmake --build build/examples-native
```

The version floor is a pair of port requirements that landed in different releases — 0.13.0 put geodetic
position into degrees, 0.14.0 added `out.specificForce_{x,y,z}_m_s2` — and configure tests for each
separately, reading `modelDescription.xml` out of the located FMU rather than trusting a version number that
`AETHERION_ROOT` could aim at a stale build tree.
