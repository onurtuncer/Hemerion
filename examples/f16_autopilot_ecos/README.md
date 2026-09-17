# examples/f16_autopilot_ecos — F-16 autopilot closed loop → five sensor FMUs → STM32 flight software, co-simulated with Ecos

NASA TM-2015-218675's four **closed-loop autopilot check-cases**, selected with `--case`. All four start from
check-case 11's trim condition (Kitty Hawk, 10 013 ft, 335 KTAS, heading 45°) and step exactly one command:

| `--case` | check-case | command | window |
|---|---|---|---|
| `13.1` (default) | subsonic altitude change | +100 ft (10 013 → 10 113 ft) at t = 5 s | 60 s |
| `13.2` | subsonic airspeed change | KEAS → 277 kt (from trim ≈ 288) at t = 5 s | 60 s |
| `13.3` | subsonic heading change | course 45 → 60° at t = 15 s | 240 s |
| `13.4` | subsonic lateral side-step | 2000 ft right of the courseline at t = 20 s | 240 s |

Where [`f16_trim_ecos`](../f16_trim_ecos/) flies the plant open-loop, this example closes the loop:

```
┌──────────────────────┐  out.* (11 signals)   ┌──────────────────────┐
│     F16Plant.fmu     ├──────────────────────>│   F16Autopilot.fmu   │   cmd.altCmd_ft, keasCmd_kt,
│  (Aetherion 6-DoF    │                       │  (DML LQR SAS +      │<─ baseChiCmd_deg, latOffset_ft
│   plant + trim)      │<──────────────────────┤   autopilot)         │   — this host owns the schedule
│                      │  ctrl.* (4 surfaces)  └──────────────────────┘
│                      │
│                      ├──> hemerion_gps_fmu ──UBX-NAV-PVT/UDP──┐
│                      ├──> hemerion_imu_fmu ──SPI/shared mem───┤     f16_flight_computer
│                      ├──> hemerion_bmp390_fmu ──I2C/shm───────┼──>  (examples/f16_trim_ecos's
│                      ├──> hemerion_mmc5983ma ──I2C/shm────────┤      executable, reused unchanged)
│                      └──> hemerion_radalt_fmu ──frames/UDP────┘
└──────────────────────┘
        └──────── f16_autopilot_cosim (Ecos master, 0.01 s base step) ────────┘
```

The five sensor FMUs ride the same truth with the same wiring as the trim example, and are consumed by **the
same `f16_flight_computer` executable** — this example deliberately has no flight computer of its own. What
changes is the flight: the maneuvers excite exactly the states a straight-and-level flyout leaves
unobservable (13.3 and 13.4 especially), which is what makes these runs the sensor-fusion exercise the EKF in
`modules/gnc` needs, with every stack in band throughout.

## Why the communication step is 0.01 s, not 0.1 s

Aetherion's standalone 13.x examples evaluate their DML LQR as a **zero-order-hold discrete controller at the
integration step rate** — feedback read from the *current* state, surfaces applied over that same step, no
transport delay — at a recommended 0.02 s; their headers warn that dt = 0.1 diverges ("LQR plant is stiff").

An Ecos co-simulation cannot reproduce zero delay: it is a Jacobi master — every instance steps, *then*
connections transfer — so the plant flies [t, t+h] on surfaces the autopilot computed from the state at
t−h, one communication step late, whatever the stepping order. What the host can choose is h. A one-step
delay at h costs about the loop phase of zero-delay ZOH at 2h, so **h = 0.01 s puts this co-simulation at
the standalone's own recommended cadence**. The control law itself is stateless (a pure LQR gain evaluation,
no integrators), so cadence and delay are the *only* differences between the two drivers of the same DML —
and the measured responses below land on the published solutions.

**The sensors do not follow the loop down to 0.01 s.** Ecos' fixed-step algorithm takes a per-instance
step-size hint, stepping that instance every Nth base step with dt = N × base:

| instance | hint | effective rate |
|---|---|---|
| plant, autopilot | — (base) | 100 Hz loop |
| GPS | 0.1 s | one NAV-PVT per step of its own = **10 Hz**, as in the trim example |
| radar altimeter | 1/25 s | 25 Hz — at base rate its `max(1, lround(dt·rate))` framing would clamp to 100 Hz |
| IMU | — (base) | `lround(0.01 × 100)` = exactly one frame per base step = 100 Hz |
| BMP390, MMC5983MA | — (base) | whatever ODR the flight computer programs; stepping rate is irrelevant |

## `cmd.latOffset_ft` is feedback, not a setpoint

The standalone computes the aircraft's lateral deviation from the original courseline every step (flat earth
about the initial position, R = 6 371 000 m) and feeds the controller `deviation − commanded_step`. This host
does the same, writing the input after every step exactly like the magnetometer field — it is the one signal
in the loop the plant cannot publish, because it depends on where the flight *began*. Cases 13.1–13.3 write a
constant 0.

## Start-up

The plant's `ctrl.*` inputs start at 0, the plant does not publish its trim deflections, and Ecos' init
rounds hand the plant whatever the autopilot computed during its own initialisation. The `trimPoint`
parameter set therefore seeds the autopilot's `fb.*`/`cmd.*` inputs with the trim condition, so its
init-time output is the LQR's own answer at trim — approximately the trim deflections, the same property the
standalone relies on when it engages its controller at t = 0. The initial KEAS command is then refined after
init from the plant's actual density and airspeed (the standalone's formula and constants), and printed:
`holding trim KEAS 287.981 kt` against the references' 287.92–287.98.

## Running

Two terminals, from `build/examples-native/examples/`:

```
# terminal 1 — the "STM32" side (the trim example's executable, reused)
./f16_trim_ecos/f16_flight_computer

# terminal 2 — the closed loop
./f16_autopilot_ecos/f16_autopilot_cosim --case 13.3
```

The same pacing caveat as the trim example applies to the polled I2C parts: run with `--rtf 1` if the BMP390
and MMC5983MA sample counts matter.

## Measured responses (unpaced, default configuration)

| case | commanded | response at cut-off | the published participants |
|---|---|---|---|
| 13.1 | 10 113 ft | **10 113.5 ft**, ≈3 ft overshoot at t = 10 s, settled by t = 20 s | end within 0.6 ft of command |
| 13.2 | 277 kt | **277.0 kt**, altitude dip to 10 006.7 ft | sim_02 reaches 277.0; sim_04/05 still ≈283 kt at their windows' ends; sim_02 dips to 10 006.4 ft |
| 13.3 | course 60° | **60.04°** at cut-off (55.1° five seconds into the turn, 59.9° by t = 30 s), altitude held to 10 013.5 ft — and 240 s of integrated ground track lands **1.5 ft** from `sim_05`'s lateral deviation (32 071.6 vs 32 070.1 ft) | participants end 59.92–60.01° |
| 13.4 | 2000 ft right | **1981.5 ft at t = 60 s**, **2000.1 ft at cut-off**, course back to 45.07°, altitude held | participants read 1934–1992 ft at t = 60 s |

## Verifying

`verify_trajectory.py` makes two kinds of check — the NESC-reference envelope (as in the trim example), and
**the commanded response**, which the envelope alone cannot carry: the participants' altitude spread plus a
sensible floor is wider than 13.1's 100 ft step, so a run that never climbed could still sit inside the
envelope. Each case asserts its own response with bounds taken from what the participants themselves achieve
(13.2's band accepts 275–285.5 kt, because "reached 277 exactly" would fail two of the three published
solutions; 13.4 is judged at t = 60 s, past which the flat-earth deviation formula itself drifts). The hold
quantities are asserted too — an altitude case must hold heading and KEAS — which is what makes these runs
fusion-grade references: several quantities pinned at once.

```
python verify_trajectory.py --truth results/f16_truth.csv --case 13.1 \
  --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_02.csv \
  --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_04.csv \
  --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_05.csv
```

The participant windows are unequal again (sim_02/sim_04 stop at 20–60 s where sim_05 runs to 60–239.9 s);
the verifier compares against whatever references reach each sample time and prints the coverage.

## Building

Needs the `examples-native` preset and an **Aetherion ≥ 0.14.1** install for both `F16Plant.fmu` and
`F16Autopilot.fmu` (set `AETHERION_ROOT` if not in a default location; `--f16` / `--ap` override at
runtime). Configure reads the ports this example binds out of both archives — the plant's `out.*`/`ctrl.*`
and the autopilot's `cmd.*`/`fb.*`/`ctrl.*` — rather than trusting a version string.

```
cmake --preset examples-native
cmake --build build/examples-native
```

Note the 0.14.1 autopilot FMU exposes **no parameters**: `circlePoleSW` is baked off, which is why NESC
cases 15 and 16 (the pole/equator circumnavigations) are not rows in this example's `--case` table.
