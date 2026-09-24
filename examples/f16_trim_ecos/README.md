# examples/f16_trim_ecos — F-16 trim flyouts → five sensor FMUs → STM32 flight software, co-simulated with Ecos

NASA TM-2015-218675's two open-loop **trim flyouts**, selected with `--case`:

| `--case` | check-case | condition | what the sensors make of it |
|---|---|---|---|
| `11` (default) | subsonic trim flyout | 10 013 ft, 565.685 ft/s TAS (335 KTAS), Mach 0.52 | every stack in band for all 200 s — the cross-sensor reference |
| `12` | supersonic trim flyout | 30 013 ft, 2000 ft/s TAS, Mach 2.01 | no GPS fix, no radar return, a barometer below its rated floor |

Both trim the same aircraft over Kitty Hawk, NC on a 45° heading and fly it open-loop for 200 s; the whole
difference between them is an altitude and an airspeed. The wiring, both executables and the verifier are
identical across the two — which is the point of flying both.

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

## Why these check-cases

Case 11 is the only scenario in the NESC set where **all four environment-dependent sensor stacks are valid
at the same time**; case 12 takes three of them out of band at once, over the same wiring:

| stack | case 11 (10 013 ft, 172 m/s) | case 12 (30 013 ft, 610 m/s) | the rocket scenario |
|---|---|---|---|
| GPS | clears every envelope limit; **2001 of 2001 epochs carry a fix** | past dynModel 8's 500 m/s from t = 0; **0 of 2001 epochs carry a fix** | loses the fix at t = 31 s |
| barometer | 10 013 ft is squarely in band | most of the flight below the part's 300 hPa rated floor | leaves the atmosphere |
| magnetometer | 36° N gives a strong, near-constant field (49.3–49.5 µT) | still in band — the one survivor | field magnitude swings 10% with altitude |
| radar altimeter | 3052 m AGL is inside the part's 6000 m range | 9148 m: **every return reports no ground** | out of range within seconds of the pad |

That is what makes the case-11 run the **cross-sensor consistency reference** — baro altitude against GPS
altitude against radar height, magnetic heading against GPS course, all on one trajectory. It is the case the
EKF in `modules/gnc` will be judged on. Case 12 is the same EKF's degraded-mode counterpart: the flight where
fusion has to fall back on the IMU and the magnetometer, because everything else is out of its envelope while
still talking. See [Case 12: the flight the sensors cannot follow](#case-12-the-flight-the-sensors-cannot-follow).

`F16Plant.fmu` runs its own trim solver during initialisation and seeds the control deflections from it, so
nothing here closes a loop. The slow drift that follows is the check-case's content, not an error.

## Case 12: the flight the sensors cannot follow

`./f16_trim_cosim --case 12` flies the same aircraft at Mach 2.01 and 30 013 ft. Nothing about the wiring or
the flight computer changes; measured over a full 200 s run:

* **GPS, default `--dyn-model 8`: 0 of 2001 epochs carry a fix.** 610 m/s exceeds the airborne platform
  model's 500 m/s limit from the very first epoch, so a Mach 2 aircraft with an airborne-configured u-blox is
  flying without GPS. The receiver keeps emitting one NAV-PVT per epoch throughout — `gnssFixOK` clear, no
  position in the payload — and the flight computer's outage bookkeeping, insurance on case 11, spans the
  whole flight here.
* **GPS, `--dyn-model -1`: the fix comes back — 300 of 300 epochs over a 30 s run**, at 609.8 m/s and
  9214 m. That is COCOM's AND holding: the cut-off needs 18 000 m *and* 515 m/s, and 9.2 km is half the
  altitude threshold. This is the one flight in the NESC set that tells COCOM's AND apart from an OR — the
  rocket crosses both thresholds within seconds of each other and cannot.
* **Radar altimeter: 6003 of 6003 returns report no ground.** 9148 m is past the part's 6000 m tracking
  range. The part keeps reporting — a sensor that is working and has nothing to say — and `valid` is a column
  in the CSV rather than a filter precisely so this record isn't an empty file.
* **BMP390: 253 of 286 conversions below the part's 300 hPa rated pressure floor** (minimum 290.9 hPa), with
  the die at **−45.9 °C, below its −40 °C rating** (the FMU uses ambient air as die temperature). The part
  model saturates only at the ADC rails, so the conversions read plausibly — as they do on the real silicon,
  which offers no out-of-rating indication either. What the FMU adds is a side channel the part cannot: its
  `conversions` / `conversions_out_of_rating` FMI outputs count the violation (the host prints them in its
  summary), and each envelope crossing is debug-logged once with the offending value.
* **The magnetometer stays in band**, and with the IMU it is all the sensor fusion has left.
* **The plant's phugoid is bigger here**: +218 m peak altitude excursion and +0.47° of heading over the
  window. The published participants split the same way they do on case 11 — `sim_02` climbs +140 m through
  its phugoid while `sim_04`/`sim_05` hold altitude to 0.3 m.

Against Aetherion's standalone `F16SupersonicTrim` (≥ 0.14.1) the run verifies exactly as case 11 does:
0.491 m constant initial offset (the same five-decimal Kitty Hawk rounding), 0.000 m worst altitude
difference, 1.4×10⁻⁷ deg worst heading difference — and the NESC sanity bound passes.

## The radar altimeter measures height above *ground*

There is no terrain model. `radalt::h_agl_m` is fed the plant's MSL altitude, correct to within the terrain
elevation at Kitty Hawk — a few metres of coastal North Carolina. Over mountains this connection would be
wrong, and it is wired bare rather than through a nominal offset so the assumption is visible in one line.
On case 12 the same connection feeds it 9148 m, past the part's tracking range, so the approximation never
matters there — the part is out of range against any plausible terrain.

## Running

Two terminals, both in `build/examples-native/examples/f16_trim_ecos/`:

```
# terminal 1 — the "STM32" side
./f16_flight_computer

# terminal 2 — the co-simulation
./f16_trim_cosim              # check-case 11 (default)
./f16_trim_cosim --case 12    # check-case 12; add --dyn-model -1 to give the receiver its fix back
./f16_trim_cosim --gps-errors correlated --gps-seed 1 --stop 1000   # the realistic receiver, seeded,
                              # over a window long enough to show its 100 s correlation time
```

The flight computer takes no case option: it decodes whatever arrives, unchanged between the two.

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

For a `--case 12` run, substitute `Atmos_12_TrimCheckSupersonicF16/Atmos_12_sim_0{2,4,5}.csv` and a standalone
`F16SupersonicTrim` output. The standalone references are regenerated from an Aetherion build ≥ 0.14.1 (it
must include `TrimWeight.h` — see below) with, from the Aetherion root:

```
<build>/src/Examples/F16SteadyFlight/F16SteadyFlight --timeStep 0.1 --endTime 200 \
    --writeInterval 1 --inputFileName unused --outputFileName f16_s11_dt0.1.csv
```

(`F16SupersonicTrim` for case 12; `--inputFileName` is required by the CLI but ignored.)

**Against the NESC references** — a sanity bound, not a precision test. The published participant solutions
disagree with each other badly on both cases: on case 11 at t = 180 s `sim_02` has drifted −0.66° of heading
and +44.7 ft while `sim_04`/`sim_05` drift **+0.53° the other way** and hold altitude to a tenth of a foot;
on case 12 `sim_02` climbs +140 m through its phugoid while the other two hold altitude to 0.3 m. Two traps
in that data set are worth knowing:

* `sim_04` and `sim_05` stop at **t = 180 s**; only `sim_02` reaches 200 s. A verifier that skips any sample
  time not covered by *every* reference silently leaves the last 20 s unchecked and still prints OK. This one
  compares against whatever references reach each time and prints the coverage.
* Comparing the three files' *last rows* compares t = 200 against t = 180 and invents a ~2.4 km disagreement.
  At equal times the spread is ~181 m of ground track.

**Against an Aetherion run** — this one has resolution, and it is the question this example is actually
responsible for: does the co-simulation reproduce the plant? Against Aetherion ≥ 0.14.1's standalone
`F16SteadyFlight`, over 34 km flown:

```
initial position offset       0.491 m (identical initial conditions give 0.000)
worst ground-track difference 0.494 m (t = 180 s)
worst altitude difference     0.003 m (t = 40 s)
worst heading difference      1.88e-07 deg (t = 40 s)
```

The 0.491 m is not a divergence: it is present at t = 0 and moves by 3 mm over the whole run. The standalone
hardcodes Kitty Hawk as `36.01917 / -75.67444`, five decimals, where the published condition this example
uses has six — and that rounding predicts −0.3336 m north and −0.3597 m east, exactly the measured offset.
Run `./f16_trim_cosim --lat0 36.01917 --lon0 -75.67444` and the two drivers agree to **3 mm of ground track,
3 mm of altitude and 2×10⁻⁷ deg of heading**. That is why the verifier prints the initial offset on its own
line: an initial-condition difference and a divergence look identical in a "worst difference" figure.

The default tolerances (1 m / 0.05 m / 10⁻⁵ deg) are set to catch the bug described below, which each one
fails by an order of magnitude or more; the looser defaults they replaced let it through on two of three checks.

Neither check looks only at altitude. On straight-and-level flight altitude is nearly constant, so an
altitude-only comparison passes for a vehicle that flew a circle at the right height.

### Fixed in Aetherion 0.14.1: two drivers trimmed against different gravity

Before 0.14.1 this example and Aetherion's standalone example disagreed by 5.97 m of ground track, 2.16 m of
altitude and 5.5×10⁻⁴ deg of heading at t = 200 s, and the difference was already there at t = 0 in the trim
solution. `F16Plant.fmu` built the weight it trims against from **J2 gravity at the trim position**; the
standalone examples used the sea-level constant 9.80665 m/s². At Kitty Hawk J2 gives 9.81108 m/s², **0.045%
above** g₀, and the heavier trim came out at 1.85×10⁻³ deg more alpha. An open-loop trim flyout is a lightly
damped phugoid (~70 s period, ~7 m amplitude here), so a trim difference that small grows into metres.

It was never integration error: a 10× finer communication step (`--step 0.01`) changed those figures not at
all, and Aetherion's own step-size study is flat from 0.01 to 0.1 s. 0.14.1 moved every F-16 standalone
example onto the FMU's J2 weight through a shared `Aetherion/FlightDynamics/Trim/TrimWeight.h`. The FMU's own
trim did not change — its initial pitch is identical to 4×10⁻¹¹ deg between 0.14.0 and 0.14.1 — which is why
this example's version floor stays at 0.14.0: nothing it consumes moved. Comparing against a standalone run
**older than 0.14.1** reproduces the old disagreement, and the verifier fails it on all three checks.

### Open: this run drifts more than any published participant

Up to ~16 m in altitude and 0.44° in heading beyond the participants' own spread, in `sim_02`'s direction — and
still after 0.14.1, because that release made Aetherion self-consistent, not consistent with the check-case.
Both Aetherion paths now trim to the same point, and it is not the published one:

| source | initial pitch (= trim angle of attack) |
|---|---|
| `F16Plant.fmu` and `F16SteadyFlight`, 0.14.1 | 2.656087° |
| NESC published initial condition | **2.643331°** |

It is no longer a weight question: Aetherion's trim weight is 20 509.3 lbf against NASA's 20 509.4 lbf.

**Leading suspect, not yet a finding.** `TrimSolver` balances lift against weight with no Earth-rotation or
curvature terms, while the integrator flies the aircraft over a rotating round Earth. For level flight at this
point those terms relieve a measurable share of the lift the wing must make:

| term | lift relief |
|---|---|
| centrifugal (Earth rotation) | 0.2265% of g |
| Eötvös, 2Ω·v_E·cos(lat) | 0.1466% |
| path curvature, v²/R | 0.0475% |
| **total** | **0.4206%** |

Using the alpha-per-weight sensitivity measured from Aetherion's own two trims (1.85×10⁻³ deg per 0.0452%),
that predicts −0.0172° against the actual −0.0128° gap: the right sign and order, consistent with this run
*climbing* where the best-behaved participants hold altitude — but 135% of the gap, and no subset of the terms
closes it cleanly either. So it is a place to look on the Aetherion side, not an explanation.

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
