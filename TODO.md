# TODO

Work that is written but not yet closed out. Each entry says what is already
verified, so nobody repeats it, and what specifically remains.

---

## F-16 NESC check-cases — on `main` (PR #36, merge 392e5c6)

Second and third virtually instrumented cases before the EKF in `modules/gnc`. Needs an Aetherion
**≥ 0.14.0** install (0.14.1 is what was verified against; `F16Plant`/`F16Autopilot`/`TwoStageRocket`
FMUs under `<prefix>/share/Aetherion/fmu/`, `AETHERION_ROOT` if not in a default location). Decided with
the user: **split by topology** — open-loop trim flyouts (11, 12) in `examples/f16_trim_ecos` via
`--case`, the four closed-loop autopilot cases (13.1–13.4) in a new `examples/f16_autopilot_ecos` —
and the autopilot example **reuses the `f16_flight_computer` executable** rather than copying it.

* **Case 11 — done and verified.** All five sensor stacks in band. Against Aetherion's standalone
  `F16SteadyFlight` (≥ 0.14.1): 0.491 m constant initial offset (the standalone rounds Kitty Hawk to
  5 decimals), 0.003 m altitude, 1.9e-7 deg heading; from identical ICs, 3 mm / 3 mm / 2e-7 deg.
  NESC sanity bound passes.

* **Case 12 — code done and verified; docs not yet updated.** `f16_trim_cosim --case 12` runs, and
  verifies against `F16SupersonicTrim` (0.491 m offset / 0.000 m / 1.4e-7 deg) and the NESC bound.
  Measured sensor behaviour, none of it yet in the README:
  - GPS with the default `--dyn-model 8`: **0 of 2001** epochs carry a fix — 610 m/s exceeds the
    airborne platform's 500 m/s limit from t = 0.
  - GPS with `--dyn-model -1`: **300 of 300** fixes (30 s run) at 609.8 m/s and 9214 m — COCOM's AND
    holds; the one NESC case that tells AND from OR.
  - Radar altimeter: 6003 of 6003 returns "no ground return" (9148 m > 6000 m range).
  - BMP390: **253 of 286** conversions below the part's 300 hPa rated floor (min 290.9 hPa), and the
    die at **−45.9 °C, below the −40 °C rating** (the FMU uses ambient air as die temperature). The
    part model only saturates at the ADC rails; it models neither rating.
  - Plant drifts +218 m peak / +0.47 deg (sim_02: +140 m; sim_04/05 hold altitude to 0.3 m).

  Docs closed out: the example README carries the case-12 section with the figures above and the
  verification/regeneration commands, both file headers and the in-band comments cover both cases,
  the `examples/README.md` row names both, and the Sphinx page exists (`doc/f16_trim_ecos_cosim.rst`,
  in the toctree; parse-checked only — no Doxygen on this machine, so the full Sphinx build is CI's).

  The BMP390 rating question is decided and done: the FMU now counts out-of-rating conversions on two
  diagnostic FMI outputs (`conversions` / `conversions_out_of_rating`), debug-logs each envelope
  crossing once with the offending value, and changes nothing on the I2C side — the words stay
  plausible, as on real silicon. `f16_trim_cosim` prints the counters in its summary. Verified: unit
  test on the rating flags, FMI-level instantiate/read smoke via fmpy, full ctest green — and
  **re-run end to end against the installed Aetherion 0.14.1** (`C:/Program Files/Aetherion`, FMU
  ports confirmed). The paced 200 s `--case 12 --rtf 1` run reproduced the whole record: 0 of 2001
  fixes, 6003 of 6003 radar no-returns, min pressure 291.1 hPa / die −46.1 °C (recorded 290.9 /
  −45.9 — per-run turn-on bias), mag in band, drift +217.7 m / +0.470°, NESC verifier passes with the
  known beyond-the-spread note; `--dyn-model -1 --stop 30` again gives 300 of 300 fixes at 609.8 m/s.
  The new counter reads **9999 of 9999 out of rating** — 100 %, not the flight computer's ~90 %
  below-the-pressure-floor fraction (541 of 581 reads this run), because the counter also honours the
  −40 °C temperature rating, which the whole flight violates. Two footnotes: FMU `debugLog` lines
  (the edge logs included) never reach Ecos' console — Ecos instantiates FMUs with FMI logging off,
  a pre-existing property, the counters carry the result; and the `--aetherion` standalone bar was
  not re-measured — the install ships no example executables and the local Aetherion checkout is
  v0.11.3-era, so regenerating `f16_s12` still needs a ≥ 0.14.1 build tree.

* **Regenerating the Aetherion standalone references** (needed for `verify_trajectory.py
  --aetherion`, must be a build that includes 0.14.1's `TrimWeight.h`), from the Aetherion root:
  `<build>/src/Examples/F16SteadyFlight/F16SteadyFlight.exe --timeStep 0.1 --endTime 200
  --writeInterval 1 --inputFileName unused --outputFileName f16_s11.csv` (and `F16SupersonicTrim`
  for case 12). `--inputFileName` is required by the CLI but ignored.

* **`examples/f16_autopilot_ecos`, cases 13.1–13.4 — built and verified (2026-09-17).** All four
  cases run, verify against the NESC references, and pass the new commanded-response assertions:
  13.1 settles at 10 113.5 ft (participants end within 0.6 ft of 10 113); 13.2 reaches 277.0 kt with
  the same 10 006.7 ft altitude dip `sim_02` shows; 13.3 ends at 60.04° and its 240 s of integrated
  ground track lands 1.5 ft from `sim_05`'s lateral deviation (32 071.6 vs 32 070.1 ft); 13.4 reads
  1981.5 ft at t = 60 s (participants 1934–1992) and 2000.1 ft at cut-off. The paced
  flight-computer smoke confirms the two-rate sensors: exactly 10 Hz GPS, exactly 25 Hz radalt,
  ≈100 Hz IMU, both I2C parts polling, all fixes valid. The investigation record below is what the
  design implements; it stays here because it documents *why* the example looks the way it does.

  `F16Plant` + `F16Autopilot` + the five sensor FMUs; plant `out.*` → autopilot `fb.*` (11
  connections: alt, vt, rho, alpha, beta, roll/pitch/yaw, p/q/r — every source verified present on
  the 0.14.1 plant), autopilot `ctrl.*` → plant `ctrl.*` (4; the plant has all four inputs). The
  autopilot FMU exposes exactly 20 variables and **no parameters** — `circlePoleSW` is baked off, so
  cases 15/16 stay blocked.

  **The three questions, answered (2026-09-17, against the 0.14.1 install + the Ecos source the
  examples pin):**
  - *The standalone* (`F16AltitudeChangeSimulator.h`, reused by all four 13.x examples) is, in its
    own words, "a zero-order hold (ZOH) discrete controller at the integration step rate, matching
    the NASA reference implementation": per step it extracts feedback from the *current* state,
    evaluates the DML LQR, applies the surfaces, then integrates that same step — **no transport
    delay**. Recommended `--timeStep 0.02` (50 Hz); the header warns **dt = 0.1 diverges** ("LQR
    plant is stiff"). `DAVEMLControlModel::evaluate()` is `const` — the LQR is **stateless** (no
    integrators), so the controller has no dt-dependence of its own; cadence and delay are the only
    knobs.
  - *Ecos decimation exists*: `ss.add_model(name, path, stepSizeHint)`; `fixed_step_algorithm` steps
    that instance every N-th base step (N = ceil(hint/base)) with dt = N·base, applying its
    sets/gets only when it steps. But the co-simulation is **Jacobi**: all instances step, *then*
    connections transfer (`simulation.cpp`), so the one-communication-step transport delay is
    structural — the plant flies [t, t+h] on controls computed from state at t−h, and the
    `parallel` flag does not change it. A one-step delay at h costs about the loop phase of ZOH at
    2h, so **base step 0.01 s puts the co-sim at the standalone's recommended 0.02 cadence**; that
    is the design point (plant + autopilot + IMU + both I2C parts at base rate).
  - *The GPS FMU needs no change*: it emits exactly one NAV-PVT per `do_step`, dt-agnostic, stamped
    at `currentTime()+dt` — `stepSizeHint 0.1` on the 0.01 base gives a correctly-stamped 10 Hz
    stream. **The radalt is the one that bites**: it emits `max(1, lround(dt·rate))` frames, so at
    dt = 0.01 and 25 Hz it clamps to 1/step = 100 Hz — give it `stepSizeHint 0.04`. IMU at base is
    exactly 1 frame/step (100 Hz); BMP390/MMC5983MA convert at programmed ODR, dt-independent.

  **Command schedules, from the standalone sources** (`src/Examples/F16{Altitude,Airspeed,Heading,
  LateralSideStep}Change*.cpp` — all four start from the case-11 trim; initial KEAS = (vt/0.5144444)
  · sqrt(rho_trim/1.225) from the plant's own density and airspeed):
  - 13.1: `altCmd` 10 013 → **10 113 ft at t = 5 s**; keas/chi hold.
  - 13.2: `keasCmd` trim → **277.0 kt at t = 5 s**; alt/chi hold.
  - 13.3: `chiCmd` 45 → **60 deg at t = 15 s**; alt/keas hold.
  - 13.4: **latOffset step 2000 ft right at t = 20 s** — and `cmd.latOffset_ft` is *feedback*, not a
    constant: the standalone computes the aircraft's lateral deviation from the original courseline
    every step (flat-earth about the initial position, R = 6 371 000 m, course 45°) and feeds
    `lat_dev − (t ≥ 20 ? 2000 : 0)`. The host must do the same, post-step like the magnetometer
    field write in `f16_trim_ecos`. Cases 13.1–13.3 write `latOffset = 0` constant.

  **Start-up hazard found**: the plant's `ctrl.*` inputs start at 0, the plant does **not** publish
  its trim deflections, and Ecos's init rounds transfer the autopilot's init-time outputs into the
  plant before the first step — computed from whatever `fb` values the autopilot held during init.
  Mitigation: put the trim condition on the autopilot's `fb.*`/`cmd.*` inputs in the `trimPoint`
  parameter set, so its init output is the LQR's own answer at trim (≈ trim deflections, the same
  property the standalone relies on). Check the first second against the references when building.

  Reference windows are unequal again, reversed: 13.1/13.2 `sim_02`/`sim_04` end at 20 s while
  `sim_05` runs to 60 s; 13.3/13.4 end at 30/60 s while `sim_05` runs to 239.9 s (run to the longest
  window per case). The verifier will also need the commanded quantity (altitude step, KEAS, course,
  lateral offset), not only lat/lon/alt/heading — the references carry no command columns, so the
  commanded value comes from the scenario definition, as above.

* **Blocked on Aetherion: cases 15 and 16.** `F16Autopilot.fmu` (0.14.1) still hardcodes the
  circumnavigator inputs off (`circlePoleSW = 0`, built from `F16_control.dml`, not `F16_gnc.dml`).

* **Open, Aetherion-side: its trim is not NESC's.** Initial pitch 2.656087 vs 2.643331 deg (case 11),
  −0.733763 vs −0.736595 deg (case 12); weight is no longer the cause (20 509.3 vs 20 509.4 lbf). Lead,
  not finding: `TrimSolver` has no Earth-rotation or curvature terms; at case 11 they relieve 0.42 % of
  required lift, predicting −0.0172 deg against the −0.0128 deg gap (right sign and order, 135 % of
  size). Case 12 is a second data point for testing it — Eötvös and curvature grow with v and v² — but
  needs the alpha-per-weight sensitivity at Mach 2 first. Not yet written up for Aetherion.

* **Documentation figures — done (2026-09-19).** `plot_results.py` in both examples, and a
  `Results` section on each doc page: 7 figures per check-case for the trim example
  (`doc/_static/f16_trim_ecos/`, 8 embedded), 4 for the autopilot one
  (`doc/_static/f16_autopilot_ecos/`). Every caption number is measured from the run behind the
  figure, and all of them reproduce what the pages already claimed. Three things the figures
  surfaced that the prose did not have:

  - **The magnetometer's 5.3° heading bias is the hard iron, by design.** `Mmc5983maMeasurementConfig`
    draws it once per run at 1 µT/axis on top of the bridge offset; SET/RESET cancels the bridge
    offset and *cannot* touch hard iron. Measured from the run as −1.93/−0.86/−0.28 µT, which against
    the 21.9 µT horizontal field predicts 5.50° against 5.33° observed. Raw magnetic heading is not a
    heading reference — this is a state the EKF has to estimate.
  - **Every body rate on the trim flyout is below one gyro count** (p 0.70 LSB, q 0.32, r 0.39 at
    0.061 °/s per count), and body-X specific force varies by 0.82 of one accelerometer count over
    200 s. Dead reckoning has nothing to work with on case 11; heading must come from the
    magnetometer and the receiver.
  - **The autopilot example must run unpaced.** At a 0.01 s base step it advances 0.149 s of flight
    per wall-clock second, so `--rtf 1` cannot bind — and the polled I2C parts are the beneficiaries
    (40.4 baro conversions per second of flight against 4.5 in the trim example's *paced* runs). The
    240 s cases take ~27 min each; a flight-computer `--max-wall-s` below that truncates the sensor
    logs while leaving the truth log looking complete.

* **Then:** the EKF itself, judged on case 11, degraded-mode on case 12, exercised by 13.3/13.4.

---

## Sensor realism before the EKF is judged — what the figures showed

The F-16 figures (PR #37) reproduce every number on the two doc pages, and in doing so show
where the sensor chain is *too well-behaved* for a filter to be tuned against it. Ordered by
how much each assumption currently flatters the EKF. The plant-side half — wind, Dryden
turbulence, a non-standard atmosphere — is Aetherion's and is written up as
`TODO-wind-turbulence-atmosphere.md` in that repo; everything below is Hemerion's, in
`modules/sensors`, and can proceed in parallel with it.

* **GPS errors are white per epoch, and the receiver reports the true sigma.**
  `gpsNoiseModel.hpp:53–56` draws independent Gaussian N/E/D position, speed and course noise
  every fix; `:105–106` fills `hAcc`/`vAcc` with the configured sigmas. White noise is the single
  most flattering assumption a filter can be handed — it makes averaging work, and the rocket
  page's `gps_error.png` shows the RMS landing exactly on the injected sigma because of it.
  Real position error is first-order Gauss–Markov (multipath, ionosphere, ephemeris) with
  tau ≈ 1–3 min. **Add:** GM on N/E/D (sigma ≈ 1.5/3 m, tau ≈ 100 s) over a small white floor,
  the same on velocity; make `hAcc`/`vAcc` an *estimate* that is not the truth sigma (real
  receivers are optimistic); 50–200 ms of NAV-PVT latency beyond the communication step. This
  one first: it changes what the EKF's measurement model has to be more than anything else here.

  **Done — PR #39 (2026-09-24).** White + first-order Gauss–Markov per channel in
  `gpsNoiseModel.hpp` (position τ 100 s, velocity τ 10 s), `accuracy_scale` on `hAcc`/`vAcc`, the
  eleven values plus an integer `seed` as fixed FMI parameters, `--gps-errors white|correlated
  --gps-seed N` on all three hosts with the full model written to the `.config` sidecar,
  `sensors.gps_noise` (eight statistical assertions), and `<case>_gps_error.png` on the trim page
  with an autocorrelation panel against the sidecar's own prediction. Defaults reproduce the old
  model bit for bit; the realistic preset keeps every total 1-sigma within 2 % of the default and
  moves 96 % of the horizontal variance into the slow term. Three things learned on the way:
  both flight computers logged fixes at 6 significant digits (an 11 m grid at 36° latitude — the
  figure's ACF panel showed it as bands; fixed with `setprecision(10)`); a sample autocorrelation at
  lag τ scatters by ~√(τ/T), so the figure needs a long record — the published one is 35 τ
  (3548 s of fixes: the flight computer's `--max-wall-s` cut a 10 000 s run's log short, the
  trim example running at only ~2× real time unpaced); and **latency is deferred to the timing
  item** — fixes are stamped by index, so a delayed emission is invisible until arrival stamping
  exists.

* **IMU biases are constant.** `imu_noise_model.h:60–64`: white noise + a per-run turn-on bias
  + int16 quantisation, nothing else. A constant bias converges once and stops mattering, so
  the filter's bias states are trivially observable. **Add:** rate random walk / bias
  instability (consumer-MEMS class — gyro ~10 °/h per √h), ~0.1–0.5 % scale-factor error, a
  small misalignment matrix. The last two only show in the 13.x manoeuvres, which is where the
  EKF is exercised.

* **IMU full scale is the rocket's, not the aircraft's.** `ImuScale{ 800.0F, 16.4F }` at
  `imu_noise_model.h:64` is ±40 g / ±2000 °/s. On case 11 every true body rate is below one
  count (p 0.70 LSB, q 0.32, r 0.39 at 0.061 °/s per count) and body-X specific force varies
  by 0.82 of one count in 200 s — `case11_imu_body_rates.png` is a picture of quantisation
  bands. A flight computer would program ±250–500 °/s and ±8–16 g; the real part has the
  register. **Add:** range selection in the driver and the FMU (`ImuScale` per range), keep the
  wide range for the rocket. At ±250 °/s the phugoid resolves at ~5 counts — still a phugoid in
  dead calm, which is the Aetherion half of the fix; the two must not be conflated.

* **Barometer: standard day in both the plant and the sensor.** `baro_noise_model.h` inverts
  the ISA (`:154` onward); the plant's atmosphere is the US76, identical in the troposphere. So
  pressure altitude ≡ geometric altitude to within the model's 30 Pa turn-on bias
  (`bmp390_measurement_model.h:82`), which is what the ~1 m residuals on both altitude figures
  are. On a real day it is off by hundreds of feet, and that is the actual reason to fuse baro
  with GPS. **Add, as a stopgap until the plant publishes `out.p_static_Pa`:** `deltaT_K` and
  `deltaP_sl_Pa` parameters on the BMP390 FMU. Inconsistent with the plant's density (airspeed,
  thrust) — the note in Aetherion says why the correct place is the plant — but it makes the
  barometer what it is in reality: a superb *relative* altitude and a poor *absolute* one.

* **Magnetometer: a dipole field and hard iron only.** `geomagnetic_field.hpp:109–113` is a
  centred tilted dipole; its declination at Kitty Hawk is +0.69° where the real field's is
  about −11°. Harmless while the simulation is its own truth (the heading figure's 5.33°
  residual is the modelled hard iron, `mmc5983ma_measurement_model.h:70`, and is predicted to
  0.17°), wrong the moment the EKF carries a declination table. And with hard iron alone, an
  in-flight magnetic calibration is a trivial offset fit. **Add:** WMM or IGRF coefficients in
  place of the single dipole term (the header already says this is what a real site needs); a
  soft-iron matrix (3×3, near identity) in the measurement model, since that is what a
  calibration actually has to solve for.

* **Radar altimeter: fed MSL as AGL, no attitude term.** `radalt_noise_model.h:54–55` is 0.1 m
  noise on a 5 cm bias, quantised; the host wires `f16::out.alt_m → radalt::h_agl_m` because
  there is no terrain. At the bank angles in 13.3/13.4 a ±20° beam reads slant range or loses
  the ground. **Add:** terrain (a constant elevation with a little roughness is enough to make
  the point), a bank-angle dropout, and a cos(roll)·cos(pitch) slant term on the host side.

* **Sensor timing is exact and phase-locked.** `sensor_cadence.png` shows 100.00 / 40.00 /
  10.00 ms intervals with zero jitter, because every FMU steps on the master's clock. Real
  sensors run on their own oscillators with drift and jitter, and the flight computer stamps
  on arrival. **Add:** per-sensor clock skew and jitter in the FMUs, arrival-time stamping in the
  flight computer. This is what forces delayed-measurement handling in the filter, and it is
  the one item here that also changes the flight software.

* **Where to start:** correlated GPS (Hemerion) and turbulence + wind (Aetherion) in parallel —
  together they change what the EKF is tuned against more than everything else combined. Then
  IMU bias walk and the barometer stopgap. The rest as the filter grows the states that need
  them.

---

## Smaller loose ends

* **`E1126` is disabled in `.cmake-format`.** cmakelang 0.6.13 predates
  `file(REAL_PATH)` (CMake 3.19) and cannot parse it, so the two
  toolchain-sentinel calls in the top-level `CMakeLists.txt` report as errors
  with the code enabled.

  **Blocked upstream, not merely unactioned.** 0.6.13 is cmakelang's latest
  release, not just the version pinned in
  `.github/workflows/cmake_lint.yml` — there is no newer parser to move to.
  The only alternative is rewriting correct CMake to suit a linter's gap,
  which is worse than the entry. Drop it when cmakelang next releases and the
  pin moves with it.
