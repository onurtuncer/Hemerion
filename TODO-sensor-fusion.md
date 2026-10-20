# Sensor fusion — Hemerion half

Drafted 2026-10-08 against `main` at `55a95ce` and Aetherion `main` at
`9a80aac` (v0.16.0); steps 1–4 closed 2026-10-19 against Aetherion v0.18.0,
whose reference filter and replay harness (step 6) now exist and have replayed
a real Hemerion log. The Aetherion half is `TODO-sensor-fusion.md` in that
repo; the step numbers below are shared between the two files so the
cross-repo order is readable from either. This file picks up where `TODO.md`
leaves off at "Then: the EKF itself".

## Where things stand

Already built and verified here, so nobody repeats it:

- Five sensor FMUs (GPS, IMU, BMP390, MMC5983MA, radar altimeter) with
  correlated GPS error, IMU bias walk, scale factor and misalignment, range
  selection, clock skew and jitter, GPS latency, soft and hard iron, WMM2025.
- The on-target drivers run unchanged in the host flight computer; every log
  carries `part_time_s` and `host_time_s`.
- Ecos co-simulation of NESC cases 11, 12 and 13.1 to 13.4 against Aetherion's
  plant FMUs, with `--seed`, `--wind`, `--turbulence`, `--atmosphere`.

Not built:

- `modules/gnc` and `apps/gnc_flight` are READMEs only. No estimator exists.
- `sim/fmi` is a `package.xml`. The working master is Ecos, in `examples/`.

## Decided (2026-10-08)

- **Ownership.** Hemerion owns everything a sensor or a target touches: the
  FMUs, drivers, timing, the fixed-size flight filter in `modules/gnc`, the
  closed-loop runs and target timing. Aetherion owns truth and the estimator
  mathematics, including the reference filter and the code generator.
- **Filter form.** Lie group EKF. The 15-state quaternion design in
  `modules/gnc/README.md` is old and not the intent.
- **The generator lives in Aetherion.** `tools/gen_ekf_jacobians.cpp` will not
  be written here. `modules/gnc` receives generated plain C plus a
  configuration hash. The no-CppAD-on-target constraint is unchanged.
- **Hemerion never composes states.** Each firmware build takes one generated
  configuration, so the dimension is a compile-time constant and every matrix
  is fixed-size. The skeleton is written against generated block offsets, not
  hard-coded indices.
- **First configuration: 16 error states.** Navigation core (9), gyro bias
  (3), accelerometer bias (3), barometer bias (1, metres of pressure altitude,
  random walk). Updates: GNSS position, GNSS velocity, barometer.
- **No magnetometer in the filter** until replay equivalence (step 9) has
  passed once. The drivers, calibration and WMM stay as they are.

## Decided (2026-10-19)

- **Rates (step 4): IMU 100 Hz, GNSS 10 Hz.** One strapdown propagation per
  IMU sample; covariance prediction at most that often. See step 4 below for
  the reasoning; `modules/gnc/README.md` and `apps/gnc_flight/README.md` now
  say the same.
- **Aetherion v0.18.0 is the floor** for the co-simulation examples, by
  version and not only by port: 0.17.4 corrected the rigid body's spatial
  inertia (parallel-axis term; TwoStageRocket's Iyy was short by m DXCG²) and
  its gravity-mass coupling, 0.18.0 moved the gravity wrench to the body
  origin. No port changed, so the configure-time port tests could not see it;
  `cmake/aetherion_plant_version.cmake` reads the install's version instead.

## Done since (2026-10-19), against Aetherion's step-2/step-6 findings

- **GPS validity time.** Both flight computers stamped `nominal_time_s` =
  `fix_index` × period, one period late: fix k carries the plant state from
  the *start* of the step that emits it. Now (`fix_index` − 1) × period. Logs
  written before need −0.1 s; every plot script, and Paper 1's
  `run_statistics.py`, recognise the old convention from the intercept of
  stamp against `fix_index` and align both the same way.
- **GNSS velocity logged.** `GpsFix` carries `vel_north/east/down_mps`,
  `speed_accuracy_mps` (`sAcc`) and `ned_velocity_valid`; `UbxParser` decodes
  them; `gps_fixes.csv` appends the four columns under the names Aetherion
  proposed. velD used to be a hard-coded 0: it is now truth plus its own noise
  (0.2 m/s default) from a separate RNG stream, so every other draw of a seeded
  run is bit-identical to before (checked against `main` over 18 000 draws).
  `sAcc` is honest for an isotropic consumer: the per-axis sigma whose
  isotropic covariance has the velocity error's true trace. Note what that is
  at speed: the speed/course error model puts 1° of course noise *across*
  track, 3 m/s at 172 m/s, so the default receiver reports ~1.7 m/s and the
  velocity update is weak on the F-16 cases. A Doppler receiver's velocity
  error is per axis and speed-independent (~0.05–0.1 m/s); modelling it that
  way would change Paper 1's documented receiver, so it is a decision, not a
  fix — see "Still open".
- **GPS altitude datum.** `GpsFix::altitude_m` documented as ellipsoidal from
  UBX (`height`, not `hMSL`) and MSL from NMEA (the NMEA parser does not add
  the geoid separation). Only comments changed.
- **Rocket truth attitude.** `rocket_truth.csv` now logs `out.yaw_rad`,
  `out.pitch_rad`, `out.roll_rad` (ZYX Euler, singular at the 90° pitch of a
  vertical launch: a quaternion output is Aetherion's to add).
- **Rocket sidecar.** It wrote `alt0_m` where the replay requires `alt0_ft`;
  both are written now. Rocket logs could not be replayed at all before.

## Verified end to end (2026-10-19, Aetherion v0.18.0, `--rtf 1`)

Case 11 calm and with `--turbulence 8` (seed 11, 200 s), the rocket (seed
17) and 60 s of autopilot case 13.3, replayed with Aetherion's `EkfReplay`
built from v0.18.0, on the logs exactly as written (no stamp correction):

| Run / config | ANEES (nav, /9) RI / LI | NIS/dim GNSS pos | GNSS vel | position RMS |
|---|---|---|---|---|
| case 11 calm, nav-baro-gnss | 1.58 / 0.75 | 1.03 | — | 0.41 m |
| case 11 calm, nav-baro-gnss16 | 1.28 / 0.75 | 1.03 | 1.004 | 0.40 m |
| case 11 turbulence, nav-baro-gnss16 | 1.34 / 1.43 | 1.04 | 1.004 | 0.47 m |

GPS error at the stamp 2.12 m RMS (17.4 m one period either side); the
logged drawn IMU and BMP390 biases agree with the decoded streams to within
~1 standard error; the filter's barometer-bias estimate (+5.05 ± 0.23 m)
matches the drawn −44.7 Pa (+5.04 m). `nav-baro-gnss16` could not bind
before this change.

## Still open

- [ ] **GNSS velocity error model.** Per-axis NED Doppler noise instead of
      speed/course noise, deriving speed and course from it as a receiver
      does. Better for the filter; changes the receiver Paper 1 documents
      (its parameter table quotes speed 0.1 m/s and course 1.0°).
- [ ] **Radar altimeter rate at a fine step.** `f16_trim_cosim` steps the
      radalt FMU every base step with at least one return, so `--step 0.01`
      makes the "25 Hz" part 100 Hz (2000 returns in 20 s). Not in the
      16-state filter; fix it the way `--gps-period` was (a step-size hint
      when 1/rate is a whole number of steps), as the autopilot host does.
- [ ] **Paper 1 against v0.18.0.** The rocket now flies the corrected
      dynamics: still inside the NESC envelope at every sample, but stage-2
      ignition 138.38 km (paper 138.52), stage-2 burnout 229.10 km (paper
      220.15), 249 km at 200 s (paper "236 km"), 515 m/s at t = 10.2 s (paper
      10.1). The paper's GPS, IMU and fault statistics are unaffected by the
      stamp change (`run_statistics.py` gives identical output under both
      conventions). Decide whether to re-run the paper's results on 0.18.0.
- [ ] **Host-clock residual.** Aetherion's real replay found the
      `host = offset + scale × part` fit off by 62 ms rms, 302 ms max, where a
      0.01 s sawtooth was expected. Only a cross-check today, but step 10's
      delayed-measurement handling will rely on arrival stamps.

## To do, in order

### Step 3 — Publish the realized error states

Done (2026-10-19). Nothing on the bus side changed, and reading a term draws
nothing, so defaults stay bit-identical.

- [x] IMU FMU: 18 diagnostic outputs — `accel_bias_{x,y,z}_mps2` and
      `gyro_bias_{x,y,z}_rad_s` (continuous: they walk), and the drawn
      `accel_scale_error_*`, `gyro_scale_error_*` (fraction), and
      `accel_misalignment_*_rad`, `gyro_misalignment_*_rad`. Each axis reads
      scale × ((I + skew(misalignment)) × truth) + bias + noise;
      `test_imu_drawn_terms_reconstruct_the_output` rebuilds the part's output
      from the published terms to the quantiser.
- [x] BMP390 FMU: `pressure_bias_pa`, `temperature_bias_c`. In pascals, as
      drawn: Aetherion's barometer state is metres of pressure altitude, so
      the conversion (dh = −dp / (ρ g) at the flight's pressure) is on the
      replay side.
- [x] Logged by all three hosts as truth columns (`with_drawn_errors()` in
      `examples/common/environment.hpp`), every step.
- Deferred with the magnetometer: drawn hard iron and soft iron.

### Step 4 — Reconcile the rates

`modules/gnc/README.md` and `apps/gnc_flight/README.md` say 500 Hz predict
and 18 Hz GPS (and a 4 kHz IMU ISR). The examples run a 100 Hz IMU and 10 Hz
GPS.

- [x] Unblock Aetherion's tuning from the rate (2026-10-08). The IMU FMU's
      white noise was a per-sample sigma, so `--imu-rate` silently changed
      the noise density. It now also takes `accel_noise_density_mps2_sqrt_hz`
      and `gyro_noise_density_rad_s_sqrt_hz`; nonzero, each sample carries
      density / sqrt(period), so the noise is the same physics at any rate.
      Defaults are 0 and bit-identical. The current defaults at 100 Hz are
      equivalent to 5e-3 m/s^2/sqrt(Hz) and 2e-4 rad/s/sqrt(Hz) (about
      0.69 deg/sqrt(h)). Aetherion tunes in continuous time against those
      densities; the rate below only changes the discretisation.
- [x] Decided (2026-10-19): **IMU 100 Hz, GNSS 10 Hz; the READMEs move, the
      examples stay.** The deciding fact is what the co-simulation can verify:
      the FMUs zero-order-hold the plant between communication points, so an
      IMU rate above 1/step repeats the held input with fresh noise — samples,
      not information — and Aetherion's replay measured that hold as a 0.055 s
      effective lag at the 0.1 s step. At `--step 0.01` the default 100 Hz IMU
      is exactly one sample per step from that step's plant state. 500 Hz
      would be 5 identical-truth samples per step, the reopened sub-step
      cadence and 5× the Monte Carlo cost, for nothing a filter can use. GNSS
      stays at 10 Hz: one NAV-PVT per 0.1 s is what every example emits and
      what the fix-index time mapping rests on, and with ~100 s of error
      correlation 18 Hz adds little independent information. A faster target
      rate (400–500 Hz with coning/sculling pre-integration from a faster ISR)
      is a later step that needs sub-step emission in the FMUs first.
- [x] Examples: `f16_trim_cosim` and `rocket_gps_cosim` take `--gps-period`
      (default 0.1) as the GPS FMU's step-size hint, as `f16_autopilot_cosim`
      already did, so `--step 0.01` no longer silently makes a 100 Hz receiver
      (which would also have put every fix stamp off by 10× unless the flight
      computer got `--fix-period 0.01`). Rejected unless it is a whole number
      of steps.
- [ ] **Tell Aetherion** (its `TODO-sensor-fusion.md`, step "Rates"): design
      rates IMU 100 Hz / GNSS 10 Hz, barometer at the programmed ODR; noise
      densities 5e-3 m/s²/√Hz and 2e-4 rad/s/√Hz (the 100 Hz per-sample
      defaults, 0.05 m/s² and 0.002 rad/s); GNSS velocity now logged, with the
      caveat on `sAcc` above; `nominal_time_s` now the validity time; drawn
      errors in the truth log under the names in `drawn_error_variables()`;
      for manoeuvring cases run `--step 0.01` (the GPS stays at 10 Hz).

### Step 2 — Log contract (shared; Aetherion holds the definition)

- [x] Done (2026-10-19). Every name Aetherion's replay binds — the three
      sensor-log headers and the five sidecar keys — is written from
      `examples/common/log_contract.hpp` by both flight computers and all
      three hosts. `tests/unit/test_log_contract.cpp` (`examples.log_contract`,
      runs in CI) pins them against a copy of Aetherion v0.18.0's
      `LogContract.h`; where an install is found,
      `examples.log_contract_upstream` checks that copy against the real
      header with `scripts/check_log_contract.py`, which also checks a finished
      run's logs (`--results`). Both were shown to fail on a renamed column, a
      renamed key and a stale pinned copy.

### Step 1 housekeeping — bring the READMEs in line

- [x] `modules/gnc/README.md`: the 16-state SE₂(3) configuration, the LCI
      frame, the generator in Aetherion, the step-4 rates, the contract test.
- [x] `apps/gnc_flight/README.md`: task rates follow step 4.

### Step 9 — `modules/gnc`

Starts when Aetherion delivers generated C and golden vectors (its step 8).

- [ ] `CMakeLists.txt`, `include/`, `src/`, `test/`; add the module so the
      root foreach stops skipping it.
- [ ] Check in the generated C and header under `src/`, with the configuration
      hash. Never edit them by hand.
- [ ] Filter skeleton, hand-written: float32, fixed-size, no heap, ETL only,
      JSF++ rules. Covariance form as decided in Aetherion's step 7b.
      High-rate state propagation, lower-rate covariance prediction,
      sequential scalar updates.
- [ ] First consumer of `HEMERION_FLIGHT_SAFE_BEGIN/END`.
- [ ] **Replay equivalence under `test-native`**: run the golden vectors and
      require agreement within the stated tolerance. Refuse to run if the hash
      in the vectors differs from the hash compiled into the module. This test
      is the contract between the two repos.

### Step 10 — Close the loop

- [ ] Run the filter inside `f16_flight_computer`, fed by the drivers it
      already runs. Delayed-measurement handling for GNSS from the arrival
      stamps; exercise it with `--gps-latency`.
- [ ] Log the estimate and covariance beside the sensor logs.
- [ ] `gnc.fmu` through `generateFMU()`, wired into the Ecos examples.
- [ ] Seeded Monte Carlo batch over the hosts (`--seed`). Cost to plan for:
      the autopilot cases run unpaced at about 27 minutes per 240 s case.
- [ ] Judge on the cases already named in `TODO.md`: case 11 baseline, case 12
      degraded mode, 13.3 and 13.4 for heading. Turbulence on. Yaw covariance
      growing on case 11 without a magnetometer is expected.

### Step 11 — Target timing and fault logic

- [ ] `apps/gnc_flight` with the estimator task; `renode-h743` for WCET and
      stack, and behaviour under scheduling jitter.
- [ ] Innovation gating and sensor health, through `modules/fault`.
- [ ] Alignment and in-air reinitialisation. Heading initialisation must not
      use GNSS course (2.28 degrees from yaw in a 10 m/s crosswind).
- [ ] Innovation telemetry to the ground station.
- [ ] Send the fault cases back to Aetherion as injected scenarios.

## Deferred

- Magnetometer in the filter (hard iron, soft iron, declination): after
  step 9 passes. `MagneticCalibration` and `WorldMagneticModel` are ready for
  it.
- Second vehicle configuration (rocket): after step 9.
- FIR group delay: `hemerion-rtl` is not a submodule on `main` and no FIR is
  in the sensor chain. When one enters, its group delay must be declared to
  the filter.
- Sub-step emission cadence in the FMUs: still not needed.
- A Hemerion-owned FMI master in `sim/fmi`: Ecos is sufficient for this work.
