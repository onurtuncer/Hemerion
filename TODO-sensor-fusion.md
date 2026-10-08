# Sensor fusion — Hemerion half

Drafted 2026-10-08 against `main` at `55a95ce` and Aetherion `main` at
`9a80aac` (v0.16.0). The Aetherion half is `TODO-sensor-fusion.md` in that
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

## Still open

- [ ] **Rates (step 4).** See below. No longer blocks Aetherion's tuning,
      which can start from the IMU noise densities; it still sets the
      discretisation and the co-simulation cost.

## To do, in order

### Step 3 — Publish the realized error states

Verified: each sensor `fmu_main.cpp` registers its sigmas and `seed` as
parameters, and none of the values drawn from them. Without those, bias-state
consistency (NEES) cannot be computed on the Aetherion side.

- [ ] IMU FMU: diagnostic outputs for the current gyro and accelerometer bias
      (they walk, so outputs, not parameters), and the drawn scale-factor and
      misalignment terms.
- [ ] BMP390 FMU: drawn pressure and temperature bias.
- [ ] Log them from the three co-simulation hosts into the truth CSV, or a
      sidecar beside it.
- [ ] Same rule as the existing diagnostics: change nothing on the bus side,
      and defaults stay bit-identical.
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
- [ ] Decide the IMU output rate and the GNSS rate the filter is designed
      for, and say whether the examples or the READMEs move. A 500 Hz IMU at
      the 0.01 s base step buffers 5 samples per step, all delivered at the
      step boundary, which reopens "sub-step emission cadence" under
      Deferred. It also raises the Monte Carlo cost in step 10.
- [ ] Tell Aetherion the densities above and, once decided, the rates.

### Step 2 — Log contract (shared; Aetherion holds the definition)

- [ ] Treat the column names of `gps_fixes.csv`, `imu_samples.csv`,
      `baro_samples.csv`, the truth CSV and the `.config` sidecars as an
      interface: Aetherion's replay harness reads them. Document them in one
      place and add a test that fails on a rename.

### Step 1 housekeeping — bring the READMEs in line

- [ ] `modules/gnc/README.md`: replace the 15-state table, the generator
      section and the rates with the decisions above.
- [ ] `apps/gnc_flight/README.md`: task rates follow step 4.

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
