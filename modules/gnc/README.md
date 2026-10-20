# modules/gnc/

> **Status: not implemented.** This directory contains this README and a `package.xml` — no `include/`, no `src/`,
> no `test/`, no `CMakeLists.txt`. The root `CMakeLists.txt` skips a module with no `CMakeLists.txt` silently, so
> there is no `hemerion_gnc` target in any preset. Everything below is design intent, decided with Aetherion on
> 2026-10-08 and recorded step by step in [`TODO-sensor-fusion.md`](../../TODO-sensor-fusion.md); the step numbers
> below are that file's.

## Planned scope

The fixed-size, float32 flight filter that runs on the STM32H743: an invariant (Lie group) EKF whose mathematics is
**not written here**. Aetherion owns the estimator — state blocks, process and measurement models, the double-precision
reference filter, its tuning and the code generator — and this module receives, per vehicle configuration, generated
plain C plus a configuration hash. Hemerion owns everything a sensor or a target touches: the hand-written skeleton
around the generated code, timing, delayed-measurement handling from the drivers' arrival stamps, fault logic, and the
closed-loop runs. Generated code may flow from Aetherion (MIT) into Hemerion (GPL-3.0); nothing flows back.

The 15-state quaternion EKF this README used to describe is superseded.

## First configuration: 16 error states

The state is composed from named blocks at build time, never resized at run time; a measurement a configuration lacks
is skipped, not modelled as a zero. The first configuration, Aetherion's `kNavBaroGnss16`:

| Block | Offset | Dim | Units | Model |
|---|---|---|---|---|
| Navigation core | 0 | 9 | rad, m/s, m | one SE₂(3) element (R, v, p) in the launch-centred inertial frame |
| Gyro bias | 9 | 3 | rad/s | random walk |
| Accelerometer bias | 12 | 3 | m/s² | random walk |
| Barometer bias | 15 | 1 | m of pressure altitude | random walk (a temperature offset drifts in a climb) |

* **Group** SE₂(3) × ℝ⁷. The strapdown kinematics are group-affine on the navigation core up to the gravity gradient;
  the biases stay Euclidean.
* **Frame** launch-centred inertial (LCI): origin at the launch point, axes local NED at t₀, then frozen in inertial
  space. No Earth-rate or Coriolis terms in the propagation, float32-safe positions, and it carries the rocket
  unchanged. GNSS and barometer models handle the time-dependent ECEF↔LCI rotation.
* **Updates** GNSS position, GNSS velocity, barometer. **No magnetometer** until replay equivalence (step 9) has passed
  once; `MagneticCalibration` and `WorldMagneticModel` are ready for it.
* **Error convention** (left- or right-invariant) is still open on Aetherion's side and is part of the configuration
  hash, so it cannot be mismatched silently.
* **Covariance form** (Joseph, or UD / square root for float32) is decided by Aetherion's step 7b against truth, and is
  an input to the skeleton.

Indices are never hard-coded: the skeleton is written against the generated block offsets.

## Rates (step 4)

| Stream | Rate | Why |
|---|---|---|
| IMU → state propagation | **100 Hz**, one strapdown step per sample | What the co-simulation can verify: at `--step 0.01` the IMU FMU emits exactly one sample per step from the plant state of that step. A faster part rate inside a step only repeats the held input (the plant is zero-order-held between communication points), so 500 Hz would add samples, not information, until the FMUs emit at sub-step cadence. |
| Covariance prediction | ≤ 100 Hz, batched over IMU samples if the timing budget needs it (step 11) | |
| GNSS position + velocity | **10 Hz** | One NAV-PVT per 0.1 s, as every example emits it (`--gps-period`, independent of `--step`). The receiver's error is correlated over ~100 s, so 18 Hz would add little independent information for a higher interrupt load. Delayed fixes are applied at their validity time from the arrival stamps (step 10). |
| Barometer | the ODR the flight computer programs the BMP390 to | |

Aetherion tunes in continuous time from the IMU noise densities (`accel_noise_density_mps2_sqrt_hz`,
`gyro_noise_density_rad_s_sqrt_hz`), so a later change of rate changes the discretisation, not the tuning. A faster
target rate — 400–500 Hz with coning/sculling pre-integration from a faster driver ISR — is a later, separate step that
needs sub-step emission cadence in the FMUs first (deferred in `TODO-sensor-fusion.md`).

## Generated code and the contract test (step 9)

Aetherion's generator (CppADCodeGen, host-side only) emits plain C for the process and measurement functions and their
Jacobians for one configuration, plus a header with the dimension, the block offsets and the configuration hash, and
golden vectors recorded from the reference filter carrying the same hash. They are checked in under `src/` and never
edited by hand. The target has **no CppAD dependency**, and the `tools/gen_ekf_jacobians.cpp` once planned here will
not be written.

The contract between the two repositories is one test, under `test-native`: run the golden vectors through this
module and require agreement within the stated tolerance, refusing to run at all if the hash in the vectors differs from
the hash compiled in.

The filter skeleton is hand-written: float32, fixed-size, no heap, ETL only, JSF++ rules; state propagation per IMU
sample, covariance prediction possibly batched, sequential scalar updates. It will be the first consumer of
`HEMERION_FLIGHT_SAFE_BEGIN/END`.

## Dependencies once implemented

Per `modules/README.md`'s rules: `FreeRTOS::Kernel`, `ETL::etl`, and the BSP HAL headers only.

## Downstream: the `ekf_predict` FPGA IP

The `ekf_predict` IP block (maintained in the separate RTL repository) was sized for the superseded 15-state quaternion
design. It has to follow the configuration above — 16 states, block offsets from the generated header, the covariance
form step 7b chooses — before it can accelerate this filter.

## Open question

Whether `gnc::EkfCore` and `gnc::Se3Controller` (named in earlier design notes) survive as the actual class names. The
controller side is not designed yet: as wired today the F-16 autopilot reads plant truth, including air data
(airspeed, α, β, density) that a navigation filter does not estimate, so closing the loop on estimates (step 10) needs
a decision on where those come from.
