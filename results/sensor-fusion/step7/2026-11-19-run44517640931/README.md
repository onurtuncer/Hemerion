# Step-7 log batch, 2026-11-19 (run 44517640931)

EKFReplay summaries from the step-7 tuning batch (see `TODO-sensor-fusion.md`,
and step 7 of Aetherion's `TODO-sensor-fusion.md`). Copied unchanged from the
run's `step7-summary` artifact. The per-seed logs (`step7-case<case>-seed-<n>`
artifacts) are on the run, kept by GitHub for 90 days from 2026-11-19; they
are not in git.

- Run: https://github.com/onurtuncer/Hemerion/actions/runs/44517640931
- Hemerion: `85d3052` (main after #66, #67, #68 and #69)
- Aetherion plant and EKFReplay: 0.18.2 at `49e75b8`, Linux (ubuntu runner)
- Dispatch: `cases=11,12,13.1,13.3,13.4 seeds=1-20 turbulence=8 max_parallel=20`
- Settings: IMU 100 Hz, GNSS 10 Hz, step 0.01 s, MIL-F-8785C W20 8 m/s with
  turbulence seed = seed, each case over its reference window (200 s for 11
  and 12, 60 s for 13.1, 240 s for 13.3 and 13.4), every sensor FMU at its
  defaults. The GNSS velocity error is the per-axis Doppler model (#67), its
  first batch. Filter: `nav-baro-gnss16`, right-invariant, default tuning.

## Coverage: 62 of 100 seeds

38 jobs failed in "Install system deps" when the Ubuntu mirrors timed out or
answered 503, before anything was built or flown; none failed in a build,
flight or replay step. These tables cover only the seeds that ran:

| Case | Seeds in the tables |
|---|---|
| 11 | 19 (all but 5) |
| 12 | 12 |
| 13.1 | 6 |
| 13.3 | 9 |
| 13.4 | 16 |

The missing seeds are to be flown in a follow-up run once the apt retries
(#71) are merged; its summaries go in a sibling directory.

## Medians per case (second half of each run)

| Case | ANEES nav (/9) | NIS GNSS pos | NIS GNSS vel | NIS baro | RMS position | RMS velocity | RMS tilt | RMS yaw |
|---|---|---|---|---|---|---|---|---|
| 11 | 0.77 | 1.00 | 1.00 | 0.115 | 0.19 m | 0.022 m/s | 0.15° | 1.13° |
| 12 | 1380 | — | — | 0.45 | 5.8 km | 64 m/s | 3.0° | 27° |
| 13.1 | 1.34 | 0.99 | 1.00 | 0.118 | 0.18 m | 0.032 m/s | 0.15° | 0.74° |
| 13.3 | 1.35 | 1.01 | 1.03 | 0.114 | 0.21 m | 0.031 m/s | 0.17° | 0.38° |
| 13.4 | 1.83 | 1.01 | 1.02 | 0.115 | 0.18 m | 0.031 m/s | 0.24° | 0.34° |

## Reading

- **Doppler velocity:** case 11 velocity RMS 0.022 m/s against 0.084 m/s in
  the 2026-11-14 batch (speed/course model), with GNSS velocity NIS still
  1.0; position RMS 0.19 m against 0.43 m.
- **Case 12 diverges, overconfidently.** With no GNSS fix for 200 s at
  Mach 2, kilometres of drift is expected, but nav ANEES of ~10³ (up to ~10⁸
  on some seeds) says the covariance does not grow with the error. A tuning
  finding for step 7's degraded mode, not a logging fault: every case-12 run
  passed its log checks.
- **Autopilot cases mildly overconfident:** nav ANEES 1.3–1.8 against 0.77
  on case 11, highest on 13.4 (heading observability).
- **Barometer NIS ~0.115** wherever the barometer is in its envelope,
  consistent with Aetherion's first step-7 change (measurement sigma about
  0.34 m, not 1 m).
