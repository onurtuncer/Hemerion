# Step-7 log batch, 2026-11-19: all seeds

Cases 11, 12, 13.1, 13.3 and 13.4, seeds 1–20 each: 100 runs, every one
passing its log checks. These are the rows of
[`../2026-11-19-run44517640931/`](../2026-11-19-run44517640931/README.md)
(62 seeds) and [`../2026-11-19-reruns/`](../2026-11-19-reruns/README.md)
(38 seeds), merged by case and ordered by seed.

- The rows are unchanged: each one is byte-identical to its source CSV row.
- The statistics and Markdown come from `scripts/aggregate_replay_summaries.py`'s
  own functions, so the tables read like the per-run ones.
- Settings and provenance are in the first run's README. The two runs flew
  identical code; see the re-runs README.

All figures are second-half, from the pinned 0.18.2 `EkfReplay`
(`nav-baro-gnss16`, right-invariant, default tuning). That build predates
per-block NEES, so these tables have nav ANEES only.

## Medians per case, 20 seeds

| Case | ANEES nav (/9) | seeds with ANEES > 2 | NIS GNSS pos | NIS GNSS vel | NIS baro | RMS position | RMS velocity | RMS tilt | RMS yaw |
|---|---|---|---|---|---|---|---|---|---|
| 11 | 0.77 | 1 (max 2.04) | 1.01 | 1.00 | 0.115 | 0.19 m | 0.022 m/s | 0.14° | 1.11° |
| 12 | 921 | 20 (max 1.1·10⁸) | — | — | 0.45 | 4.6 km | 56 m/s | 2.8° | 20° |
| 13.1 | 1.18 | 0 (max 1.64) | 1.00 | 1.00 | 0.118 | 0.19 m | 0.031 m/s | 0.14° | 0.76° |
| 13.3 | 1.47 | 2 (max 2.53) | 1.01 | 1.03 | 0.115 | 0.19 m | 0.031 m/s | 0.18° | 0.29° |
| 13.4 | 1.83 | 7 (max 2.67) | 1.01 | 1.03 | 0.115 | 0.18 m | 0.031 m/s | 0.24° | 0.37° |

## Reading

The full set confirms what the 62 seeds showed:

- **Doppler velocity:** case 11 velocity RMS 0.022 m/s, against 0.084 m/s in
  the 2026-11-14 batch on the speed/course model. GNSS velocity NIS is 1.00.
- **Case 12 diverges, overconfidently.** Every seed has nav ANEES above 2,
  with a median of about 900. The covariance does not grow with the
  dead-reckoning error over 200 s without GNSS. A step-7 tuning finding for
  degraded mode.
- **The autopilot cases are overconfident in step with how much they
  manoeuvre:** nav ANEES 1.18 on 13.1, 1.47 on 13.3 and 1.83 on 13.4, where 7
  of 20 seeds exceed 2. Case 11 sits at 0.77.
- **Barometer NIS is about 0.115** wherever the barometer is in its envelope:
  its R is too large.
