# Step-7 log batch, 2026-11-19: re-flown seeds

The 38 case/seed pairs that run
[44517640931](../2026-11-19-run44517640931/README.md) lost to the Ubuntu
mirror, flown again after the apt retries (#71). Same settings as that run.
All 38 passed every check.

| Case | Seeds | Run |
|---|---|---|
| 11 | 5 | [44530268940](https://github.com/onurtuncer/Hemerion/actions/runs/44530268940) |
| 12 | 1, 7, 8, 11, 13, 14, 17, 20 | [44532755317](https://github.com/onurtuncer/Hemerion/actions/runs/44532755317) |
| 13.1 | 1, 3, 4, 5, 8, 9, 10, 11, 12, 14, 16, 17, 19, 20 | [44534486143](https://github.com/onurtuncer/Hemerion/actions/runs/44534486143) |
| 13.3 | 1, 3, 4, 9, 12, 13, 14, 15, 16, 19, 20 | [44535799287](https://github.com/onurtuncer/Hemerion/actions/runs/44535799287) |
| 13.4 | 1, 2, 5, 8 | [44537264437](https://github.com/onurtuncer/Hemerion/actions/runs/44537264437) |

- Hemerion: `1927309`. It differs from the first run's `85d3052` only under
  `.github/` and `results/`, so the flights ran identical code.
- Aetherion plant and EKFReplay: 0.18.2 at `49e75b8`, Linux.
- The tables for cases 12–13.4 are copied unchanged from each run's
  `step7-summary` artifact.
- **Case 11's table was produced locally.** That run's aggregate job failed on
  a one-seed dispatch (fixed in #73), so there is no `step7-summary` artifact.
  The table was made with `scripts/aggregate_replay_summaries.py` from the
  run's `step7-case11-seed-5` artifact, the same script and inputs CI uses.
- The per-seed logs are artifacts on each run, kept by GitHub for 90 days.

The full 20-seed tables per case are in [`../2026-11-19-combined/`](../2026-11-19-combined/README.md).
