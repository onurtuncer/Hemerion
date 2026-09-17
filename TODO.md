# TODO

Work that is written but not yet closed out. Each entry says what is already
verified, so nobody repeats it, and what specifically remains.

---

## F-16 NESC check-cases — branch `examples/aetherion-specific-force` (not merged)

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

  Remaining: README case-12 section with the figures above and the verification commands; the file
  header of `cosim_host_main.cpp` still describes only case 11; `f16_flight_computer.cpp` comments
  assume every stack is in band (its exit criterion already counts samples regardless of validity,
  which is right for case 12); the `examples/README.md` row mentions only case 11; and the Sphinx
  page for `f16_trim_ecos` does not exist yet. Decide whether the BMP390 FMU should flag operation
  outside its pressure/temperature rating.

* **Regenerating the Aetherion standalone references** (needed for `verify_trajectory.py
  --aetherion`, must be a build that includes 0.14.1's `TrimWeight.h`), from the Aetherion root:
  `<build>/src/Examples/F16SteadyFlight/F16SteadyFlight.exe --timeStep 0.1 --endTime 200
  --writeInterval 1 --inputFileName unused --outputFileName f16_s11.csv` (and `F16SupersonicTrim`
  for case 12). `--inputFileName` is required by the CLI but ignored.

* **Next: `examples/f16_autopilot_ecos`, cases 13.1–13.4.** `F16Plant` + `F16Autopilot` + the five
  sensor FMUs; plant `out.*` → autopilot `fb.*`, autopilot `ctrl.*` → plant `ctrl.*`. The FMU has no
  step scheduling, so the host owns the command schedule on `cmd.altCmd_ft`, `cmd.keasCmd_kt`,
  `cmd.baseChiCmd_deg`, `cmd.latOffset_ft`: 13.1 +100 ft at 5 s; 13.2 KEAS → 277 kt at 5 s; 13.3
  course 45 → 60 deg at 15 s; 13.4 2000 ft right of course at 20 s. Initial KEAS command = trim KEAS
  from the plant's density and airspeed.

  **Investigate before building:** the autopilot FMU evaluates once per Ecos communication step with a
  one-step transport delay — at the 0.1 s step that is a 10 Hz sampled loop with 0.1 s delay, which is
  probably not what Aetherion's standalone `F16HeadingChange` does, so it would not reproduce the same
  numbers. But the GPS FMU emits one NAV-PVT per step, so simply using a 0.01 s step means 100 Hz GPS.
  Check how the standalone evaluates its controller, whether Ecos supports per-model step decimation,
  and whether the GPS FMU can decimate.

  Reference windows are unequal again, reversed: 13.1/13.2 `sim_02`/`sim_04` end at 20 s while
  `sim_05` runs to 60 s; 13.3/13.4 end at 30/60 s while `sim_05` runs to 239.9 s. The verifier will
  also need the commanded quantity (altitude step, KEAS, course, lateral offset), not only
  lat/lon/alt/heading.

* **Blocked on Aetherion: cases 15 and 16.** `F16Autopilot.fmu` (0.14.1) still hardcodes the
  circumnavigator inputs off (`circlePoleSW = 0`, built from `F16_control.dml`, not `F16_gnc.dml`).

* **Open, Aetherion-side: its trim is not NESC's.** Initial pitch 2.656087 vs 2.643331 deg (case 11),
  −0.733763 vs −0.736595 deg (case 12); weight is no longer the cause (20 509.3 vs 20 509.4 lbf). Lead,
  not finding: `TrimSolver` has no Earth-rotation or curvature terms; at case 11 they relieve 0.42 % of
  required lift, predicting −0.0172 deg against the −0.0128 deg gap (right sign and order, 135 % of
  size). Case 12 is a second data point for testing it — Eötvös and curvature grow with v and v² — but
  needs the alpha-per-weight sensitivity at Mach 2 first. Not yet written up for Aetherion.

* **Then:** open a PR for this branch; the EKF itself.

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
