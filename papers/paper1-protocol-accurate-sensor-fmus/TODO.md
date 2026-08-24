# Journal companion — extension roadmap

The conference paper targets **SIMULTECH / SIMPAC**. The companion targets a
journal — primary **Simulation Modelling Practice and Theory (SMPT, Elsevier)**,
which has a standing invited-extension pipeline from SIMULTECH; fast open-access
fallback **MDPI *Aerospace***.

A journal version must add ~30–40% genuinely new material over the conference
paper, cite it explicitly, and avoid self-plagiarism. Items 1 and 2 below alone
justify the extension; the rest deepen it.

## Extension delta

- [x] ~~**1. Validate all five sensors end-to-end, not just GPS+IMU.**~~ Mostly
  done, and it went into the conference paper rather than being saved for the
  journal: the BMP390 and MMC5983MA are now register-accurate I²C parts with
  case-study results (Sections "The barometer stops being an altimeter" and
  "The magnetometer is unusable until it is conditioned"). **The radar altimeter
  remains unvalidated** — the scenario is a launch to orbit and has no ground
  under it. Validating it needs a different scenario, which is still a real
  journal delta.

- [ ] **2. Quantify SIL→HWIL transfer.** Partly closed: the BMP390 SWIL loop
  (firmware in Renode → emulated I²C → C# bridge → TCP → shared-memory bus →
  the same device model the FMU embeds) now gates in CI, and the conference
  paper says so. Still to do: run `swil.mag_logger` end to end (blocked on WSL,
  see the repo's `TODO.md`), and put the same decoders on physical STM32H743
  hardware with a measured byte-level comparison across all three tiers.

- [ ] **3. Richer error models.** Correlated in-run bias (Gauss-Markov),
  scale-factor / misalignment, and the soft-iron term deliberately deferred —
  each plugged into the same `sat(round(·))` form. The magnetic environment is
  also still a centred tilted dipole rather than a spherical-harmonic model,
  and the MMC5983MA's self-test coil is register-modelled but not magnetically
  modelled, so a driver self-test passes vacuously.

- [ ] **6. Multi-drop I²C.** `sim/i2c_shm` carries one peripheral per bus, so
  the BMP390 and the MMC5983MA sit on separate simulated buses where a real
  board puts both on I2C1 at `0x76` and `0x30`. Addressing two parts on one bus
  is the one thing about driving I²C the work does not exercise, and it is
  exactly the kind of gap a register-accurate claim invites a reviewer to probe.

- [ ] **4. Deeper FMI-LS-BUS comparison.** Possibly an actual head-to-head on a
  bus both can express, turning the three-axes argument into a measured
  contrast.

- [ ] **5. Citable reproducibility artifact.** The JOSS paper discussed earlier,
  referenced from the journal version to give it a DOI.
