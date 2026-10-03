# Paper 1 — what is left

The draft targets **Simulation Modelling Practice and Theory** directly (the
SIMULTECH-first plan is retired; its extension roadmap is in git history).
Forward-looking list; the README's checklist records what has been done and
when. Branch `papers/paper1-smpt-resync`, as of 2026-10-03.

## Referee-level

- [ ] **Step the consumer on the importer's clock.** A flight-computer FMU that
      owns the three bus controllers and advances them in its own `doStep`.
      This closes three things at once: it makes a run reproducible sample for
      sample (today only the parts' streams are), it removes the host
      dependence of the polled-part sample counts (904/878 here, ~2000 on the
      docs' Linux host), and it is the precondition for closing a guidance
      loop. The paper currently defends the open loop (Discussion); a referee
      may still ask for this.
- [ ] **Monte Carlo over seeds.** The GPS RMS is quoted from 312 fixes (4 %
      standard error). Run N seeds and report the distribution against the
      analytic expectation. GPS and IMU samples are pacing-independent, so
      unpaced runs (~25 s each) suffice; the magnetometer needs paced runs.
      Mechanics: `rocket_gps_cosim --seed <n>` + `run_statistics.py` per
      results directory.
- [ ] **Radar altimeter.** Still the one sensor with a model and no case-study
      run in this paper. `examples/f16_trim_ecos` carries a radalt residual
      figure (commit `fb74c9b`): check whether that is an end-to-end run of the
      same FMU through the on-target parser and, if so, cite its numbers in a
      short subsection rather than leaving the Limitations sentence.
- [ ] **SIL → HWIL on physical hardware.** The paper now claims only
      co-simulation and Renode. Put the same decoders on an STM32H743 with a
      measured byte-level comparison across the three tiers. First step is
      `swil.mag_logger` end to end (blocked on WSL; repo `TODO.md`).
- [ ] **Multi-drop I²C.** `sim/i2c_shm` carries one peripheral per bus; a real
      board puts the BMP390 and MMC5983MA on I2C1 at `0x76` and `0x30`. A
      register-accurate claim invites exactly this probe.
- [ ] **Related Work breadth.** ArduPilot/PX4 are now cited. Still missing:
      SystemC/TLM register-accurate peripheral modelling, QEMU/Renode sensor
      emulation papers, virtual-ECU tooling (dSPACE VEOS, Synopsys
      Virtualizer), DCP, GNSS protocol/RF simulators, and a citation for the
      Introduction's "historically bug-prone" driver-code claim.
- [ ] **COCOM wording.** "An AND, in every receiver one can buy" is stronger
      than the sources support; cite the u-blox document the model follows and
      note that manufacturers differ. Also re-verify "one usable fix in 2001
      with `--dyn-model 8`" with a current run (it is quoted from the docs).
- [ ] **FMI 3.0 binary-output variant.** A referee will ask why the byte
      stream is not *also* exposed as an FMI 3.0 `Binary` output so the design
      is master-visible. Either build it (the FMUs already export 3.0) and
      report, or argue the trade explicitly in Section 2.
- [ ] **Timing terms.** `--sensor-clock` and `--gps-latency` exist but are off
      in the reference run. One run with them on, reported as a short
      subsection, would answer the "what about timing" question directly.

## Presentation

- [ ] Vector figures: `plot_results.py` writes PNG only; add PDF output and
      switch the ten `includegraphics` calls (the envelope figure is already
      PDF).
- [ ] Tone pass over what remains of the essay voice; page-count pass against
      SMPT's limit if it sets one.
- [ ] Table 7 (injected faults) is cramped in the `p{}` columns; consider
      `tabularx` or shorter cell text.
- [ ] Re-check highlights (≤85 characters), keywords (1–7) and abstract
      (≤250 words) against the Guide for Authors at submission time.

## Submission mechanics

- [ ] End matter: confirm CRediT roles, competing interests and funding; fill
      in or delete the generative-AI declaration (every `% TODO` in
      `main.tex`).
- [ ] Zenodo (or equivalent) DOI for the exact commit, cited from Data
      availability instead of the live repository.
- [ ] Verify citations against primary sources: NASA TM-2015-218675
      title/authors, `bosch_bmp390` and `memsic_mmc5983ma` document numbers,
      and the three entries BibTeX flags as missing pages (`pedersen2016fmi`,
      `mikelsons2017virtual`, `jackson2011daveml`).
- [ ] Cover letter and suggested reviewers.

## Repository follow-ups the paper surfaced

- [ ] Document `gps_flight_computer --fault` in
      `examples/rocket_gps_ecos/README.md` and `doc/rocket_gps_ecos_cosim.rst`.
- [ ] The consumer's poll loop takes ~0.23 s per pass on Windows against
      ~0.1 s on Linux, which is what sets the polled-part counts; look at the
      shared-memory link's backoff (`sleep_for` granularity on Windows) before
      blaming the drivers.
- [ ] `doc/rocket_gps_ecos_cosim.rst` still quotes the original three-sensor
      transcript and a different run's magnetometer numbers (48–57°); fine as
      labelled, but a single current transcript would be cleaner.
