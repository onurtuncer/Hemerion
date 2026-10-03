# Paper 1 — Protocol-Accurate Sensor FMUs

Draft manuscript: *Protocol-Accurate Sensor FMUs for Sensor-in-the-Loop
Co-Simulation of Embedded GNC Firmware.*

## Status

Second draft, synchronised with the repository as of the MMC5983MA work
(`doc/api/sensors_mag.rst`, `doc/rocket_gps_ecos_cosim.rst`). The draft now
covers three transport classes (talker UDP, polled SPI, polled I²C), two
fidelity tiers, and end-to-end validation of **four** sensors rather than two —
GPS, IMU, the Bosch BMP390 and the MEMSIC MMC5983MA. 32 pages in the single-column review layout.

Formatted for **Simulation Modelling Practice and Theory** (SMPT, Elsevier)
with the `elsarticle` class: single-column `preprint,12pt` layout with line
numbers for review, numbered references (`elsarticle-num`), highlights,
keywords and Elsevier's end-matter declarations. The earlier IEEEtran
conference version (aimed at SIMULTECH / SIMPAC) is in git history.

## Files

| File | Purpose |
|---|---|
| `main.tex` | The manuscript (`elsarticle`, SMPT). |
| `highlights.txt` | The highlights as Elsevier's separate upload file. Keep it in step with the `highlights` environment in `main.tex`. |
| `references.bib` | Bibliography. BibTeX has no inline comment syntax, so the entries still needing author/venue/year checked against the primary source are listed under "Open items" below. |
| `figures/` | PNGs produced by the example's `plot_results.py` from the reference run in `data/`. |
| `data/` | Provenance of the reference run: configuration, console transcripts, and `run_statistics.py`. |

## Building

Needs a LaTeX distribution (TeX Live / MiKTeX) with `elsarticle` and `lmodern`
(standard in both):

```
pdflatex main
bibtex main
pdflatex main
pdflatex main
```

or, if `latexmk` is available:

```
latexmk -pdf main.tex
```

## Open items before submission

- [ ] **Verify citations against primary sources** — especially NASA
      TM-2015-218675 (confirm exact title/authors), the FMI intro paper, the
      co-simulation survey, the Renode reference, and the two datasheet entries
      added with the register-accurate parts: `bosch_bmp390` (document
      number/revision date) and `memsic_mmc5983ma` (revision letter/date).
- [x] Reformat for the venue: done for SMPT (`elsarticle`).
- [x] **Re-sync the draft with the repository** (2026-10-02, commit `55a95ce`,
      Aetherion 0.16.0 plant). The scenario was re-run paced (`--rtf 1 --seed 17`)
      and once more unpaced with the same seed; all ten figures, every count and
      every statistic in Sections 5-6 now come from those two runs. Provenance is
      in `data/`: the run configuration, both console transcripts, and
      `run_statistics.py`, which computes the quoted numbers from a results
      directory. Claims that changed as a result:
      - GPS decoded RMS is 2.18 m / 3.08 m, within one standard error of the
        expected 2.12 m / 3.0 m - not "to the centimetre", which was one
        seed's luck. Table 5 now covers IMU and magnetometer channels too.
      - The uncalibrated-magnetometer heading error is reported as a
        distribution (median 34 deg) because this seed's bridge offset is a
        2.4 % tail draw (74-83 deg).
      - The BMP390 floor is not constant: 9.9-11.8 kPa as die temperature
        moves.
      - Transport counts are from one run, not three different ones.
- [x] Items closed on 2026-10-03: the plant version is read from the FMU
      (0.16.0); line coverage is 85.9 % from Codecov at `55a95ce` (the old
      function/branch figures are gone, Codecov does not publish them); the
      "2000 km east" error is corrected in `doc/rocket_gps_ecos_cosim.rst` and
      `plot_results.py` (it is ~540 km); the MMC5983MA FMU comment now says
      200 ms. Related Work now credits ArduPilot SITL's protocol-level GPS and
      register-level I2C devices and reframes the novelty as "standard FMUs,
      importable anywhere, co-solved with an external plant". The IMU is called
      what it is: register-accurate but synthetic. Figure 1 is a drawn
      architecture figure (TikZ); Figure 2 is the plant-envelope check. The
      Discussion defends the open loop and reports cost (21.1 s per 200 s of
      flight unpaced; a 10x IMU rate costs nothing measurable). The flight
      computer harness gained `--fault imu-range | mag-skip-conditioning |
      mag-leave-reset`, and Section 6.7 reports the three injected faults.
- [ ] Still open: stepping the consumer on the importer's clock (a
      flight-computer FMU), which is also the precondition for closing the
      loop; Monte Carlo over seeds for the GPS RMS; vector versions of the
      PNG figures.
- [ ] Fill in or confirm every `% TODO` in the end matter of `main.tex`:
      CRediT roles, competing interests, funding, acknowledgements, and the
      generative-AI declaration (delete it if no such tools were used).
- [ ] Archive a tagged release (e.g. Zenodo) and cite its DOI in the Data
      availability statement instead of the live repository.
- [ ] Re-check the SMPT Guide for Authors at submission time for limits that
      could not be confirmed here (abstract kept at 250 words, Elsevier's
      default; 1-7 keywords; highlights of at most 85 characters).
- [x] Add a **related-work novelty scan** (ArduPilot/PX4 done 2026-10-03; SystemC/TLM, QEMU, virtual-ECU and DCP citations still to add) — the current Related Work section is
      argued but lightly cited; a proper literature search should confirm the
      "no prior protocol-accurate sensor FMU" claim and add citations.
- [x] Consider adding a **results table** of per-sensor RMS (Table 5, 2026-10-02) (decoded vs injected)
      to complement the per-sensor figures. The GPS row exists
      (`tab:validation`); the IMU, BMP390 and MMC5983MA rows would need a
      residual analysis the example's logs already support.
- [ ] The **radar altimeter is still unvalidated end-to-end** — it is the one
      sensor of the five with a model and no case-study run. Either wire it into
      a scenario that has ground beneath it or say so explicitly in
      Section "Limitations".
- [ ] Re-run `swil.mag_logger` once WSL is recovered (see the repo's `TODO.md`)
      and promote the Discussion's magnetometer SWIL sentence from "written but
      not yet executed" to a measured result.
- [ ] Regenerate figures at higher DPI / as PDF if the venue prefers vector art.
- [ ] Page-count pass against SMPT's limit, if it sets one.
