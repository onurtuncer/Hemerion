# Paper 1 — Protocol-Accurate Sensor FMUs

Draft manuscript: *Protocol-Accurate Sensor FMUs for Sensor-in-the-Loop
Co-Simulation of Embedded GNC Firmware.*

## Status

Second draft, synchronised with the repository as of the MMC5983MA work
(`doc/api/sensors_mag.rst`, `doc/rocket_gps_ecos_cosim.rst`). The draft now
covers three transport classes (talker UDP, polled SPI, polled I²C), two
fidelity tiers, and end-to-end validation of **four** sensors rather than two —
GPS, IMU, the Bosch BMP390 and the MEMSIC MMC5983MA. 10 pages.

Written to the **IEEEtran** two-column conference class per request. Target venue is **SIMULTECH / SIMPAC** (SCITEPRESS), which actually
ships its own `scitepress` class — swapping is mechanical (change
`\documentclass`, move the author block); the content and structure carry over.
For a double-blind SCITEPRESS submission, anonymize the author block and any
first-person repo references.

## Files

| File | Purpose |
|---|---|
| `main.tex` | The manuscript (IEEEtran conference). |
| `references.bib` | Bibliography. BibTeX has no inline comment syntax, so the entries still needing author/venue/year checked against the primary source are listed under "Open items" below. |
| `figures/` | PNGs copied from `doc/_static/rocket_gps_ecos/`. Regenerate via the example's `plot_results.py`. |

## Building

Needs a LaTeX distribution (TeX Live / MiKTeX) with `IEEEtran.cls` (standard in
both):

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
- [ ] Decide venue class (IEEEtran vs SCITEPRESS) and reformat accordingly.
- [ ] Add a **related-work novelty scan** — the current Related Work section is
      argued but lightly cited; a proper literature search should confirm the
      "no prior protocol-accurate sensor FMU" claim and add citations.
- [ ] Consider adding a **results table** of per-sensor RMS (decoded vs injected)
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
- [ ] Fill in `\section*{Acknowledgment}` (funding/institutional).
- [ ] Regenerate figures at higher DPI / as PDF if the venue prefers vector art.
- [ ] Page-count pass once the venue and its limit are fixed.
