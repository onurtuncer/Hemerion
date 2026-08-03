# Paper 1 — Protocol-Accurate Sensor FMUs

Draft manuscript: *Protocol-Accurate Sensor FMUs for Sensor-in-the-Loop
Co-Simulation of Embedded GNC Firmware.*

## Status

First full draft. Written to the **IEEEtran** two-column conference class per
request. Target venue is **SIMULTECH / SIMPAC** (SCITEPRESS), which actually
ships its own `scitepress` class — swapping is mechanical (change
`\documentclass`, move the author block); the content and structure carry over.
For a double-blind SCITEPRESS submission, anonymize the author block and any
first-person repo references.

## Files

| File | Purpose |
|---|---|
| `main.tex` | The manuscript (IEEEtran conference). |
| `references.bib` | Bibliography. Entries marked `% VERIFY` need author/venue/year checked against the primary source before camera-ready. |
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

- [ ] **Verify all `% VERIFY` citations** — especially NASA TM-2015-218675
      (confirm exact title/authors), the FMI intro paper, the co-simulation
      survey, and the Renode reference.
- [ ] Decide venue class (IEEEtran vs SCITEPRESS) and reformat accordingly.
- [ ] Add a **related-work novelty scan** — the current Related Work section is
      argued but lightly cited; a proper literature search should confirm the
      "no prior protocol-accurate sensor FMU" claim and add citations.
- [ ] Consider adding a **results table** of per-sensor RMS (decoded vs injected)
      to complement the GPS/IMU figures.
- [ ] Fill in `\section*{Acknowledgment}` (funding/institutional).
- [ ] Regenerate figures at higher DPI / as PDF if the venue prefers vector art.
- [ ] Page-count pass once the venue and its limit are fixed.
