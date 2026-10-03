# Hemerion publication arc

Written 2026-08-03. Sequencing plan for the paper program; per-paper detail lives
in each paper's own directory.

## The through-line

One claim runs under every paper here:

> **Protocol accuracy is the invariant that lets a single firmware codebase move
> unchanged from desktop co-simulation, to instruction-set emulation, to
> hardware — and the fidelity it buys is measurable, not asserted.**

Papers are cross-sections of that program, not separate efforts:

- **P1 / P1J** establish the mechanism (sensor FMUs emit byte-exact wire frames).
- **P2** establishes the consequence (one codebase, three execution tiers).
- **P4a** establishes the payoff (protocol accuracy catches real firmware defects
  that ideal-signal simulation cannot).
- **P3** makes the whole thing citable and reproducible.

P4a is the scientific justification for P1–P2. Without it the program describes a
technique; with it, the technique is shown to matter. It should not be treated as
optional tail-end work.

## Track A — simulation infrastructure (write-ups of work that exists)

### P1 — Protocol-accurate sensor-hardware-simulator FMUs
- **Venue:** SIMULTECH / IEEE SIMPAC (SCITEPRESS, double-blind, 10–12pp)
- **Status:** full draft in `paper1-protocol-accurate-sensor-fmus/`
- **Blocking:** citation verification (`% VERIFY` marks), venue class decision
  (IEEEtran → SCITEPRESS is mechanical), vector figures, page pass,
  author anonymization
- **Depends on:** nothing. Submittable inside ~4 weeks of writing effort.

### P1J — Journal companion
- **Venue:** Simulation Modelling Practice and Theory (Elsevier) primary — has a
  standing invited-extension pipeline from SIMULTECH. Fallback: MDPI *Aerospace*.
- **Delta (≥30–40% new):** roadmap in
  `paper1-protocol-accurate-sensor-fmus/TODO.md`; items 1 and 2 carry it.
- **Depends on:** the measurement campaign (below).

### P2 — Single-codebase tri-modal execution (Native / Renode SWIL / HWIL)
- **Venue:** ACM TECS (journal, best fit) — or EMSOFT/DATE if a conference slot is
  wanted. EMSOFT is a hard bar for a single-system paper; TECS is the realistic
  primary.
- **Contribution:** the transport-abstraction layer (`udp_bridge`, `shm_bridge`,
  `spi_shm`) as the thing that makes tier-swapping free, plus a *measured*
  characterisation of what each tier costs and preserves.
- **What is missing:** the measurement. The plumbing exists; the numbers do not.
- **Depends on:** the measurement campaign (below). Can stand alone from P1 but
  reads better citing it.

### P3 — JOSS software paper
- **Cost:** low. Needs a tagged release, install story, test coverage, docs.
- **Value:** a DOI to cite from P1J and P2, disproportionate to its cost.
- **Do it:** in parallel with P1J writing, not before P1 is out.

## The measurement campaign (the critical path)

P1J and P2 both block on the *same* body of work. Do it once, write it twice.

1. Run baro, mag, radalt end-to-end in a case study — currently modelled but never
   validated. Gives P1J its "one pattern, five parts" breadth.
2. Run the identical decoder in three tiers: native, Renode, physical STM32H743.
   Show byte-level results carry across. This is P1J item 2 *and* P2's core
   evidence.
3. Instrument timing/determinism per tier — P2-specific, cheap once (2) is wired.
4. Richer error models (Gauss–Markov bias, scale/misalignment, soft-iron) —
   P1J only, lowest priority of the four.

**This campaign is the single largest schedule risk in the program**, because it
requires working physical STM32H743 hardware and two papers depend on it. If
hardware access is uncertain, resolve that *before* committing P1J to an invited
extension deadline.

## Track B — GNC application (engineering projects, not write-ups)

`modules/gnc`, `modules/actuators`, `modules/datalogger` and `apps/gnc_flight`
are empty. Nothing in this track is a writing task yet.

### P4a — What protocol accuracy buys you *(recommended next application paper)*
- **Thesis:** a navigation filter validated against byte-exact sensor frames
  catches defect classes that ideal-signal testing misses — dropout handling,
  saturation, quantisation, checksum/sync loss, stale-frame reuse.
- **Why this one first:** needs an EKF fusing the five existing FMUs. It does
  *not* need guidance, control, or actuators. Far cheaper than P4, and it is the
  paper that makes the whole program matter.
- **Leverage:** `modules/fault` (818 LOC) already exists; protocol-level fault
  injection at the FMU boundary is nearly free and turns this into a strong
  results section.
- **Venue:** AIAA SciTech → JGCD, or a fault-tolerance venue (DASC) if the
  injection results dominate.

### P4 — Onboard convex guidance
- **Blocked on:** implementing `modules/gnc` essentially from zero — guidance,
  control, actuator interfaces. Realistically 6–12 months of engineering.
- **Venue:** AIAA SciTech → JGCD.
- **Naming hazard:** `examples/rocket_gps_ecos` refers to **Ecos**, the
  *co-simulation master* (Ecos-platform/ecos) — not **ECOS**, the embotech SOCP
  solver a convex-guidance paper would use. Reviewers will conflate these.
  Rename the example or disambiguate explicitly before P4 is drafted.

## Sequencing

| Phase | Work | Output |
|---|---|---|
| 0 (now, ~4wk) | Finish P1: citations, venue class, figures, anonymization | P1 submitted |
| 1 (~2–3mo, parallel) | Measurement campaign items 1–3 | Evidence base |
| 2 | Write P1J and P2 off the campaign; P3 alongside | 3 submissions |
| 3 | Build `modules/gnc` nav filter; fault-injection study | P4a |
| 4 | Build guidance/control | P4 |

Phase 0 and Phase 1 can overlap — P1 is writing, the campaign is lab work.

## Standing risks

- **Double-blind vs public repo.** The Hemerion repo carries the author's name in
  git history. Any paper citing it de-anonymizes the submission. Prepare an
  anonymized artifact mirror before P1 goes out.
- **Self-plagiarism P1 → P1J.** Cite P1 explicitly; keep the delta above 30%.
- **Hardware single point of failure.** See campaign note above.
- **Track B is not a writing schedule.** Do not put P4 on a deadline calendar
  until `modules/gnc` compiles and flies in simulation.
