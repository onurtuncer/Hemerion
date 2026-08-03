# Journal companion — extension roadmap

The conference paper targets **SIMULTECH / SIMPAC**. The companion targets a
journal — primary **Simulation Modelling Practice and Theory (SMPT, Elsevier)**,
which has a standing invited-extension pipeline from SIMULTECH; fast open-access
fallback **MDPI *Aerospace***.

A journal version must add ~30–40% genuinely new material over the conference
paper, cite it explicitly, and avoid self-plagiarism. Items 1 and 2 below alone
justify the extension; the rest deepen it.

## Extension delta

- [ ] **1. Validate all five sensors end-to-end, not just GPS+IMU.** Baro, radar
  altimeter, and magnetometer have models but no case-study validation yet.
  Running them gives the "one pattern, five parts" breadth a journal wants.

- [ ] **2. Quantify SIL→HWIL transfer.** Run the same decoder in Renode and on
  the physical STM32H743, showing the byte-level results carry across tiers.
  This is the strongest possible validity claim and is currently asserted, not
  measured.

- [ ] **3. Richer error models.** Correlated in-run bias (Gauss-Markov),
  scale-factor / misalignment, and the soft-iron term deliberately deferred —
  each plugged into the same `sat(round(·))` form.

- [ ] **4. Deeper FMI-LS-BUS comparison.** Possibly an actual head-to-head on a
  bus both can express, turning the three-axes argument into a measured
  contrast.

- [ ] **5. Citable reproducibility artifact.** The JOSS paper discussed earlier,
  referenced from the journal version to give it a DOI.
