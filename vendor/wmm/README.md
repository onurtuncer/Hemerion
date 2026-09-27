# World Magnetic Model 2025 coefficients

Third-party data, vendored verbatim. Nothing here is Hemerion's work and
nothing here should be edited — see *Updating* below.

## Provenance

| | |
|---|---|
| Source | <https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip> |
| Linked from | <https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients> |
| Retrieved | 2026-09-27 |
| Archive SHA-256 | `2e76569370d081f2cd7919490218bd094ca9afde347b198eff5621e0af460d03` |

Files extracted from that archive and kept here:

| File | SHA-256 |
|---|---|
| `WMM.COF` | `dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f` |
| `WMM2025_TestValues.txt` | `e6975b093dddeb6153e0b23cc418425c438167e7c5b1dd795da379cb654f5819` |

`WMM.COF` is byte-identical to the archive's `WMM2025.COF`; only one copy is
kept. The archive's `README-WMM-COEFS.txt` and the test-value PDF are not
vendored — the installation instructions do not apply to this repository, and
the PDF duplicates the `.txt`.

## Why the file is here at all

Because these numbers must not be transcribed. A spherical-harmonic model is
90 lines of coefficients that no reviewer can eyeball for correctness, and a
single mistyped digit produces a field that looks entirely plausible and is
wrong. Same reason the turbulence work refused to transcribe MIL-F-8785C's
exceedance chart from memory. The file is fetched once, checksummed, and
committed; `tools/generate_wmm_coefficients.py` turns it into the C++ header
the model compiles against, so the generated table has a provenance chain back
to NOAA rather than to anyone's recollection.

`WMM2025_TestValues.txt` is NOAA's own set of 100 reference field values
spanning the model's epoch range, altitudes and latitudes. It is what
`sensors.world_magnetic_model` checks against, so the implementation is
validated against the model's authors rather than against itself.

## Model

* **Epoch** 2025.0, released 2024-11-13, **valid to 2030.0**. The
  implementation refuses dates outside that window rather than extrapolating
  quietly; see `WorldMagneticModel`.
* Degree and order 12, 90 coefficient lines, terminated by two lines of 9s.
* Coefficients in nT, secular variation in nT/year.

## Terms

The WMM and its coefficients are a work of the U.S. Government and are **in the
public domain**: NCEI states that "the WMM source code is in the public domain
and not licensed or under copyright."

Attribution is nevertheless required of works incorporating it, per 17 U.S.C.
403, and NCEI asks that model values be cited as:

> NOAA NCEI Geomagnetic Modeling Team; British Geological Survey. 2024: World
> Magnetic Model 2025.

The model is produced jointly by NOAA's NCEI and the British Geological
Survey, for the U.S. National Geospatial-Intelligence Agency and the U.K.
Defence Geographic Centre.

## Updating

WMM2025 expires at the end of 2029. To move to its successor:

1. Download the new archive from the NCEI coefficients page, and record its
   URL, date and SHA-256 in the table above.
2. Replace `WMM.COF` and the test values with the new ones.
3. Re-run `python tools/generate_wmm_coefficients.py` to regenerate
   `modules/sensors/include/Hemerion/mag/wmm_coefficients.h`.
4. Expect `sensors.world_magnetic_model` to still pass — it checks against the
   new model's own test values — and expect `examples.geomagnetic_field` to
   need its declination assertion revisited, since that one is pinned to a
   value on purpose.

Do not hand-edit either file. `python tools/generate_wmm_coefficients.py
--check` fails if the generated header has drifted from the `.COF`, and it
runs in CI.
