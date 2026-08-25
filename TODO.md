# TODO

Work that is written but not yet closed out. Each entry says what is already
verified, so nobody repeats it, and what specifically remains.

---

## Smaller loose ends, all flagged but unactioned

* **A failing `assert()` hangs the test suite on Windows** instead of failing.
  The CRT answers `abort()` with a modal dialog, so in a headless CI job a
  one-line assertion failure reads as a hung runner — it cost a 180 s ctest
  stall once. `modules/sensors/test/test_mmc5983ma.cpp` has a
  `fail_fast_instead_of_blocking()` guard (verified: exit 3 immediately
  instead of hanging); every sibling test in that directory has the same
  exposure and no guard. Worth lifting into shared test scaffolding rather
  than copying nine times.

* **No `.gitattributes`.** With `core.autocrlf=input` a local CRLF write
  normalises at commit, which is why CI never saw the CRLF that appeared in
  `cmake/generate_fmu.cmake`. A contributor without that setting would commit
  CRLF and — now that `cmake_lint` actually scans files — turn the job red on
  `C0327`. A `* text=auto` line closes it, at the cost of a one-time
  renormalisation diff.

* **MMC5983MA self-test is register-modelled but not magnetically modelled.**
  `Internal control 3`'s `St_enp`/`St_enm` are stored so writes do not fault,
  but the self-test coil's extra field never appears in a measurement, so a
  driver self-test would pass vacuously against the FMU. Documented in
  `mmc5983ma_i2c_slave.cpp`. Build any self-test against hardware, or model
  the coil field first.

* **`E1126` is disabled in `.cmake-format`** because cmakelang 0.6.13 predates
  `file(REAL_PATH)` and `file(ARCHIVE_EXTRACT)`. Drop the entry when the pin
  in `.github/workflows/cmake_lint.yml` next moves.
