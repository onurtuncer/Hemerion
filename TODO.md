# TODO

Work that is written but not yet closed out. Each entry says what is already
verified, so nobody repeats it, and what specifically remains.

---

## Run `swil.mag_logger` end to end

**Status:** written, registered, and unverified. Everything underneath it is
verified; the pytest itself has never executed.

`tests/swil/test_mag_logger.py` is the SWIL loop for the MMC5983MA:
emulated firmware → Renode `STM32F7_I2C` → the runtime-loaded
`HemerionI2cShmBridge` C# peripheral → TCP → `i2c_shm_tcp_bridge` →
shared-memory bus → `Mmc5983maI2cSlave`. It could not be run when it was
written: `pyrenode3` is not installed on the Windows side, and WSL — where
these tests run, per the Windows-build/WSL-test split — was returning
`Wsl/Service/E_UNEXPECTED` on every invocation. Recovering WSL is the only
blocker.

### Already verified, do not redo

* `mag_logger` cross-compiles and links for the H743 (51 KB flash, no
  warnings from its own sources).
* `mmc5983ma_shm_peripheral` and `i2c_shm_tcp_bridge` both build, launch
  together, create/attach the bus, and print the banners the harness parses.
* Every regex in the harness (`OFFSET_PATTERN`, `FIELD_PATTERN`,
  `PERIPHERAL_OFFSET_PATTERN`, the bound-port pattern) checked against the
  real captured output of those two tools.
* `test_i2c_shm_mmc5983ma` passes: the same driver, the same device model and
  the same blocking SET/RESET handshake across the real shared-memory
  transport — everything except the Renode, C# and TCP legs.
* The pytest imports cleanly and every `conftest` helper it names exists.

The unverified span is therefore only Renode → C# bridge → TCP, which is
unchanged shared code already exercised by `swil.baro_logger`.

### To close it

Inside WSL, with `pyrenode3` installed (`tests/swil/requirements.txt`) and the
host tools built there as *Linux* binaries — `host_tool()` requires an
executable under `build/*/sim/i2c_shm/tools/`, which a cross build cannot
produce:

```sh
cmake --build --preset test-native --target i2c_shm_tcp_bridge mmc5983ma_shm_peripheral
cmake --build --preset test-swil  --target mag_logger
HEMERION_SWIL_STRICT=1 ctest --test-dir build/test-swil -R swil.mag_logger --output-on-failure
```

`HEMERION_SWIL_STRICT=1` matters: without it a missing artefact is a pytest
skip, and ctest scores a skip as a pass.

### What a failure would mean

The load-bearing assertion is not the field values — it is that the bridge
offset the firmware prints matches the one the peripheral independently
reports having been born with. Firmware that skipped the SET/RESET pair
entirely still prints plausible-looking field numbers, wrong by roughly the
size of Earth's field, and only that comparison separates the two. So an
offset mismatch means one of the five layers did not carry the blocking,
multi-transaction handshake intact; a *sign-flipped* field with a correct
offset means the part was left RESET rather than SET.

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
