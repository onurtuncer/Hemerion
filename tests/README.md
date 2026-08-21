# tests/

Cross-cutting integration and SWIL tests. Unit tests that test a single module in isolation live inside that module's own `test/` subdirectory. This directory is for tests that span multiple modules, test the full RTOS task graph, or require Renode SWIL to exercise real firmware behaviour.

---

## Test categories

`tests/CMakeLists.txt` adds each of these only `if(EXISTS ...)`, so a category that was never written is skipped in
silence. Only `swil/` is written:

| Directory | Status | What it tests | Requires |
|---|---|---|---|
| `swil/` | **present** | Full firmware running in Renode, observable via pyrenode3 | Renode + pyrenode3 |
| `unit/` | **planned** | Cross-module interfaces with no RTOS. Per-module unit tests live in each module's own `test/` and do exist | Native host only |
| `integration/` | **planned** | Multi-module message flows, queue wiring | Native host only |
| `fmu/` | **planned** | FMI variable naming and step correctness. Packaging *is* covered — `generateFMU()` registers an `fmu`-labelled archive-layout test per FMU, which is what `ctest --preset fmu-native` runs | `fmu-native` build |

---

## Running tests

```bash
# Native unit + integration tests (fast, no Renode)
cmake --preset test-native
cmake --build --preset test-native
ctest --preset test-native --output-on-failure

# SWIL tests (requires Renode on PATH)
cmake --preset test-swil
cmake --build --preset test-swil

# The BMP390 SWIL loop also needs two *native* host tools. test-swil is a cross
# build and sim/ is host-only, so it cannot produce them; build them separately:
cmake --preset test-native
cmake --build --preset test-native --target i2c_shm_tcp_bridge bmp390_shm_peripheral

# HEMERION_SWIL_STRICT makes a missing artefact fail instead of skip -- see below
HEMERION_SWIL_STRICT=1 ctest --preset test-swil -L swil --output-on-failure

# FMU tests
cmake --preset fmu-native
cmake --build --preset fmu-native
ctest --preset fmu-native -L fmu --output-on-failure
```

---

## SWIL test harness (`swil/`)

SWIL tests use `pyrenode3` (Renode's Python bindings, hosted in-process via
pythonnet/CoreCLR) to launch a Renode machine, load the firmware ELF, and
observe behaviour over its UART. Each test is a Python file consumed by
pytest via the `add_test(... COMMAND pytest ...)` entry in
`tests/swil/CMakeLists.txt`.

```
tests/swil/
├── CMakeLists.txt     # find_package(Renode)/find_package(Pyrenode3) gate + add_test
├── requirements.txt    # pytest + pyrenode3 (pinned commit), for the venv described below
├── conftest.py          # renode_machine fixture, firmware_elf(), missing()
├── test_led_blink.py   # apps/led_blink smoke test: watches usart3 for "LED ON"/"LED OFF"
└── test_baro_logger.py # apps/baro_logger across the full I2C chain (see below)
```

The `renode_machine` fixture in `conftest.py`:
1. Sets `$repl` to `sim/renode/boards/nucleo_h743zi2.repl` (absolute path — `execute_script` resolves relative paths against Renode's own install root, not the repo).
2. Runs `sim/renode/scripts/swil_lockstep.resc`, which creates the machine and loads that platform.
3. Yields the `Machine` handle to the test.
4. On teardown, calls `Emulation().clear()`.

The test itself still has to `load_elf()` its own firmware and start the
emulation — the fixture only brings up the bare platform:

```python
from pyrenode3.wrappers import Emulation, TerminalTester
from conftest import firmware_elf, peripheral

def test_led_blinks(renode_machine):
    renode_machine.load_elf(str(firmware_elf("led_blink")))

    tester = TerminalTester(peripheral(renode_machine, "sysbus.usart3"), timeout=5.0)
    Emulation().StartAll()

    assert tester.WaitFor("LED ON", None, False, False, False, False) is not None
```

`firmware_elf()` locates the ELF in the build tree that registered the test:
`tests/swil/CMakeLists.txt` passes `HEMERION_SWIL_FIRMWARE_DIR` pointing at
that build's `apps/`, and failing that any `build/*/apps/<app>/<app>` is
searched. Candidates are checked for an `EM_ARM` ELF header, so a
`native_linux` build of the same app name is never handed to Renode.

When nothing is found it **skips** rather than fails — `tests/swil` runs
against whatever was last built and does not drive builds itself. Set
`HEMERION_SWIL_STRICT=1` to invert that:

> A skipped pytest is a **passed** ctest. Both SWIL tests once skipped
> silently in CI — `firmware_elf()` looked under `build/renode-h743/` while
> the job builds the `test-swil` preset into `build/test-swil/` — so the
> whole suite reported success having run nothing. CI now sets
> `HEMERION_SWIL_STRICT=1`, which turns every "not built" skip into a
> failure naming the build step that is missing. Route any new
> skip-on-missing path through `conftest.missing()` so it is covered too.

### Setup: Renode + pyrenode3

`pyrenode3` hosts Renode via pythonnet/CoreCLR in-process, which requires
Renode's **.NET 8 / CoreCLR** Linux build. The Windows Renode installer ships
a .NET-Framework build that pyrenode3 cannot load — on a Windows dev machine,
run the harness from **WSL2** instead (matches the Ubuntu + Renode container
CI uses). Cross-compiling the firmware itself still happens on the Windows
host with the ARM toolchain already set up there; WSL only needs Renode and
the pyrenode3 venv.

One-time setup inside the WSL2 distro:

```bash
# Renode (.deb ships the CoreCLR build; /usr/bin/renode is a launcher
# wrapping `dotnet /opt/renode/bin/Renode.dll`)
wget https://github.com/renode/renode/releases/download/v1.16.1/renode_1.16.1_amd64.deb
sudo apt install ./renode_1.16.1_amd64.deb

python3 -m venv ~/swilvenv
~/swilvenv/bin/pip install -r tests/swil/requirements.txt
```

pyrenode3's loader needs to be pointed at Renode's *build output*
(`/opt/renode/bin`), not the `/usr/bin/renode` shell launcher, which it
can't introspect:

```bash
export PYRENODE_BUILD_DIR=/opt/renode
export PYRENODE_BUILD_OUTPUT=bin
```

### Running

Build the firmware on Windows first (the WSL side only runs tests, it
doesn't cross-compile):

```powershell
cmake --build --preset renode-h743 --target led_blink
```

Then, from WSL2, against the repo on the Windows drive (`/mnt/d/...`):

```bash
cd /mnt/d/Dev/Hemerion/tests/swil
~/swilvenv/bin/python3 -m pytest test_led_blink.py -v
```

`test_baro_logger.py` additionally needs the two `sim/i2c_shm/tools` binaries
built **for the OS pytest runs under** — Linux ones, in this split, not the
Windows build. Build them once inside WSL:

```bash
cd /mnt/d/Dev/Hemerion
cmake --preset test-native
cmake --build --preset test-native --target i2c_shm_tcp_bridge bmp390_shm_peripheral
```

They are then found by the `build/*/sim/i2c_shm/tools/` glob, or point
`HEMERION_SWIL_TOOLS_DIR` at them explicitly.

`tests/swil/CMakeLists.txt`'s `find_package(Renode)` / `find_package(Pyrenode3)`
gate means a `cmake --preset test-swil` configure on Windows (where Renode
is the incompatible .NET-Framework build) silently skips `tests/swil`
rather than failing — the `ctest --preset test-swil` path is meant for a
single Linux environment (CI's Ubuntu + Renode container) that has both the
ARM toolchain and a CoreCLR Renode build; it isn't wired to span the
Windows-build / WSL-test split above.

### The BMP390 loop (`test_baro_logger.py`)

`test_led_blink.py` needs nothing but the firmware. `test_baro_logger.py`
drives every layer between the emulated I2C controller and a
register-accurate device model:

```
baro_logger (emulated) -> Renode STM32F7_I2C -> HemerionI2cShmBridge (C#)
    -> TCP -> i2c_shm_tcp_bridge -> sim/i2c_shm -> bmp390_shm_peripheral
```

so the test also supervises two host processes. Points worth knowing:

* **The two host tools are native binaries.** `sim/` is host-only and
  `HEMERION_BUILD_SIM` hard-errors under a cross toolchain, so the
  `test-swil` build that registers this test can never produce them. They
  come from `test-native`; `HEMERION_SWIL_TOOLS_DIR` overrides the search.
* **No fixed TCP port.** The bridge is launched with `--port 0` and reports
  the port it actually bound on stdout; the test reads it back and writes a
  small platform overlay naming it. That is why there is no `bmp390` node in
  `sim/renode/boards/nucleo_h743zi2.repl` — see
  `sim/renode/i2c_bridge/DESIGN.md`.
* **`targetAddress` is stated twice in that overlay, on purpose.** Renode
  matches the transfer on the registration address but the C# class puts
  `targetAddress` on the shm bus, and it has no default — so a missing one
  is a load-time error rather than a silent mismatch.
* **Stop the peripheral politely.** `bmp390_shm_peripheral` unlinks its
  shared-memory segment only on a normal return from `main()`, and handles
  SIGINT/SIGTERM so that path is reachable. A SIGKILL strands
  `/dev/shm/<bus>` until the machine reboots.
* **Helper failures are reported as themselves.** The test waits for the
  bridge to accept a connection before starting the emulation and checks
  both helpers' liveness as it goes, quoting their captured output — a stale
  segment or a taken port surfaces as that, not as firmware which failed to
  print its banner.

---

## FMU tests

**What runs today** is packaging verification, registered automatically by `generateFMU()` — one `fmu`-labelled ctest
per model per FMI version (`cmake/fmu_archive_test.cmake`, `cmake/verify_fmu_archive.cmake`). Each unzips the archive
and checks `modelDescription.xml` is at its root, `binaries/` beside it holds the library, and the `modelIdentifier`
inside the description matches. `ctest --preset fmu-native` runs exactly these, with `noTestsAction` set to `error` so
the preset cannot pass by matching nothing — which is what it did before those tests existed.

**Planned, in a `tests/fmu/` that does not exist yet** — everything requiring an FMI *importer*:
- Loads without error via fmi4c
- Exposes the expected variable names and causalities
- Produces numerically correct output for a known input sequence
- Handles `fmi2Reset` and re-instantiation without memory leaks

Numerical behaviour is meanwhile exercised end-to-end by `examples/rocket_gps_ecos`, which flies the GPS, IMU and
BMP390 FMUs under Ecos.

---

## CI matrix

| Preset | Runner | Trigger |
|---|---|---|
| `test-native` | Ubuntu 24.04 | Every push |
| `fmu-native` | Ubuntu 24.04 | Every push |
| `renode-h743` (build only) | Ubuntu 24.04 | Every push |
| `test-swil` | Ubuntu 24.04 + Renode container | Every PR, and pushes to `main` (`swil.baro_logger` informational — see below) |

SWIL used to run only *after* a PR merged, so a broken loop was discovered on
`main` rather than on the PR that broke it — four consecutive runs failed at
`actions/checkout` without anyone noticing. It now runs on every PR like the
other build jobs; Renode startup adds ~30 s, which is cheap against that
blind spot. The job is `concurrency`-grouped, so a new push to a PR cancels
the superseded run.

`swil.led_blink` gates; `swil.baro_logger` runs in a `continue-on-error` step
for now, the way IWYU, Metrix++ and the examples clang-tidy pass do. It fails
for a reason that predates the harness: `baro_logger` hangs before its first
print and emits *no* usart3 output at all, while `led_blink` prints happily on
the same platform — so the emulated I2C path is what needs fixing, not the
test. Note this is a `continue-on-error` step and **not** a skip: a skipped
pytest is a passed ctest, which is how this suite spent months green while
testing nothing. The test still runs, still reports, and still captures the
Renode log and both helper logs on failure. Drop `continue-on-error` from
`swil.yml` once the emulated I2C path works.
