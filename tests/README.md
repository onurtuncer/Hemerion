# tests/

Cross-cutting integration and SWIL tests. Unit tests that test a single module in isolation live inside that module's own `test/` subdirectory. This directory is for tests that span multiple modules, test the full RTOS task graph, or require Renode SWIL to exercise real firmware behaviour.

---

## Test categories

| Directory | What it tests | Requires |
|---|---|---|
| `unit/` | Individual module interfaces, no RTOS | Native host only |
| `integration/` | Multi-module message flows, queue wiring | Native host only |
| `swil/` | Full firmware running in Renode, observable via pyrenode3 | Renode + pyrenode3 |
| `fmu/` | FMU packaging, FMI variable naming, step correctness | `fmu-native` build |

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
ctest --preset test-swil -L swil --output-on-failure

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
├── conftest.py          # renode_machine fixture: loads the platform, yields the Machine
└── test_led_blink.py   # apps/led_blink smoke test: watches usart3 for "LED ON"/"LED OFF"
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

`firmware_elf()` skips (rather than fails) when the target app hasn't been
built under `build/renode-h743/` yet — `tests/swil` runs against whatever
that preset most recently produced; it doesn't drive the cross-build itself.

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

`tests/swil/CMakeLists.txt`'s `find_package(Renode)` / `find_package(Pyrenode3)`
gate means a `cmake --preset test-swil` configure on Windows (where Renode
is the incompatible .NET-Framework build) silently skips `tests/swil`
rather than failing — the `ctest --preset test-swil` path is meant for a
single Linux environment (CI's Ubuntu + Renode container) that has both the
ARM toolchain and a CoreCLR Renode build; it isn't wired to span the
Windows-build / WSL-test split above.

---

## FMU tests (`fmu/`)

Verify that each module FMU:
- Loads without error via fmi4c
- Exposes the expected variable names and causalities from `model_description.xml`
- Produces numerically correct output for a known input sequence
- Handles `fmi2Reset` and re-instantiation without memory leaks (checked with ASan on Linux)

---

## CI matrix

| Preset | Runner | Trigger |
|---|---|---|
| `test-native` | Ubuntu 24.04 | Every push |
| `fmu-native` | Ubuntu 24.04 | Every push |
| `renode-h743` (build only) | Ubuntu 24.04 | Every push |
| `test-swil` | Ubuntu 24.04 + Renode container | PR merge to `main` |
| `cross-stm32f446` (build only) | Ubuntu 24.04 | Every push |

SWIL tests are gated to PR merge because Renode startup adds ~30 s per test suite.
