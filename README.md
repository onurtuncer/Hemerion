# HEMERION — Embedded Firmware Framework

[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Linux Build](https://github.com/onurtuncer/Hemerion/actions/workflows/linux.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/linux.yml)
[![Windows Build](https://github.com/onurtuncer/Hemerion/actions/workflows/windows.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/windows.yml)
[![Cross Build (ARM)](https://github.com/onurtuncer/Hemerion/actions/workflows/cross_build.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/cross_build.yml)
[![SWIL Tests](https://github.com/onurtuncer/Hemerion/actions/workflows/swil.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/swil.yml)
[![codecov](https://codecov.io/gh/onurtuncer/Hemerion/branch/main/graph/badge.svg)](https://codecov.io/gh/onurtuncer/Hemerion)
[![Clang-Format](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_format.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_format.yml)
[![Clang-Tidy](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_tidy.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_tidy.yml)
[![IWYU](https://github.com/onurtuncer/Hemerion/actions/workflows/iwyu.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/iwyu.yml)
[![CMake-Format](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_format.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_format.yml)
[![CMake-Lint](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_lint.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_lint.yml)
[![Metrix++](https://img.shields.io/badge/metrix%2B%2B-complexity%20report-blue)](https://onurtuncer.github.io/Hemerion/generated/metrixpp_report.html)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://onurtuncer.github.io/Hemerion/)

<p align="center">
  <img src="assets/logo.svg" width="180" alt="HEMERION Monogram"/>
</p>

**HEMERION** is a research-grade embedded firmware framework for
**aerospace GNC and real-time embedded systems**. It targets FreeRTOS on
STM32 microcontrollers and provides a modular architecture that supports
three build modes from a single codebase:

- **SWIL** — firmware running inside Renode simulation, no hardware required
- **HWIL** — firmware flashed to physical STM32 targets
- **Native** — modules compiled as x86 shared libraries for FMI 2.0 / 3.0 co-simulation with Aetherion

What ties the three together is **protocol accuracy**: a sensor's simulation
model is not an ideal signal source but a register-accurate part that answers
real bus traffic — the same BMP390 register map answers the emulated I2C
controller under Renode and the host FMI master under Ecos, and the same
on-target driver reads it in both. Firmware moves between tiers unchanged.

Hemerion is the firmware counterpart to
[Aetherion](https://github.com/onurtuncer/Aetherion), the C++23 host-side GNC
simulation framework. Together they form a lockstep co-simulation stack
targeting small launch vehicles.

The name *ΗΜΕΡΙΟΝ* (Hemerion) is inspired by the Greek word **Ἥμερος**,
meaning *tame* or *cultivated* — the disciplined, deterministic counterpart
to the boundless realm of *Aetherion*.

---

## ✨ Features

- **FreeRTOS task architecture** with deterministic scheduling on STM32
- **Protocol-accurate sensor stacks** — register-level part models paired with the on-target drivers that read them (BMP390, MMC5983MA, MEMS IMU, u-blox GPS, radar altimeter)
- **Renode SWIL simulation** on a Nucleo-H743ZI2 platform description, with a runtime-loaded I2C bridge that lets emulated firmware talk to a host-side part model
- **FMI 2.0 *and* 3.0 FMU export** from one set of sources via `generateFMU()` — no hand-written FMI plumbing
- **Simulated SPI and I2C buses in shared memory**, so the same sensor model serves the host co-simulation and the emulator
- **Embedded Template Library (ETL)** as an STL alternative for bare-metal targets
- **CMakePresets-based cross-compilation** toolchain for ARM Cortex-M targets
- **Post-modern C++23 firmware design**

---

## Sensor suite

`modules/sensors/` is the fullest-built module and the framework's reference
example of the one-module-three-targets idea. Each stack is a driver (runs on
the target), a raw→SI conversion layer (host-testable, no bus), and a
hardware-simulator FMU (host-only) sharing one register-map header with the
driver — so a wire-protocol change cannot land on only one side.

| Stack | Part | Bus / wire format | Notes |
|---|---|---|---|
| `baro/` | Bosch **BMP390** | I2C, register-accurate | Drives the full SWIL chain: emulated I2C → Renode bridge → `i2c_shm` → part model |
| `mag/` | MEMSIC **MMC5983MA** | I2C, register-accurate | SET/RESET offset cancellation; SWIL-tested via `mag_logger` |
| `imu/` | Generic MEMS IMU | SPI, command byte + auto-increment register map (ADIS16470 / ICM-42688 / BMI088 shape) | Simulator is an SPI peripheral on `sim/spi_shm` |
| `gps/` | u-blox-class receiver | UBX (`NAV-PVT`) and NMEA over UART/UDP | COCOM-limited; parsers and emitter unit-tested both directions |
| `radalt/` | Generic radar altimeter | Range + track-status word | Conversion and packet layers only |

See [`doc/sensor_models.rst`](doc/sensor_models.rst) for the model details and
[`modules/README.md`](modules/README.md) for per-module build status.

---

## Repository layout

```
hemerion/
├── CMakeLists.txt          # Root superbuild — includes all modules, bsp, sim
├── CMakePresets.json       # Named presets for cross / native / test builds
├── toolchains/             # CMake toolchain files (arm-none-eabi, x86_64)
├── cmake/                  # Shared find-modules and helper functions
│
├── modules/                # Reusable firmware libraries + FMU wrappers
│   ├── sensors/            # built  -- baro, mag, imu, gps, radalt (see above)
│   ├── rtos_core/          # built  -- task/queue registries, memory pools
│   ├── fault/              # built  -- fault registry, watchdog supervisor
│   ├── power/              # built  -- battery monitor, regulator sequencer
│   ├── comms/              # partial-- CAN framing only
│   ├── actuators/          # empty  -- no sources, no CMakeLists.txt
│   ├── gnc/                # empty  -- no sources, no CMakeLists.txt
│   └── datalogger/         # empty  -- no sources, no CMakeLists.txt
│
├── bsp/                    # Board support packages (one per target)
│   └── stm32h743_nucleo/   # the only BSP that exists; see bsp/README.md
│
├── sim/                    # Host-side simulation targets (never cross-compiled)
│   ├── renode/             # .repl/.resc platform + the HemerionI2cBridge peripheral
│   ├── fmi/                # FMI co-simulation master -- PLANNED, package.xml only
│   ├── shm_bridge/
│   ├── spi_shm/            # Simulated SPI bus in shared memory; the IMU FMU answers it
│   ├── i2c_shm/            # Simulated I2C bus + the host tools bridging it to Renode
│   └── udp_bridge/
│
├── vendor/                 # Third-party deps, submodules where upstream allows (FreeRTOS, ETL, CMSIS, STM32 HAL; fmu4cpp is a copy)
│
├── apps/                   # Top-level firmware executables (link modules + bsp)
│   ├── led_blink/          # built -- the gating SWIL example
│   ├── baro_logger/        # built -- first consumer of the BMP390 I2C driver
│   ├── mag_logger/         # built -- same, for the MMC5983MA
│   ├── can_actuator_link/  # host-only -- minimal modules/comms consumer; returns
│   │                       #   early under CMAKE_CROSSCOMPILING (stubs TX via <print>)
│   └── gnc_flight/         # planned -- README only; modules/gnc is empty
│
├── examples/               # Host-only end-to-end scenarios (HEMERION_BUILD_EXAMPLES=ON)
│   └── rocket_gps_ecos/    # Ecos co-simulation: Aetherion rocket -> GPS/IMU/mag FMUs
│
├── tests/                  # Cross-cutting integration and SWIL tests
├── doc/                    # Doxygen config, Sphinx source, ADRs
├── scripts/                # Toolchain check / install helpers
└── papers/                 # Publication drafts written against this codebase
```

---

## 📐 Architecture Overview

HEMERION is structured around three tiers:

- **Firmware tier** — FreeRTOS tasks, HAL drivers, CAN framing on STM32
- **SWIL tier** — Renode virtual machine executing firmware binaries without hardware
- **Co-simulation tier** — an FMI master orchestrating Hemerion sensor FMUs and Aetherion plant models

The sensor part models are the hinge. Both loops below drive the *same*
register-accurate model, so a driver bug reproduces identically in either:

```
        ┌──────────────────────────────┐
        │      Aetherion (Plant)       │  TwoStageRocket.fmu -- truth states
        └──────────────┬───────────────┘
                       │  FMI 2.0 / 3.0
        ┌──────────────▼───────────────┐
        │   FMI master (Ecos today;    │
        │   sim/fmi/ is still planned) │
        └──────────────┬───────────────┘
                       │
        ┌──────────────▼───────────────┐
        │  Hemerion sensor FMUs        │  BMP390 / MMC5983MA / IMU / GPS
        │  (register-accurate parts)   │
        └───────┬──────────────┬───────┘
                │              │
    UDP / spi_shm │              │ i2c_shm  ← the same model, other consumer
                ▼              ▼
   ┌────────────────────┐  ┌────────────────────────────┐
   │ Host flight SW     │  │ HemerionI2cBridge (Renode) │
   │ (examples/)        │  └─────────────┬──────────────┘
   └────────────────────┘                │
                              ┌──────────▼──────────────┐
                              │ Renode SWIL / STM32 HW  │  unmodified .elf
                              └─────────────────────────┘
```

### Design principles

**One module, three build targets.** Every module in `modules/` is meant to
build as a cross-compiled static library, as an x86 FMU shared library, and as
a native test binary. *(Reached for `sensors/`; `actuators/`, `gnc/` and
`datalogger/` are still empty directories — see `modules/README.md` for
per-module status.)* No `#ifdef TARGET` in module source — target differences are
injected by the BSP and the CMake toolchain file.

**BSPs own all hardware details.** A module never includes a path like
`stm32h743xx.h` directly; it goes through the BSP, so swapping a board means
switching the preset, not editing module code. *(The intended form of this is
a set of HAL abstraction headers — `hemerion/hal/gpio.h` and friends — which
**do not exist yet**; drivers currently take a bus type from the active BSP
directly. See `cmake/README.md` and `bsp/README.md`.)*

**`vendor/` is submodules wherever upstream's layout allows.** Every
dependency exposes a proper CMake target (`FreeRTOS::Kernel`, `ETL::etl`,
`CMSIS::Core`, …), defined in one place — `vendor/CMakeLists.txt` — rather
than through per-dependency wrapper modules. This keeps the cross/native
compiler split clean: the target carries the right include paths and compile
flags for whichever toolchain is active.

The one exception is **fmu4cpp**, vendored as a directory copy of upstream's
`export/` subtree because that subtree is not separately packaged; it is
noted as such below. `fmi4c` is *not* vendored at all — it arrives
transitively through Ecos in `examples/`.

**`sim/` is host-only.** Nothing under `sim/` is ever cross-compiled — the
root `CMakeLists.txt` hard-errors if you try. Renode board definitions, the
simulated I2C/SPI buses, and the shared-memory and UDP bridges to Aetherion
live here and link against the native build of module FMUs. *(The
Hemerion-owned FMI master, `sim/fmi/`, is still planned — the co-simulation
that runs today is `examples/rocket_gps_ecos`, driven by Ecos.)*

`stm32h743_nucleo` is the only BSP in the tree and the primary HWIL/SWIL
target; see [`bsp/README.md`](bsp/README.md) for what a second one would have
to provide.

---

## 🔌 Communication

`modules/comms/` provides **CAN framing primitives** — a peripheral-independent
`CanFrame` (2.0A/2.0B) with validation, and a fixed-capacity `CanFifo` over it.
`apps/can_actuator_link` packs actuator setpoints through both.

That is the whole of it today, and this README used to claim more: there is no
CANOpen object dictionary, no bus-management stack, and no multi-machine CAN
hub. Transmission itself is still stubbed — with no CAN HAL abstraction and no
BSP behind it, the app prints frames instead of sending them, which is also why
it excludes itself from cross builds. MAVLink telemetry is named in the module
plan but unbuilt; see [`modules/README.md`](modules/README.md).

---

## 🖥️ SWIL Simulation

Firmware is validated in simulation before any hardware contact. Renode runs
the *unmodified* cross-compiled `.elf`, so a SWIL pass exercises the real
driver against the real register map:

- **Nucleo-H743ZI2 platform description** — [`sim/renode/boards/nucleo_h743zi2.repl`](sim/renode/boards/nucleo_h743zi2.repl), brought up by [`sim/renode/scripts/swil_lockstep.resc`](sim/renode/scripts/swil_lockstep.resc)
- **`HemerionI2cBridge`** — a runtime-loaded `II2CPeripheral` that carries the emulated I2C transfers out to a host-side part model over TCP → `sim/i2c_shm` → the BMP390 or MMC5983MA simulator. See [`sim/renode/i2c_bridge/DESIGN.md`](sim/renode/i2c_bridge/DESIGN.md)
- **`pyrenode3` orchestration** — `tests/swil/` drives the platform from pytest, asserting on USART3 output

---

## 🧪 Testing

The framework includes a structured verification strategy:

| Layer | How it runs | What it covers |
|---|---|---|
| **Unit tests** (Catch2 v3) | `ctest --preset test-native` | Driver register encoding, raw→SI conversion, NMEA/UBX parse and emit, CAN framing, the shared-memory bus links |
| **FMU packaging tests** | `ctest --preset fmu-native` | Every packaged `.fmu` archive's layout, one test per archive registered by `generateFMU()` |
| **SWIL regression tests** | `ctest --preset test-swil -L swil` | `led_blink`, `baro_logger` and `mag_logger` firmware run unmodified under Renode, asserted on USART3 output |
| **Co-simulation examples** | `cmake --build --preset examples-native` | `rocket_gps_ecos` — Aetherion truth driving the GPS, IMU and magnetometer FMUs through Ecos |
| **HWIL** | — | Planned; no physical-target harness in the tree yet |

CI additionally runs clang-format, clang-tidy, IWYU, cmake-format/lint,
sanitizers (`-DHEMERION_ENABLE_SANITIZERS=ON`), coverage
(`-DHEMERION_ENABLE_COVERAGE=ON`) and a Metrix++ complexity report — see
[`.github/workflows/`](.github/workflows/).

---

## Dependencies

- **FreeRTOS-Kernel** — vendored via CMake / git submodules
- **CMSIS-Core, CMSIS-Device (H7/F4), STM32H7xx/STM32F4xx HAL drivers** — vendored via git submodules; see `vendor/CMakeLists.txt` for the `CMSIS::Core`, `CMSIS::STM32H7`, `STM32H7xx::HAL`, `CMSIS::STM32F4`, `STM32F4xx::HAL` targets. The F4 targets are vendored but currently unused: no F4 BSP or preset exists
- **Embedded Template Library (ETL)** — vendored via git submodule; STL alternative for bare-metal C++
- **fmu4cpp** — FMI 2.0/3.0 co-simulation *export*; every module FMU is built on it via `generateFMU()` (`cmake/generate_fmu.cmake`). Vendored as a directory copy of upstream's `export/` subtree, not a submodule — see `vendor/CMakeLists.txt`
- **fmi4c** — FMI *import*; how an FMI master loads a packaged `.fmu`. Not vendored here — Ecos fetches it as its own dependency in `examples/rocket_gps_ecos`
- **Renode** — SWIL simulation platform
- **Catch2 v3** — unit and integration testing
- **CMake ≥ 3.26** with `CMakePresets.json` for cross-compilation

---

## 📚 Documentation

- [Detailed Documentation](https://onurtuncer.github.io/Hemerion/) — guides, sensor models, and co-simulation walkthroughs
- [API Reference](https://onurtuncer.github.io/Hemerion/api/index.html) — every public header, generated from the source comments

Both are built from `doc/`. To build them locally:

```bash
pip install -r doc/requirements.txt   # plus doxygen on PATH
sphinx-build -b html doc build/sphinx
```

`doc/conf.py` runs Doxygen itself when `build/doxygen/xml` is missing, so that
single command produces the whole site including the API reference. The CMake
route (`-DHEMERION_BUILD_DOCS=ON`) runs the same two steps as an explicit target.

Adding a public header? `doc/check_api_coverage.py` fails CI until an
`api/` page documents it.

Guides worth starting from:

- [`doc/led_blink_tutorial.rst`](doc/led_blink_tutorial.rst) — first firmware build, end to end
- [`doc/sensor_models.rst`](doc/sensor_models.rst) — how the hardware-simulator FMUs are built
- [`doc/rocket_gps_ecos_cosim.rst`](doc/rocket_gps_ecos_cosim.rst) — the Ecos co-simulation walkthrough
- [`doc/bsp_freertos_wiring.rst`](doc/bsp_freertos_wiring.rst) — how the BSP wires FreeRTOS to the H743
- [`doc/swil_windows_setup.rst`](doc/swil_windows_setup.rst) — Renode under WSL2 on Windows

---

## Research

`papers/` holds publication drafts written against this codebase, with
[`papers/ARC.md`](papers/ARC.md) sequencing them. The through-line is the
protocol-accuracy claim above: that emitting byte-exact wire frames from
sensor models is what lets one firmware codebase move unchanged across
co-simulation, emulation and hardware — and that the fidelity is measurable
rather than asserted.

---

## Development

### Bug Tracker

- [GitHub Issues](https://github.com/onurtuncer/Hemerion/issues)

### Forums and Discussions

- [GitHub Discussions](https://github.com/onurtuncer/Hemerion/discussions/)

---

# Build & Installation

### Prerequisites

| Tool | Version | Purpose |
|---|---|---|
| CMake | ≥ 3.26 | Build system |
| arm-none-eabi-gcc | ≥ 12 | Cross-compiler |
| GCC ≥ 14 or MSVC 2022 | host native | Native / FMU builds — GCC 14+ required for `<print>` (C++23); Ubuntu 24.04 ships GCC 13 by default, install via `sudo apt-get install g++-14` |
| Renode | ≥ 1.15 | SWIL simulation |
| Python | ≥ 3.10 | Renode scripting, pyrenode3 |

Run `scripts/check-toolchain.ps1` (Windows) or `scripts/check-toolchain.sh` (Linux) to verify your environment against this table before configuring a preset.

To install whatever's missing, run `scripts/install-toolchain.ps1` (Windows, via winget) or `scripts/install-toolchain.sh` (Linux, via apt + pip). Both prompt before installing each tool individually; pass `-Yes`/`-y` to skip prompts. Re-run the check script afterwards in a new shell.

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/onurtuncer/Hemerion.git
cd Hemerion
```

Already cloned without them? The cross build fails at configure time with
`The link interface of target "hemerion_bsp_stm32h743_nucleo" contains:
CMSIS::STM32H7 but the target was not found` — `vendor/CMakeLists.txt` only
defines a vendor target when its submodule directory is populated, so an
uninitialised submodule surfaces as a missing target rather than a missing
checkout. Fix it with:

```bash
git submodule update --init --recursive
```

On Windows, the Arm toolchain installs to
`C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\<version>\bin` and is
**not** added to `PATH`; prepend it in the shell you configure from, or CMake
will not find `arm-none-eabi-gcc`.

### Build presets

```bash
# Firmware for Renode SWIL (cross-compiled, STM32H743).
# Builds led_blink, baro_logger and mag_logger .elf/.bin/.hex.
cmake --preset renode-h743
cmake --build --preset renode-h743

# FMU shared libraries for Aetherion co-simulation (native x86).
# Produces both build/fmu-native/fmus/fmi2/ and .../fmi3/ from one build.
cmake --preset fmu-native
cmake --build --preset fmu-native
ctest --preset fmu-native          # verifies every packaged .fmu archive

# Unit tests on host (native, no hardware or Renode needed)
cmake --preset test-native
cmake --build --preset test-native
ctest --preset test-native

# End-to-end co-simulation examples (fmu-native plus examples/, e.g. the
# Ecos-coupled rocket -> GPS/IMU/mag -> flight-software demo; fetches Ecos
# from source)
cmake --preset examples-native
cmake --build --preset examples-native
```

Presets in full: `renode-h743` (cross firmware), `fmu-native`,
`examples-native`, `test-native`, `test-swil`. See
[`CMakePresets.json`](CMakePresets.json) for their toolchain/BSP bindings —
each preset's `description` field explains why it is wired the way it is.

## Windows Build Using Visual Studio

1. Open **Visual Studio**
2. Click **Open Folder…**
3. Select the **Hemerion** directory
4. Let Visual Studio configure CMake automatically
5. Choose a configuration (e.g. **x64-Release** for host-side tools or **renode-h743** for firmware)
6. Build the project via **Build > Build All**

## Running SWIL Simulation

Interactively, with Renode installed (the script brings up the bare
Nucleo-H743 platform; load your own ELF from the monitor):

```bash
renode sim/renode/scripts/swil_lockstep.resc
```

The automated harness under `tests/swil/` drives the same platform from
pytest via pyrenode3. It needs **two** builds — the cross build for the
firmware, and a native one for the host-side bridge tools that
`test_baro_logger.py` and `test_mag_logger.py` use, which a cross toolchain
cannot produce (`sim/` is host-only, so `test-swil` never even adds it):

```bash
cmake --build --preset test-swil
cmake --build --preset test-native \
    --target i2c_shm_tcp_bridge bmp390_shm_peripheral mmc5983ma_shm_peripheral

export HEMERION_SWIL_TOOLS_DIR="$PWD/build/test-native/sim/i2c_shm/tools"
HEMERION_SWIL_STRICT=1 ctest --preset test-swil -L swil --output-on-failure
```

One host tool per simulated part, so that `--target` list grows with
`tests/swil/`. `HEMERION_SWIL_STRICT=1` makes a missing artefact fail rather
than skip — without it, ctest scores a skipped pytest as a pass. See
[`tests/README.md`](tests/README.md) for the full harness, and
[`doc/swil_windows_setup.rst`](doc/swil_windows_setup.rst) for the WSL2 setup
Windows machines need.

---

## Related Projects

| Project | Description |
|---|---|
| [Aetherion](https://github.com/onurtuncer/Aetherion) | C++23 host-side GNC simulation framework — the plant-side sibling of Hemerion. **≥ 0.13.0** for the co-simulation example, whose plant FMU must report geodetic position in degrees |
| CellForge | Offline robot programming platform (separate repository) |

---

## 👤 Author

**Prof. Dr. Onur Tuncer**
Aerospace Engineer, Researcher & C++ Systems Developer
Email: **onur.tuncer@itu.edu.tr**

<p align="left">
  <img src="assets/itu_logo.png" width="180" alt="Istanbul Technical University"/>
</p>
