# HEMERION — Embedded Firmware Framework

[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Linux Build](https://github.com/onurtuncer/Hemerion/actions/workflows/linux.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/linux.yml)
[![Windows Build](https://github.com/onurtuncer/Hemerion/actions/workflows/windows.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/windows.yml)
[![codecov](https://codecov.io/gh/onurtuncer/Hemerion/branch/main/graph/badge.svg)](https://codecov.io/gh/onurtuncer/Hemerion)
[![Clang-Format](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_format.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/clang_format.yml)
[![CMake-Format](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_format.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_format.yml)
[![CMake-Lint](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_lint.yml/badge.svg)](https://github.com/onurtuncer/Hemerion/actions/workflows/cmake_lint.yml)
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
- **Native** — modules compiled as x86 shared libraries for FMI 2.0 co-simulation with Aetherion

Hemerion is the firmware counterpart to [Aetherion](../aetherion), the
C++23 host-side GNC simulation framework. Together they form a lockstep
co-simulation stack targeting small launch vehicles.

The name *ΗΜΕΡΙΟΝ* (Hemerion) is inspired by the Greek word **Ἥμερος**,
meaning *tame* or *cultivated* — the disciplined, deterministic counterpart
to the boundless realm of *Aetherion*.

---

## ✨ Features

- **FreeRTOS task architecture** with deterministic scheduling on STM32
- **Renode SWIL simulation** via `.repl` / `.resc` board definitions for STM32H7 and STM32F4
- **CAN bus communication** for distributed actuator and sensor networks
- **FMI 2.0 co-simulation interface** for tight coupling with Aetherion flight dynamics
- **Embedded Template Library (ETL)** as an STL alternative for bare-metal targets
- **CMakePresets-based cross-compilation** toolchain for ARM Cortex-M targets
- **Post-modern C++23 firmware design**
- **Optional Zynq-7020 AMP mezzanine** *(planned)* — FPGA-accelerated sensor preprocessing, evaluated alongside the STM32H743 path

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
│   ├── sensors/
│   ├── actuators/
│   ├── comms/
│   ├── gnc/
│   ├── rtos_core/
│   ├── fault/
│   ├── power/
│   └── datalogger/
│
├── bsp/                    # Board support packages (one per target)
│   ├── stm32h743_nucleo/
│   └── template/
│
├── sim/                    # Host-side simulation targets (never cross-compiled)
│   ├── renode/
│   ├── fmi/
│   │   ├── master.cpp      # FMI 2.0 co-simulation master
│   │   └── plant/          # Imports Aetherion's plant FMU via fmu4cpp
│   ├── shm_bridge/
│   └── udp_bridge/
│
├── vendor/                 # Git submodules — no source copies (FreeRTOS, ETL, fmi4c, fmu4cpp, ...)
│
├── apps/                   # Top-level firmware executables (link modules + bsp)
├── tests/                  # Cross-cutting integration and SWIL tests
└── doc/                    # Doxygen config, Sphinx source, ADRs
```

---

## 📐 Architecture Overview

HEMERION is structured around three tiers:

- **Firmware tier** — FreeRTOS tasks, HAL drivers, CAN middleware on STM32
- **SWIL tier** — Renode virtual machine executing firmware binaries without hardware
- **Co-simulation tier** — FMI master orchestrating Hemerion firmware and Aetherion plant models

```
┌─────────────────────────────┐
│      Aetherion (Plant)      │  ← FMI 2.0 / UDP
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│   Hemerion Co-sim Master    │  ← C++23, FMI 2.0
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│   Renode SWIL / STM32 HW    │  ← FreeRTOS firmware
└─────────────────────────────┘
```

### Design principles

**One module, three build targets.** Every module in `modules/` builds as a
cross-compiled static library, as an x86 FMU shared library, and as a native
test binary. No `#ifdef TARGET` in module source — target differences are
injected by the BSP and the CMake toolchain file.

**BSPs own all hardware details.** A module never includes a path like
`stm32h743xx.h` directly. It includes the BSP abstraction header
(`hemerion/hal/gpio.h`) which the active BSP implements. Swapping a board
means switching the preset, not editing module code.

**`vendor/` is submodules only.** No vendored source is copied into the
tree. Every dependency exposes a proper CMake target (`FreeRTOS::Kernel`,
`ETL::etl`, `fmi4c::fmi4c`). This keeps the cross/native compiler split
clean — the target carries the right include paths and compile flags for
whichever toolchain is active.

**`sim/` is host-only.** Nothing under `sim/` is ever cross-compiled. Renode
board definitions, the FMI master, and the shared-memory bridge to Aetherion
live here and link against the native build of module FMUs.

### Planned: optional FPGA / AMP tier

An optional Zynq-7020 mezzanine, run in Xilinx's AMP (Asymmetric
Multi-Processing) configuration, is under evaluation as an alternative to
the STM32H743 path — not a committed replacement:

```
┌─────────────────────────────┐
│      Aetherion (Plant)      │  ← FMI 2.0 / UDP
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  Zynq-7020 PS — Core 0      │  ← Linux, OpenAMP remoteproc, ETH bridge
│  (bsp/zynq_core0, planned)  │
└────────────┬────────────────┘
             │ RPMsg (OpenAMP)
┌────────────▼────────────────┐
│  Zynq-7020 PS — Core 1      │  ← FreeRTOS, same Hemerion tasks unchanged
│  (bsp/zynq_core1, planned)  │
└────────────┬────────────────┘
             │ AXI / EMIO
┌────────────▼────────────────┐
│  Zynq-7020 PL — FPGA IP     │  ← IMU FIR, sensor sync, ... (separate repo)
└──────────────────────────────┘
```

The PL IP blocks and their Verilator test harness are maintained in a
separate repository. See the "AMP targets (planned)" section of
`bsp/README.md` for status and open questions. `stm32h743_nucleo` remains
the primary HWIL/SWIL target until a Phase 1 prototype validates this
path.

---

## 🔌 Communication

HEMERION provides **CAN bus** communication — CANOpen-style messaging for
distributed sensor and actuator nodes, with a **Virtual CANHub** for
multi-machine CAN simulation within Renode.

---

## 🖥️ SWIL Simulation

Firmware is validated in simulation before any hardware contact:

- Custom STM32H7 / STM32F4 board definitions via Renode `.repl` and `.resc` scripts
- RISC-V CLINT / PLIC peripheral configuration for mixed-ISA experimentation
- `pyrenode3` Python bindings for automated SWIL test orchestration

---

## 🧪 Testing

The framework includes a structured verification strategy:

- **Unit tests** with Catch2 for firmware logic components
- **SWIL regression tests** via Renode scripted scenarios
- **Hardware-in-the-Loop (HWIL)** validation on physical STM32 targets (planned)
- **FMI co-simulation integration tests** against Aetherion plant models

---

## Dependencies

- **FreeRTOS-Kernel** — vendored via CMake / git submodules
- **CMSIS-Core, CMSIS-Device (H7/F4), STM32H7xx/STM32F4xx HAL drivers** — vendored via git submodules; see `vendor/CMakeLists.txt` for the `CMSIS::Core`, `CMSIS::STM32H7`, `STM32H7xx::HAL`, `CMSIS::STM32F4`, `STM32F4xx::HAL` targets
- **Embedded Template Library (ETL)** — vendored via git submodule; STL alternative for bare-metal C++
- **fmi4c** — FMI 2.0 export for module FMUs
- **fmu4cpp** — FMI 2.0 import; used by `sim/fmi/plant` to consume Aetherion's plant FMU
- **Renode** — SWIL simulation platform
- **Catch2 v3** — unit and integration testing
- **CMake ≥ 3.26** with `CMakePresets.json` for cross-compilation

---

## 📚 Documentation

- [Detailed Documentation](https://onurtuncer.github.io/Hemerion/)

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
| GCC or MSVC | host default | Native / FMU builds |
| Renode | ≥ 1.15 | SWIL simulation |
| Python | ≥ 3.10 | Renode scripting, pyrenode3 |

Run `scripts/check-toolchain.ps1` (Windows) or `scripts/check-toolchain.sh` (Linux) to verify your environment against this table before configuring a preset.

To install whatever's missing, run `scripts/install-toolchain.ps1` (Windows, via winget) or `scripts/install-toolchain.sh` (Linux, via apt + pip). Both prompt before installing each tool individually; pass `-Yes`/`-y` to skip prompts. Re-run the check script afterwards in a new shell.

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/onurtuncer/Hemerion.git
cd Hemerion
```

### Build presets

```bash
# Firmware for Renode SWIL (cross-compiled, STM32H743)
cmake --preset renode-h743
cmake --build --preset renode-h743

# FMU shared libraries for Aetherion co-simulation (native x86)
cmake --preset fmu-native
cmake --build --preset fmu-native

# Unit tests on host (native, no hardware or Renode needed)
cmake --preset test-native
cmake --build --preset test-native
ctest --preset test-native
```

See `CMakePresets.json` for the full list of presets and their toolchain/BSP bindings.

## Windows Build Using Visual Studio

1. Open **Visual Studio**
2. Click **Open Folder…**
3. Select the **Hemerion** directory
4. Let Visual Studio configure CMake automatically
5. Choose a configuration (e.g. **x64-Release** for host-side tools or **renode-h743** for firmware)
6. Build the project via **Build > Build All**

## Running SWIL Simulation

With Renode installed:

```bash
renode scripts/stm32h7_hemerion.resc
```

---

## Related Projects

| Project | Description |
|---|---|
| [Aetherion](https://github.com/onurtuncer/Aetherion) | C++23 host-side GNC simulation framework — the plant-side sibling of Hemerion |
| CellForge | Offline robot programming platform (separate repository) |

---

## 👤 Author

**Prof. Dr. Onur Tuncer**
Aerospace Engineer, Researcher & C++ Systems Developer
Email: **onur.tuncer@itu.edu.tr**

<p align="left">
  <img src="assets/itu_logo.png" width="180" alt="Istanbul Technical University"/>
</p>
