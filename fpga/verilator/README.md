# fpga/verilator/

**Status:** Planned. The pattern below was proven against the FIR filter
(`ip/imu_fir/`) in an exploratory design session, but no harness code has
landed in this tree yet.

## Purpose

Verilator compiles synthesizable Verilog to a C++ class, so a PL IP block
can be unit-tested with the same Catch2/CTest pipeline as the rest of
Hemerion — no separate simulation environment, no `$display`-driven
testbench style.

This is a **different layer** from `tests/swil`: Renode SWIL runs compiled
Hemerion firmware with no PL involved at all. This harness runs PL logic
with no firmware involved at all. The two only meet once a real
Zynq AMP target exists and both layers run against the same data.

## Pattern (per IP block, e.g. `ip/imu_fir/verilator/`)

```
ip/<name>/verilator/
├── harness/
│   ├── main.cpp          # Catch2 TEST_CASE driver
│   └── <Name>Harness.hpp # thin wrapper around the Verilated class
└── CMakeLists.txt        # verilate() + Catch2 link
```

The harness wraps the Verilated DUT with a `tick()`-style API (drive
inputs, clock the design, poll `valid_out` with a timeout) so test code
reads like a normal unit test rather than a waveform script.

## Build integration (not wired up yet)

```cmake
# cmake/<name>.cmake (future)
find_package(verilator HINTS $ENV{VERILATOR_ROOT})
verilate(<name>_verilated SOURCES ... TOP_MODULE <top>)
```

```json
// CMakePresets.json (future addition to test-native)
"HEMERION_BUILD_PL_TESTS": "ON"
```

`HEMERION_BUILD_PL_TESTS` does not exist in the root `CMakeLists.txt` yet.
Add it only once the first harness (`ip/imu_fir/verilator/`) is real —
don't wire an empty option in ahead of content.

## Prerequisite

Verilator is **not** required for anything that exists in this repo today
(Icarus covers RTL-level `tb/` testing, Renode covers firmware SWIL). Only
install/require it once a `verilator/` harness is actually added under
`fpga/ip/`.
