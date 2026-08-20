# cmake/

Shared CMake helper functions, find-modules, and vendor target wrappers. The root `CMakeLists.txt` adds this directory to `CMAKE_MODULE_PATH` so everything here is available project-wide without path prefixes.

---

## Files

Rows marked **planned** describe intended design that is **not in the tree**. Nothing outside `present` can be
`include()`d today — see the repository's scaffolding status in the root `README.md`.

| File | Status | Purpose |
|---|---|---|
| `generate_fmu.cmake` | present | `generateFMU()` — compiles model sources against the vendored fmu4cpp export layer and packages a standards-conforming `.fmu`. Kept under fmu4cpp's MIT licence to stay cheap to re-sync |
| `fmu_archive_test.cmake` | present | `hemerion_add_fmu_archive_test()` — registers the `fmu`-labelled packaging test that `generateFMU()` adds per archive |
| `verify_fmu_archive.cmake` | present | The `cmake -P` script that test runs: checks `modelDescription.xml` is at the archive root, `binaries/` beside it, and the identifiers agree |
| `FindAetherion.cmake` | present | Find-module — locates an installed Aetherion (`Aetherion::Aetherion`) for the plant side |
| `FindRenode.cmake` | present | Find-module — locates the Renode executable for the SWIL ctest gate |
| `FindPyrenode3.cmake` | present | Find-module — locates the pyrenode3 Python package |
| `FindArmNoneEabiGcc.cmake` | present | Find-module — verifies `arm-none-eabi-gcc` and its version, ahead of `toolchains/arm-none-eabi.cmake` |
| `Safety/FlightSafe.hpp` | present | `FLIGHT_SAFE` pragma-region markers referenced by the coding standard |
| `hemerion_module.cmake` | **planned** | `hemerion_add_module()` helper — static lib + FMU + test targets from one call. Modules use plain CMake until it lands; `modules/sensors/CMakeLists.txt` carries the migration note |
| `sanitizers.cmake` | present | ASan + UBSan for native builds, opt-in via `-DHEMERION_ENABLE_SANITIZERS=ON`. GCC/Clang only; warns loudly rather than skipping quietly on a compiler it cannot instrument |
| `coverage.cmake` | present | gcov instrumentation for native builds, opt-in via `-DHEMERION_ENABLE_COVERAGE=ON`. Refuses MSVC outright rather than producing an empty report |
| `hemerion_hal/` | **planned** | HAL abstraction headers modules would include. No such directory exists; `bsp/README.md`'s HAL contract is design intent |
| `version.cmake` | **planned** | Would inject `HEMERION_VERSION_*`. The root `CMakeLists.txt` reads the version straight out of `package.xml` instead, and there is no `VERSION` file |
| `FreeRTOS.cmake`, `ETL.cmake`, `STM32CubeH7.cmake`, `STM32CubeF4.cmake`, `fmi4c.cmake`, `fmu4cpp.cmake`, `open_ecat.cmake` | **planned** | Per-dependency target wrappers. None exist: `vendor/CMakeLists.txt` defines every vendored target in one place, `fmi4c` arrives transitively through Ecos in `examples/`, and there is no `vendor/open_ecat` |

---

## `hemerion_add_module()` — planned, not implemented

> **Nothing in this section exists.** `cmake/hemerion_module.cmake` has not been written; every module under
> `modules/` uses plain `add_library()` / `add_subdirectory()` today. What follows is the intended shape of the
> helper, kept because it is the design the modules are meant to migrate to — not a description of the build.

Call it from a module's `CMakeLists.txt`:

```cmake
hemerion_add_module(
    NAME       sensors
    SOURCES    src/imu.cpp src/baro.cpp src/gps.cpp
    DEPENDS    FreeRTOS::Kernel ETL::etl
)
```

What it does:

1. Creates `hemerion_sensors` as a static library (cross-compiled when `CMAKE_CROSSCOMPILING` is true).
2. When `HEMERION_BUILD_FMU` is ON (set by `fmu-native` preset), adds `modules/sensors/fmu/` as a subdirectory to build `sensors.fmu`.
3. When `HEMERION_BUILD_TESTS` is ON (set by `test-native` preset), adds `modules/sensors/test/` as a subdirectory.
4. Applies project-wide compile options from `hemerion_compile_options` interface target.

Optional arguments:

```cmake
hemerion_add_module(
    NAME        gnc
    SOURCES     src/ekf.cpp src/control.cpp
    DEPENDS     ETL::etl hemerion_sensors
    INCLUDES    include
    NO_FMU      # skip FMU generation for this module
)
```

---

## HAL abstraction headers (`hemerion_hal/`) — planned, not implemented

> **Nothing in this section exists.** There is no `cmake/hemerion_hal/` directory and no `hemerion::hal_headers`
> target. Drivers written so far (`modules/sensors/`) talk to a BSP-provided bus type directly — see
> `bsp/stm32h743_nucleo/`. This is the contract they are meant to converge on.

These are the interface contracts that modules would include. BSPs implement them. They are plain C headers with `extern "C"` linkage so they compile cleanly under both C and C++.

```
cmake/hemerion_hal/
├── hemerion/hal/gpio.h
├── hemerion/hal/uart.h
├── hemerion/hal/spi.h
├── hemerion/hal/i2c.h
├── hemerion/hal/can.h
├── hemerion/hal/timer.h
└── hemerion/hal/board.h        # hal_board_init(), hal_board_reset()
```

An `INTERFACE` library target `hemerion::hal_headers` exposes this directory. Modules link `hemerion::hal_headers`; BSPs provide implementations.

---

## Adding a new helper

1. Create `cmake/<name>.cmake`.
2. The file is automatically available via `include(<name>)` because `cmake/` is on `CMAKE_MODULE_PATH`.
3. Add an entry to the table above — as `present`, and move any matching `planned` row up to it.

Keep helpers small and focused. If a helper grows beyond ~50 lines, it probably belongs in a dedicated subdirectory with its own README.
