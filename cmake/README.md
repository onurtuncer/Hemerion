# cmake/

Shared CMake helper functions, find-modules, and vendor target wrappers. The root `CMakeLists.txt` adds this directory to `CMAKE_MODULE_PATH` so everything here is available project-wide without path prefixes.

---

## Files

| File | Purpose |
|---|---|
| `hemerion_module.cmake` | `hemerion_add_module()` helper — generates static lib + FMU + test targets |
| `hemerion_hal/` | HAL abstraction headers consumed by modules |
| `FreeRTOS.cmake` | CMake target wrapper for `vendor/FreeRTOS-Kernel` |
| `ETL.cmake` | CMake target wrapper for `vendor/etl` |
| `STM32CubeH7.cmake` | CMake target wrapper for STM32H7 HAL/LL |
| `STM32CubeF4.cmake` | CMake target wrapper for STM32F4 HAL/LL |
| `fmi4c.cmake` | CMake target wrapper for `vendor/fmi4c` |
| `fmu4cpp.cmake` | CMake target wrapper for `vendor/fmu4cpp`; used by `sim/fmi/plant` to import Aetherion's plant FMU |
| `open_ecat.cmake` | CMake target wrapper for `vendor/open_ecat` |
| `FindRenode.cmake` | Find-module — locates Renode executable for CTest SWIL targets |
| `FindPyrenode3.cmake` | Find-module — locates pyrenode3 Python package |
| `FindArmNoneEabiGcc.cmake` | Find-module — verifies the arm-none-eabi-gcc cross-compiler and its version, ahead of `toolchains/arm-none-eabi.cmake` |
| `version.cmake` | Reads `VERSION` file; injects `HEMERION_VERSION_*` into compile definitions |
| `sanitizers.cmake` | Enables ASan/UBSan for `test-native` preset on supported compilers |

---

## `hemerion_add_module()`

The primary helper. Call it from a module's `CMakeLists.txt`:

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

## HAL abstraction headers (`hemerion_hal/`)

These are the interface contracts that modules include. BSPs implement them. They are plain C headers with `extern "C"` linkage so they compile cleanly under both C and C++.

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
3. Add an entry to the table above.

Keep helpers small and focused. If a helper grows beyond ~50 lines, it probably belongs in a dedicated subdirectory with its own README.
