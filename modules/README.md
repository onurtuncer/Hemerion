# modules/

Reusable firmware libraries. Each module is an independent CMake subdirectory that compiles to three artifacts depending on the active preset:

| Artifact | Preset family | Output |
|---|---|---|
| Static library | `cross-*`, `renode-*` | `libhemerion_<module>.a` linked into an app |
| FMU shared library | `fmu-native` | `<module>.fmu` consumed by Aetherion or any FMI 2.0 master |
| Test binary | `test-native` | Native executable run by CTest |

---

## Module list

| Module | Responsibility |
|---|---|
| `sensors/` | IMU, barometer, GPS drivers; raw → SI unit conversion; FMU export |
| `actuators/` | Servo, ESC, pyro channel drivers; FMU export |
| `comms/` | CAN framing, EtherCAT slave stack, MAVLink codec; OpenAMP/RPMsg transport *(planned, see `bsp/README.md` AMP targets)* |
| `gnc/` | EKF/UKF state estimation, control law; FMI co-simulation interface |
| `rtos_core/` | FreeRTOS task definitions, queue registry, tick config, memory pools |
| `fault/` | Health monitor, watchdog, error code registry |
| `power/` | BMS interface, regulator enable sequencing; FMU export |
| `datalogger/` | Flash ring buffer, COBS framing, telemetry packetiser |

---

## Module internal layout

Every module follows the same directory convention:

```
modules/<name>/
├── CMakeLists.txt          # Builds static lib; conditionally adds fmu/ and test/ targets
├── include/
│   └── hemerion/<name>/   # Public headers only — no implementation details
├── src/                    # Implementation files
├── fmu/
│   ├── CMakeLists.txt      # One generateFMU() call — builds and packages the .fmu
│   └── fmu_main.cpp        # Variable registrations + do_step(), on fmu4cpp
└── test/
    ├── CMakeLists.txt
    └── test_<name>.cpp     # Unity test cases; compiled for native host
```

The `fmu/` and `test/` subdirectories are added to the build only when the corresponding preset is active. Cross-compiled firmware builds skip both.

There is no hand-written FMI plumbing: `fmu_main.cpp` derives one class from `fmu4cpp::fmu_base`, registers its variables by name and implements `do_step()`. The vendored fmu4cpp export layer (`vendor/fmu4cpp/`) supplies the complete FMI 2.0 entry-point table, and `generateFMU()` (`cmake/generate_fmu.cmake`) compiles the two together, **generates** `modelDescription.xml` from the registered variables at build time, and zips the result into `<build>/fmus/fmi2/<name>.fmu`. Adding `fmi3` to the call's `FMI_VERSIONS` emits an FMI 3.0 archive from the same sources.

---

## Adding a new module

1. Copy `modules/template/` to `modules/<your_module>/`.
2. Rename the CMake target (`hemerion_template` → `hemerion_<your_module>`).
3. Add `add_subdirectory(modules/<your_module>)` in the root `CMakeLists.txt`.
4. Implement the HAL abstraction interface your module needs in the relevant BSP under `bsp/`.

No other files need to change. The three build artifacts are created automatically by the preset logic in `cmake/hemerion_module.cmake`.

---

## Dependencies

Modules may depend on:

- `FreeRTOS::Kernel` — RTOS primitives (cross-compiled builds only)
- `ETL::etl` — container and algorithm library (no dynamic allocation)
- BSP abstraction headers (`hemerion/hal/*`) — resolved at link time by the active BSP

Modules must **not** depend on:

- `sim/` targets
- Other modules (inter-module communication goes through RTOS queues defined in `rtos_core/`)
- Any STM32 HAL header directly — always go through the BSP abstraction layer

---

## Coding standard

Hemerion firmware targets JSF++ applicability where feasible. Key rules enforced by CI:

- No dynamic memory allocation after init (`ETL` containers, static pools)
- No exceptions, no RTTI
- `[[nodiscard]]` on all functions returning error codes
- Every public header must compile cleanly with `-Wall -Wextra -Wpedantic` under both `arm-none-eabi-gcc` and the host compiler
