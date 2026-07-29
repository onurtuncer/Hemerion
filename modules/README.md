# modules/

Reusable firmware libraries. Each module is an independent CMake subdirectory that compiles to three artifacts depending on the active preset:

| Artifact | Preset family | Output |
|---|---|---|
| Static library | `cross-*`, `renode-*` | `libhemerion_<module>.a` linked into an app |
| Co-simulation FMU | `fmu-native` | `<module>.fmu` (FMI 2.0 and 3.0) consumed by Aetherion or any FMI master |
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

There is no hand-written FMI plumbing: `fmu_main.cpp` derives one class from `fmu4cpp::fmu_base`, registers its variables by name and implements `do_step()`. The vendored fmu4cpp export layer (`vendor/fmu4cpp/`) supplies the complete entry-point table for both FMI generations, and `generateFMU()` (`cmake/generate_fmu.cmake`) compiles the two together **once per FMI version listed in its `FMI_VERSIONS`**, generates each archive's `modelDescription.xml` from the registered variables at build time, and zips the result into `<build>/fmus/<fmiVersion>/<name>.fmu`.

The sensor FMUs list `fmi2 fmi3`, so one `cmake --build` produces both an FMI 2.0 and an FMI 3.0 archive from a single set of sources — the model code is version-agnostic, and the same registered variable *names* appear in both descriptions. One binary cannot export both ABIs, so each version gets its own shared library (`<name>_fmi2`, `<name>_fmi3`), and that per-version target name is also the `modelIdentifier` inside its description.

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
