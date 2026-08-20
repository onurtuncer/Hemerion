# modules/

Reusable firmware libraries. Each module is an independent CMake subdirectory that compiles to three artifacts depending on the active preset:

| Artifact | Preset family | Output |
|---|---|---|
| Static library | `cross-*`, `renode-*` | `libhemerion_<module>.a` linked into an app |
| Co-simulation FMU | `fmu-native` | `<module>.fmu` (FMI 2.0 and 3.0) consumed by Aetherion or any FMI master |
| Test binary | `test-native` | Native executable run by CTest |

---

## Module list

A module with no `CMakeLists.txt` is skipped by the root `CMakeLists.txt` foreach without comment, so an empty
directory here is indistinguishable from a built one at configure time. Status is therefore stated explicitly:

| Module | Status | Responsibility |
|---|---|---|
| `sensors/` | **built** | IMU, barometer (BMP390), GPS, radalt, mag drivers; raw → SI conversion; per-sensor hardware-simulator FMUs |
| `rtos_core/` | **built** | Task registry, queue registry, memory pools |
| `fault/` | **built** | Fault registry, watchdog supervisor. HAL health monitor and hardware IWDG feed are not built |
| `power/` | **built** | Battery monitor, regulator sequencer |
| `comms/` | **partial** | CAN framing only — one source file. EtherCAT slave stack and MAVLink codec are not built, and EtherCAT is not currently being pursued |
| `actuators/` | **empty** | Servo, ESC, pyro channel drivers — no sources, no `CMakeLists.txt` |
| `gnc/` | **empty** | EKF/UKF state estimation, control law — no sources, no `CMakeLists.txt` |
| `datalogger/` | **empty** | Flash ring buffer, COBS framing, telemetry packetiser — no sources, no `CMakeLists.txt` |

OpenAMP/RPMsg transport under `comms/` is planned — see `bsp/README.md`'s AMP targets.

---

## Module internal layout

The convention modules follow. `sensors/` is the fullest realisation of it; note that its FMU subtrees live under
`include/Hemerion/<sensor>/fmu/` rather than a top-level `fmu/`, and its tests use Catch2 rather than Unity:

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

1. Copy an existing module — `modules/power/` is the smallest complete one. (`modules/template/` is planned and not in the tree.)
2. Rename the CMake target to `hemerion_<your_module>`.
3. Add the directory name to the module foreach in the root `CMakeLists.txt`.
4. Implement whatever board access your module needs in the relevant BSP under `bsp/`.

> The intended end state is that step 2 collapses into one `hemerion_add_module()` call, with the three artifacts
> derived automatically. `cmake/hemerion_module.cmake` **has not been written** — write plain CMake for now, and see
> `cmake/README.md`.

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

The `sim/` rule exists to keep host-only code out of cross-compiled firmware,
so it binds the module's **library** artifact — the thing that ships on the
vehicle. A module's `<sensor>/fmu/` subtree is not that: it is host-only
simulation code, gated behind `HEMERION_BUILD_FMU`, never cross-compiled, and
never linked into the module library. Those subtrees may link `sim/` targets,
and one does: `modules/sensors/include/Hemerion/imu/fmu/` links `hemerion_spi_shm`,
because the IMU hardware simulator is an SPI peripheral and duplicating a
cross-platform shared-memory bus into `modules/` would be worse than the
dependency. (Contrast `gps/fmu/`'s `udpSender`, where the duplicated surface is
a single `sendto()` and the copy is cheaper than the coupling.)

---

## Coding standard

Hemerion firmware targets JSF++ applicability where feasible. Key rules enforced by CI:

- No dynamic memory allocation after init (`ETL` containers, static pools)
- No exceptions, no RTTI
- `[[nodiscard]]` on all functions returning error codes
- Every public header must compile cleanly with `-Wall -Wextra -Wpedantic` under both `arm-none-eabi-gcc` and the host compiler
