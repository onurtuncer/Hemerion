# apps/

Top-level firmware executables. An app is the only thing in the repository that links — it composes one or more modules with a BSP and produces either a flashable `.elf` or an FMU composition entry point.

---

## App list

Every app guards its targets behind `if(TARGET hemerion_bsp_stm32h743_nucleo)`, and `apps/CMakeLists.txt` skips a
subdirectory with no `CMakeLists.txt`, so an app that is not built simply does not appear — no configure error.

| App | Status | Target | Description |
|---|---|---|---|
| `led_blink/` | **built** | STM32H743 | Minimal FreeRTOS LED blink demo; the gating SWIL example via `tests/swil/test_led_blink.py` |
| `baro_logger/` | **built** | STM32H743 | First firmware consumer of the BMP390 I2C driver; drives the full SWIL I2C chain via `tests/swil/test_baro_logger.py` (currently informational — it hangs before its first print) |
| `can_actuator_link/` | **built** | STM32F446 nominally | Minimal CAN actuator command sender; demonstrates `modules/comms` framing. No F446 BSP exists, so it only builds under the H743 BSP guard |
| `gnc_flight/` | **planned** | STM32H743 | Main GNC flight computer: sensors → EKF → control → actuators. README only — no sources, no `CMakeLists.txt`, and `modules/gnc` is empty |
| `datalink/` | **planned** | STM32F446 | Telemetry downlink + uplink: MAVLink over UART/radio. Named in `apps/CMakeLists.txt`; the directory does not exist |

---

## App internal layout

The intended convention. Neither `task_config.hpp` nor `fmu_topology.yaml` exists in any app today — the two built
apps are a single `main.cpp` plus a `CMakeLists.txt`.

```
apps/<name>/
├── CMakeLists.txt          # Links modules + BSP; generates .elf, .bin, .hex
├── main.cpp                # Entry point: HAL init, task creation, scheduler start
├── task_config.hpp         # Task priorities, stack sizes, queue depths for this app
└── fmu_topology.yaml       # FMU wiring for co-simulation (used by sim/fmi/ master)
```

Apps are thin by design. If logic ends up in `main.cpp` beyond task creation and scheduler launch, it belongs in a module instead.

---

## Build and flash

`gnc_flight` is used below as the eventual example; substitute `led_blink` or `baro_logger` to run these today.

```bash
# Build an app for Nucleo-H743
cmake --preset renode-h743
cmake --build --preset renode-h743 --target led_blink

# Flash via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
    -c "program build/renode-h743/apps/gnc_flight/gnc_flight.elf verify reset exit"

# Or via STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -d build/renode-h743/apps/gnc_flight/gnc_flight.bin 0x08000000
```

---

## FMU co-simulation wiring — planned, not implemented

> **Nothing in this section exists.** No app has an `fmu_topology.yaml`, and `sim/fmi/` holds only a `package.xml` —
> there is no master to read one. The co-simulation that runs today wires its FMUs in C++ in
> `examples/rocket_gps_ecos`. There is no `sim/fmi/README.md`.

`fmu_topology.yaml` would describe how module FMUs are connected when the app runs in co-simulation mode under the `sim/fmi/` master. Example:

```yaml
fmus:
  - id: sensors
    path: build/fmu-native/modules/sensors/sensors.fmu
  - id: gnc
    path: build/fmu-native/modules/gnc/gnc.fmu
  - id: actuators
    path: build/fmu-native/modules/actuators/actuators.fmu

connections:
  - from: sensors.imu_accel_x
    to:   gnc.accel_x
  - from: gnc.fin_deflection_1
    to:   actuators.servo_1_cmd
```

The FMI master in `sim/fmi/` reads this file at startup and resolves variable references before the first step. See `sim/fmi/README.md` for the full topology schema.

---

## Task priority scheme — planned

> No app has a `task_config.hpp` yet. These are the bands to use when one is written.

Use named constants rather than raw numbers. Suggested priority bands:

| Band | Priority | Used for |
|---|---|---|
| `TASK_PRIO_IDLE` | 0 | Background logging flush |
| `TASK_PRIO_LOW` | 1–3 | Telemetry, health monitor |
| `TASK_PRIO_MID` | 4–6 | Sensor acquisition, datalogger |
| `TASK_PRIO_HIGH` | 7–9 | Control loop, actuator output |
| `TASK_PRIO_CRITICAL` | 10 | Fault handler, watchdog kick |

`configMAX_PRIORITIES` in the BSP's `FreeRTOSConfig.h` must be set to at least 11.
