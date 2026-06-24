# apps/

Top-level firmware executables. An app is the only thing in the repository that links — it composes one or more modules with a BSP and produces either a flashable `.elf` or an FMU composition entry point.

---

## App list

| App | Target | Description |
|---|---|---|
| `gnc_flight/` | STM32H743 | Main GNC flight computer: sensors → EKF → control → actuators |
| `ecs_gateway/` | STM32F446 | EtherCAT ↔ CAN gateway: relays TwinCAT setpoints to CAN actuators |
| `datalink/` | STM32F446 | Telemetry downlink + uplink: MAVLink over UART/radio |

---

## App internal layout

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

```bash
# Build the GNC flight app for Nucleo-H743
cmake --preset renode-h743
cmake --build --preset renode-h743 --target gnc_flight

# Flash via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
    -c "program build/renode-h743/apps/gnc_flight/gnc_flight.elf verify reset exit"

# Or via STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -d build/renode-h743/apps/gnc_flight/gnc_flight.bin 0x08000000
```

---

## FMU co-simulation wiring

`fmu_topology.yaml` describes how module FMUs are connected when the app runs in co-simulation mode under the `sim/fmi/` master. Example:

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

## Task priority scheme

Use the constants in `task_config.hpp` rather than raw numbers. Suggested priority bands:

| Band | Priority | Used for |
|---|---|---|
| `TASK_PRIO_IDLE` | 0 | Background logging flush |
| `TASK_PRIO_LOW` | 1–3 | Telemetry, health monitor |
| `TASK_PRIO_MID` | 4–6 | Sensor acquisition, datalogger |
| `TASK_PRIO_HIGH` | 7–9 | Control loop, actuator output |
| `TASK_PRIO_CRITICAL` | 10 | Fault handler, watchdog kick |

`configMAX_PRIORITIES` in the BSP's `FreeRTOSConfig.h` must be set to at least 11.
