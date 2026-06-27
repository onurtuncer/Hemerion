# apps/gnc_flight/

**Status:** Concept only. Listed in `apps/README.md`'s app table
("Main GNC flight computer: sensors → EKF → control → actuators",
target STM32H743) but no `CMakeLists.txt`, `main.cpp`, `task_config.hpp`,
or `fmu_topology.yaml` exist yet. The root `apps/CMakeLists.txt` `foreach`
already lists `gnc_flight` and skips it via its `EXISTS` guard until
`CMakeLists.txt` is added here.

This README records a **reference task map** from early design notes.
It is a starting point for `task_config.hpp`, not a spec to match exactly
— see the caveat at the bottom about how it relates to the real
`modules/rtos_core` registries.

## Reference task map

Priorities follow `apps/README.md`'s band scheme
(`TASK_PRIO_LOW`=1–3, `TASK_PRIO_MID`=4–6, `TASK_PRIO_HIGH`=7–9,
`TASK_PRIO_CRITICAL`=10).

| Task | Priority | Period/trigger | Stack |
|---|---|---|---|
| `ImuDrvTask` | 7 | 250 µs (4 kHz ISR-fed) | 1024 w |
| `EkfTask` | 7 | 500 Hz | 2048 w |
| `GncControlTask` | 6 | 200 Hz | 2048 w |
| `PwmOutputTask` | 6 | 400 Hz | 512 w |
| `GpsDrvTask` | 5 | 18 Hz | 1024 w |
| `CanBusTask` | 5 | event-driven | 1024 w |
| `TelemetryTask` | 3 | 50 Hz | 2048 w |
| `AetherionBridgeTask` | 3 | 100 Hz, UDP | 2048 w |
| `HealthMonitorTask` | 1 | 1 Hz | 512 w |
| `LoggerTask` | 1 | 10 Hz | 4096 w |

## Reference IPC primitives

| Queue | Contents |
|---|---|
| `imuQueue` | `ImuSample` × 32 |
| `gpsQueue` | `GpsFix` × 4 |
| `ctrlQueue` | `CtrlOutput` × 8 |
| `telQueue` | `TelFrame` × 16 |

| Mutex/Semaphore | Purpose |
|---|---|
| `ekfStateMtx` | EKF state read guard |
| `canBusMtx` | CAN write guard |
| `imuDoneSem` | DMA completion |
| `gpsDoneSem` | UART idle-line |

Scheduler: 1000 Hz tick, preemption on, lazy FPU context stacking.

## Sketch: GncTask + FLIGHT_SAFE

```cpp
// apps/gnc_flight/tasks/GncTask.hpp (sketch — not yet written)
#include "hemerion/safety/flight_safe.hpp"   // real header: cmake/Safety/FlightSafe.hpp

class GncTask {
public:
    static constexpr UBaseType_t kPriority   = 6;
    static constexpr uint32_t    kStackWords = 2048;

    explicit GncTask(QueueHandle_t imuQ, QueueHandle_t gpsQ) noexcept;
    static void taskEntry(void* param) noexcept;

private:
    HEMERION_FLIGHT_SAFE_BEGIN
    void run() noexcept;
    HEMERION_FLIGHT_SAFE_END

    QueueHandle_t imuQueue_;
    QueueHandle_t gpsQueue_;
};
```

`HEMERION_FLIGHT_SAFE_BEGIN`/`END` are real and implemented
(`cmake/Safety/FlightSafe.hpp`), but nothing in the codebase uses them
yet — `GncTask` would be the first consumer.

## Caveat: how this relates to the real `rtos_core`

`modules/rtos_core`'s `TaskRegistry`/`QueueRegistry` are generic —
fixed-capacity arrays of descriptors registered by name at runtime
(`register_task(descriptor)`, `find_by_name(...)`), with **no
pre-defined task or queue names baked into `rtos_core` itself**. The
named tasks and queues above (`ImuDrvTask`, `imuQueue`, etc.) are an
`apps/gnc_flight`-level convention — this app would call
`TaskRegistry::register_task` with these names in its `main.cpp`, not add
them to `rtos_core`. Don't go looking for `imuQueue` as a global anywhere
else in the tree.
