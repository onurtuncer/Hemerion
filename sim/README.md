# sim/

Host-side simulation and co-simulation infrastructure. Nothing under `sim/`
is ever cross-compiled — see the "`sim/` is host-only" design principle in
the root README. These targets only build under native presets
(`fmu-native`, `test-native`) and link against the native build of module
FMUs and the Aetherion plant FMU.

---

## Components

| Directory | Responsibility |
|---|---|
| `renode/` | Renode board definitions (`.repl`) and emulation scripts (`.resc`) for STM32H7/STM32F4 SWIL targets |
| `fmi/` | FMI 2.0 co-simulation master: steps module FMUs and the Aetherion plant FMU in lockstep |
| `shm_bridge/` | Shared-memory transport between the FMI master and a locally running Aetherion process |
| `udp_bridge/` | UDP transport for the FMI master when Aetherion runs out-of-process or on a different host |

---

## `fmi/` — co-simulation master

`fmi/` orchestrates every FMU in a simulation run: the module FMUs built by
`fmu-native` (`sensors.fmu`, `gnc.fmu`, `actuators.fmu`, ...) and the
**plant FMU**, which is Aetherion's compiled flight-dynamics model rather
than anything Hemerion builds itself.

```
sim/fmi/
├── CMakeLists.txt          # FMI master executable; links fmu4cpp + Aetherion
├── master.cpp              # Lockstep scheduler: steps every loaded FMU
├── topology.cpp             # Reads apps/<app>/fmu_topology.yaml, wires variable connections
└── plant/
    ├── CMakeLists.txt      # hemerion_plant target
    ├── include/hemerion/plant/PlantModel.hpp
    └── src/PlantModel.cpp  # Imports Aetherion's .fmu via fmu4cpp, exposes step()/outputs()
```

### `fmi/plant/` — Aetherion plant FMU consumer

Where every other FMU in the system is something Hemerion **exports** (a
module wrapped for FMI), the plant FMU is something Hemerion **imports**:
the truth model produced by Aetherion (`Aetherion::generateFMU()`, see
`cmake/FindAetherion.cmake`). `PlantModel` wraps `fmu4cpp`'s FMU-import API
around that `.fmu` and exposes plain C++ getters/steppers to the rest of
the FMI master:

```cpp
hemerion::plant::PlantModel plant("build/fmu-native/aetherion/aetherion_plant.fmu");
plant.setInput("fin_deflection_1", cmd);
plant.doStep(dt);
double accel_x = plant.getOutput("true_accel_x");
```

`PlantModel` never appears in a cross-compiled or HWIL build — on real
hardware the "plant" is the real vehicle, sensed through the real sensor
module. It is only linked into `fmi/` under the `fmu-native` / `test-native`
presets.

---

## `shm_bridge/` — shared-memory transport

Used when Aetherion runs as its own local process instead of being imported
in-process via `fmi/plant/PlantModel` — the FMI master and that process
synchronize each simulation step over a named shared-memory segment rather
than function calls.

```
sim/shm_bridge/
├── CMakeLists.txt
├── include/hemerion/sim/shm_bridge/
│   ├── bridge_protocol.h   # BridgeRegion wire format + StepPhase handshake
│   ├── shm_segment.h       # Cross-platform named-segment RAII wrapper
│   └── shm_bridge.h        # Lockstep API built on the two headers above
├── src/
│   ├── shm_segment.cpp     # shm_open/mmap (POSIX) or CreateFileMapping (Windows)
│   └── shm_bridge.cpp
└── test/
```

The FMI master creates the segment (`ShmBridge::create_master`) once per
run; Aetherion attaches to it (`ShmBridge::open_peer`). Each step is a
four-phase handshake over a lock-free atomic in the shared region —
`kIdle → kInputsPosted → kOutputsPosted → kIdle` — with `kShutdownRequested`
unblocking the peer's wait loop during teardown:

```cpp
hemerion::sim::shm_bridge::ShmBridge master = *ShmBridge::create_master("hemerion_aetherion_step");
master.post_inputs(sim_time_s, dt_s, actuator_commands);
ChannelFrame outputs;
master.wait_for_outputs(outputs, 1000ms);
```

Synchronization is a spin-wait with a short sleep backoff, not an OS
semaphore — acceptable for same-host steps that complete in microseconds to
low milliseconds. `udp_bridge/` is the alternative transport for when
Aetherion runs out-of-process on a different host, where this shared-memory
approach doesn't apply.

---

## Dependencies

- **`fmu4cpp`** — vendored under `vendor/fmu4cpp`; wrapped by
  `cmake/fmu4cpp.cmake`. Used to import the Aetherion plant FMU, and to
  host module FMUs in-process for stepping by the master.
- **`Aetherion::Aetherion`** — located via `find_package(Aetherion)` /
  `cmake/FindAetherion.cmake`; supplies the plant FMU itself.
- **`pyrenode3` / Renode** — used only by `renode/`.

---

## Build presets that touch `sim/`

| Preset | What runs |
|---|---|
| `fmu-native` | Builds module FMUs + `fmi/` master + `fmi/plant/` importer |
| `test-native` | Runs `tests/fmu/` and `tests/integration/` against the master, with the plant FMU in the loop |
| `test-swil` | Uses `renode/` only; `fmi/` is not involved |
