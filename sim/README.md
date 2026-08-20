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
| `fmi/` | **planned.** FMI 2.0 co-simulation master. The directory holds a `package.xml` and nothing else — no `master.cpp`, no `plant/`, no `CMakeLists.txt`. The working end-to-end co-simulation today is `examples/rocket_gps_ecos`, which uses Ecos as the master instead |
| `i2c_shm/` | Simulated I2C bus in shared memory, plus the host tools that bridge it to an emulated controller (see `renode/i2c_bridge/DESIGN.md`) |
| `shm_bridge/` | Shared-memory transport between the FMI master and a locally running Aetherion process |
| `spi_shm/` | Simulated SPI bus in shared memory: sensor FMUs answer chip-select-framed transfers from host or emulated firmware |
| `udp_bridge/` | UDP transport for the FMI master when Aetherion runs out-of-process or on a different host |

---

## `fmi/` — co-simulation master (planned, not implemented)

> **Nothing in this section exists.** `sim/fmi/` contains a single `package.xml`. There is no master executable, no
> `PlantModel`, and no `fmu_topology.yaml` reader; `sim/CMakeLists.txt` adds the directory only if it ever grows a
> `CMakeLists.txt`. For a co-simulation that actually runs today, see `examples/rocket_gps_ecos`, which drives the
> GPS, IMU and BMP390 FMUs from Ecos. The design below is retained as the intent for a Hemerion-owned master.

`fmi/` is intended to orchestrate every FMU in a simulation run: the module FMUs built by
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

### `fmi/plant/` — Aetherion plant FMU consumer (planned)

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

## `spi_shm/` — simulated SPI bus

Where `shm_bridge/` shares *variable values* between a master and a plant,
`spi_shm/` shares a *bus*. It exists because not every sensor pushes bytes at
a flight computer: an SPI part sits there holding samples until the MCU
asserts chip select and comes to get them, and a hardware simulator that
cannot be polled tests only half the driver.

```
sim/spi_shm/
├── CMakeLists.txt
├── include/hemerion/sim/spi_shm/
│   ├── spi_shm_protocol.h          # SpiBusRegion wire format + BusPhase handshake
│   ├── spi_shm_link.h              # SpiShmPeripheral / SpiShmController endpoints
│   └── spi_peripheral_endpoint.h   # what an FMU needs to *be* an SPI peripheral
├── src/
└── test/
```

The peripheral (a sensor FMU) creates the segment and services transfers on a
background thread — real silicon answers chip select whenever it is asserted,
not when its physics model happens to be stepping. The controller (host flight
software, or emulated firmware) attaches and issues transfers:

```cpp
auto spi = *SpiShmController::attach_within("hemerion_imu_spi", 120s);
std::array<std::uint8_t, 4> tx{ 0x81, 0, 0, 0 };  // read STATUS, auto-increment
std::array<std::uint8_t, 4> rx{};
spi.transfer(tx.data(), rx.data(), tx.size(), 1000ms);
bool drdy = spi.data_ready();                      // the part's DRDY line
```

**One transfer is one handshake.** That is the granularity a HAL SPI call has
on the target, and the granularity at which a controller can still choose its
MOSI bytes — a driver builds the whole TX buffer before it calls
`HAL_SPI_TransmitReceive`, so byte *k* could not have depended on byte
*k−1*'s answer anyway. The peripheral end still shifts the posted buffer byte
by byte through its own state machine, so the MISO content is what per-bit
clocking would have produced, and a device model is written the way a
datasheet describes the part.

`SpiPeripheralEndpoint` is what a sensor FMU actually holds: it owns bus
naming (with an environment-variable override), segment lifecycle, the service
thread and the data-ready line, and binds a device model duck-typed through
the `SpiShiftable` concept. The FMU then declares only *which* part it is —
see `modules/sensors/include/Hemerion/imu/fmu/` for the one consumer so far.

Since the region is shared memory, both ends must be on the same host; Renode's
emulated SPI peripheral is the path for a remote or in-emulator controller.

---

## `udp_bridge/` — UDP transport

Used when Aetherion runs out-of-process on a different host from the FMI
master, where `shm_bridge/`'s shared-memory segment doesn't apply — the two
sides exchange one UDP datagram per step instead of synchronizing over
shared bytes.

```
sim/udp_bridge/
├── CMakeLists.txt
├── include/hemerion/sim/udp_bridge/
│   ├── bridge_protocol.h  # StepPacket wire format + PacketType
│   ├── udp_socket.h       # Cross-platform connected-UDP-socket RAII wrapper
│   └── udp_bridge.h       # Lockstep API built on the two headers above
├── src/
│   ├── udp_socket.cpp     # BSD sockets (POSIX) or Winsock (Windows)
│   └── udp_bridge.cpp
└── test/
```

The FMI master binds a socket and targets Aetherion's address
(`UdpBridge::create_master`); Aetherion binds its own socket pointed back at
the master (`UdpBridge::create_peer`). Each step is two datagrams rather than
a shared phase transition — a `kInputs` packet from the master, a `kOutputs`
packet back from the peer — with a `kShutdown` packet unblocking the peer's
wait loop during teardown:

```cpp
hemerion::sim::udp_bridge::UdpBridge master =
    *UdpBridge::create_master("0.0.0.0", 9100, "192.168.1.50", 9100);
master.post_inputs(sim_time_s, dt_s, actuator_commands);
ChannelFrame outputs;
master.wait_for_outputs(outputs, 1000ms);
```

UDP can drop a datagram outright, so a `wait_for_*()` timeout here can mean
the packet itself never arrived, not just a slow peer — callers are expected
to retry the corresponding `post_*()` call after a timeout. Both ends are
assumed to be the same architecture; no byte-swapping is done on `StepPacket`,
matching the no-byte-swap assumption `shm_bridge/` already makes for its
shared region.

---

## Dependencies

- **`fmu4cpp`** — vendored under `vendor/fmu4cpp` as a directory copy of
  upstream's `export/` subtree (not a submodule). Its targets are defined
  directly in `vendor/CMakeLists.txt`; there is no `cmake/fmu4cpp.cmake`
  wrapper. Hemerion uses it to *export* module FMUs via `generateFMU()`.
  Importing an FMU is fmi4c's job, which arrives transitively through Ecos
  in `examples/` — Hemerion does not vendor it.
- **`Aetherion::Aetherion`** — located via `find_package(Aetherion)` /
  `cmake/FindAetherion.cmake`; would supply the plant FMU. Nothing in the
  tree calls it yet.
- **`pyrenode3` / Renode** — used only by `renode/`.

---

## Build presets that touch `sim/`

| Preset | What runs |
|---|---|
| `fmu-native` | Builds the module FMUs and the shm/SPI/I2C/UDP transports they sit on, and runs the `fmu`-labelled archive-layout tests `generateFMU()` registers. The `fmi/` master is planned and builds nothing. Does **not** build `i2c_shm/tools/` — nothing in an FMU build consumes those host binaries |
| `test-native` | Builds and runs every `sim/` unit test (`shm_bridge`, `spi_shm`, `i2c_shm`, `udp_bridge`), and builds `i2c_shm/tools/`, since it sets `HEMERION_BUILD_SIM`. `tests/fmu/` and `tests/integration/` do not exist |
| `test-swil` | Uses `renode/` only; `fmi/` is not involved. Being a cross build it cannot add `sim/` at all, so the SWIL BMP390 test needs a companion `test-native` build for its host tools |
