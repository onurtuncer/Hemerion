# Renode I2C-to-shm bridge — design note

**Status: implemented.** `HemerionI2cBridge.cs` (this directory) is the
runtime-loaded `II2CPeripheral` socket client; `sim/i2c_shm/tools/` holds
`i2c_shm_tcp_bridge` (the controller-end TCP server) and
`bmp390_shm_peripheral` (the BMP390 FMU minus FMI, for harnesses without an
FMI master); `tests/swil/test_baro_logger.py` closes the loop with the
unmodified `baro_logger` firmware. The sections below are the survey and
rationale the implementation followed.

Goal: let Renode-emulated firmware (`apps/baro_logger` on the emulated
STM32H743) talk to the BMP390 hardware-simulator FMU over the shared-memory
I2C bus (`sim/i2c_shm`), closing the same SWIL loop the host co-simulation
closes in `examples/rocket_gps_ecos` — with the *emulated* HAL driving the
*real* register-accurate part model.

## What the survey established (2026-08-04, Renode v1.16.1 under WSL)

* Renode's bundled `platforms/cpus/stm32h743.repl` already models the H743's
  I2C peripherals: `i2c1: I2C.STM32F7_I2C @ sysbus 0x40005400` (the I2C-v2
  IP the H7 shares with the F7). The firmware's `HAL_I2C_Mem_Read/Write`
  therefore drives an emulated controller that exists today — nothing to add
  on the MCU side of the bus.
* Renode I2C device models implement `II2CPeripheral`: `Write(byte[])` for
  the write phase, `byte[] Read(int)` for the read phase, and
  `FinishTransmission()` at STOP. Registration in the platform description
  (`bmp390: ... @ i2c1 0x76`) gives address matching for free; an absent
  registration is an address NACK, exactly the semantics `sim/i2c_shm`
  carries. Custom C# peripherals load at runtime from a `.resc` script
  (`include @path/file.cs`) — **no Renode rebuild required.**
* The event mapping onto one `I2cShmController` transaction is direct:
  accumulated `Write` bytes + a following `Read(count)` form one
  write-phase/read-phase transaction; `FinishTransmission` after writes
  alone is a write-only transaction; `Read` with no preceding write is a
  pure read.

## The topology problem, and the plan

`sim/i2c_shm` is same-host, same-OS shared memory. This repo's split —
firmware built on Windows, **Renode running in WSL2** (a VM) — means the
C# peripheral cannot map a Windows segment created by a Windows-side FMU.
Two-stage plan:

1. **Socket hop (portable, first).** A small host tool,
   `i2c_shm_tcp_bridge`, links `hemerion_i2c_shm` as the *controller* end
   and serves a trivial length-prefixed TCP protocol:
   `{address, write_len, write_bytes, read_len} -> {result, read_bytes}`.
   The C# `II2CPeripheral` is then a thin socket client — robust in both
   mono and .NET, and indifferent to whether the FMU runs on Windows or in
   WSL. This mirrors how the GPS FMU already reaches Renode (UDP for the
   UART), and one bridge process can serve any future I2C part.
2. **Direct shm mapping (optional, Linux-only optimization).** When FMU and
   Renode both run under Linux, the C# peripheral could map
   `/dev/shm/<bus>` itself (`MemoryMappedFile` + `Volatile` accessors on
   the `I2cBusRegion` layout). Deferred: it duplicates the handshake logic
   in a second language for a latency win that an emulated 100 kHz bus
   cannot observe.

## What was built

* `sim/renode/i2c_bridge/HemerionI2cBridge.cs` — the `II2CPeripheral` socket
  client (runtime-loaded, one class, no Renode fork). `targetAddress` has no
  default and must be given: Renode never tells a peripheral its own
  registration address, so the two values are independent and a default let
  them disagree silently.
* `sim/i2c_shm/tools/i2c_shm_tcp_bridge.cpp` — the host-side controller-end
  bridge. `--port 0` binds an ephemeral port and reports it on stdout, which
  is how the test learns where to point Renode.
* `sim/i2c_shm/tools/bmp390_shm_peripheral.cpp` — the BMP390 device model on
  the shm bus, the FMU minus FMI, for harnesses with no FMI master in the
  room. Runs until `--duration-s` elapses or SIGINT/SIGTERM arrives; a
  SIGKILL strands the bus segment, so stop it politely.
* `tests/swil/test_baro_logger.py` — the pyrenode3 test, asserting the
  `BARO up:` line and two `p=` readings against the ISA at the peripheral's
  truth altitude. The second reading is the point: it proves the data-ready
  cycle re-arms rather than the first read catching a lucky power-on state.

There is deliberately **no `bmp390` node in
`sim/renode/boards/nucleo_h743zi2.repl`**. The bridge's TCP port is chosen at
run time (see `--port 0` above), so the registration cannot be a static line
in the board file. The test writes a small platform overlay into a temporary
directory instead and loads it after the bare platform boots:

```
bmp390: I2C.HemerionI2cShmBridge @ i2c1 0x76
    targetAddress: 0x76
    port: <the port the bridge reported>
```

Anyone wanting the part permanently on the board can add that node with a
fixed `port` and run the bridge on it — the board file is left clean because
the SWIL test is the only consumer today, and it needs a fresh port per run.

## Running it outside the test

The two host tools are native binaries: a cross build cannot produce them
(`sim/` is host-only and `HEMERION_BUILD_SIM` hard-errors under a cross
toolchain), so they come from a native preset while the firmware comes from
`renode-h743`/`test-swil`:

```
cmake --build --preset test-native --target i2c_shm_tcp_bridge bmp390_shm_peripheral
```

`tests/swil` finds them by globbing the build trees, or via
`HEMERION_SWIL_TOOLS_DIR`. Set `HEMERION_SWIL_STRICT=1` — as CI does — to turn
"artefact not built" from a skip into a failure; ctest scores a skipped pytest
as a pass, so without it a job missing these binaries reports success having
run none of the loop.
