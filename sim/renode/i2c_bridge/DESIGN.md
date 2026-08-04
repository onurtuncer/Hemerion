# Renode I2C-to-shm bridge — design note (not yet implemented)

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

## Deliverables when implemented

* `sim/renode/i2c_bridge/HemerionI2cBridge.cs` — the `II2CPeripheral`
  socket client (runtime-loaded, one class, no Renode fork).
* `sim/i2c_shm/tools/i2c_shm_tcp_bridge.cpp` — the host-side
  controller-end bridge.
* `sim/renode/boards/nucleo_h743zi2.repl` — `bmp390` registration on
  `i2c1` at 0x76.
* `tests/swil/test_baro_logger.py` — pyrenode3 test asserting the
  `BARO up:` line and sane `p=`/`T=` values on the VCP against the running
  BMP390 FMU.
