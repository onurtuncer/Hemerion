# bsp/

Board support packages. One subdirectory per physical or virtual target. A BSP owns everything that is target-specific: linker script, startup code, clock configuration, pin mapping, and `FreeRTOSConfig.h`. Module source never reaches into a BSP subdirectory — it includes only the HAL abstraction headers in `cmake/hemerion_hal/`.

---

## Available BSPs

| BSP | Target | Notes |
|---|---|---|
| `stm32h743_nucleo/` | STM32H743ZI Nucleo-144 | Primary HWIL and Renode SWIL target |
| `stm32f446_custom/` | Custom ECS gateway board | CAN + EtherCAT slave, TwinCAT 3 interface |
| `native_linux/` | x86_64 Linux (PREEMPT_RT) | POSIX shims over FreeRTOS API for desktop testing |
| `zynq_core0/` *(planned)* | Zynq-7020 PS Core 0 | Linux (PetaLinux), OpenAMP `remoteproc` master, ETH bridge to Aetherion. See [AMP targets](#amp-targets-planned) below. |
| `zynq_core1/` *(planned)* | Zynq-7020 PS Core 1 | FreeRTOS, bare-metal — runs the same Hemerion tasks as `stm32h743_nucleo` over RPMsg. |
| `template/` | Scaffold for new boards | Copy this to add a new target |

---

## BSP internal layout

```
bsp/<name>/
├── CMakeLists.txt              # Defines hemerion_bsp_<name> INTERFACE library
├── include/
│   └── hemerion/hal/           # Implements the HAL abstraction (gpio, uart, spi, …)
│       ├── gpio.h
│       ├── uart.h
│       └── board.h             # Board-level init, clock setup
├── src/
│   ├── startup_<device>.s      # Reset handler, vector table
│   └── system_<device>.c       # SystemClock_Config, MPU setup
├── linker/
│   └── <device>_flash.ld       # Linker script (flash + RAM regions)
└── FreeRTOSConfig.h            # Tick rate, heap size, stack depths for this target
```

The BSP CMake target is an `INTERFACE` library. It contributes include paths, compile definitions, and the linker script to whatever app or module links against it — it contains no compiled object files of its own.

---

## HAL abstraction contract

Every BSP must implement the headers declared in `cmake/hemerion_hal/`. These are thin, `extern "C"` interfaces:

```c
// hemerion/hal/gpio.h
void hal_gpio_write(uint8_t port, uint8_t pin, bool level);
bool hal_gpio_read(uint8_t port, uint8_t pin);
void hal_gpio_set_mode(uint8_t port, uint8_t pin, hal_gpio_mode_t mode);
```

If a peripheral is not available on a given target (e.g. `native_linux/` has no real SPI), the BSP provides a stub implementation that logs a warning via the datalogger. This keeps module code unconditional.

---

## Adding a new board

1. Copy `bsp/template/` to `bsp/<your_board>/`.
2. Fill in the linker script, startup file, and clock configuration.
3. Implement every header in `include/hemerion/hal/`.
4. Add a new preset in `CMakePresets.json`:

```json
{
  "name": "cross-your_board",
  "inherits": "cross-base",
  "cacheVariables": {
    "HEMERION_BSP": "your_board"
  }
}
```

5. Add a Renode `.repl` file in `sim/renode/boards/` if SWIL simulation is needed.

No module source changes. No root `CMakeLists.txt` changes beyond `add_subdirectory`.

---

## AMP targets (planned)

`zynq_core0/` and `zynq_core1/` are an **evaluated alternative** to the
STM32H743 path, explored as part of an optional FPGA mezzanine (the PL IP
lives in a separate repository). They are not a committed replacement — design work
stopped at the architecture-reference stage, no BSP code exists yet, and
`stm32h743_nucleo` remains the primary target until a Phase 1 prototype
(PYNQ-Z2) validates the split.

The split follows Xilinx's standard AMP (Asymmetric Multi-Processing)
model on the Zynq-7020's two Cortex-A9 cores:

- **Core 0 — Linux.** Owns the Ethernet bridge to Aetherion, loads the
  Core 1 firmware image via `remoteproc`, and handles bulk data logging.
  HAL contract is implemented with POSIX sockets/pthreads instead of
  HAL+FreeRTOS.
- **Core 1 — FreeRTOS.** Runs Hemerion's existing tasks essentially
  unchanged — same `hemerion/hal/*` contract, same `FLIGHT_SAFE` pragma
  regions, same task code. Only the BSP underneath differs.

Cross-core communication is OpenAMP/RPMsg, not the `hemerion/hal/*`
contract — it is a new transport, not a new HAL header. If this track is
picked up, the RPMsg interface should land as a `modules/comms` addition
(see `modules/README.md`), since it is firmware-side IPC, not
board-specific hardware access.

**Critical constraint if this is ever implemented:** Core 1's GNC loop
must not depend on Core 0 being alive. If Linux locks up, RPMsg sends are
skipped, not blocked — `FLIGHT_SAFE` code must never wait on the Linux
side.

---

## FreeRTOSConfig notes

`FreeRTOSConfig.h` lives in the BSP, not in `vendor/FreeRTOS-Kernel/`. This is intentional — tick rate, heap implementation, and stack sizes are board-dependent. The `FreeRTOS::Kernel` CMake target in `vendor/` picks up the config header via the BSP's include path, which must be on the target's `INTERFACE_INCLUDE_DIRECTORIES` before `FreeRTOS::Kernel` is linked.

Recommended baseline values for STM32H7:

| Parameter | Value |
|---|---|
| `configCPU_CLOCK_HZ` | 480000000 |
| `configTICK_RATE_HZ` | 1000 |
| `configTOTAL_HEAP_SIZE` | 131072 (128 kB) |
| `configUSE_TIMERS` | 1 |
| `configUSE_TASK_NOTIFICATIONS` | 1 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 |
