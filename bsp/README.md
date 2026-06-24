# bsp/

Board support packages. One subdirectory per physical or virtual target. A BSP owns everything that is target-specific: linker script, startup code, clock configuration, pin mapping, and `FreeRTOSConfig.h`. Module source never reaches into a BSP subdirectory — it includes only the HAL abstraction headers in `cmake/hemerion_hal/`.

---

## Available BSPs

| BSP | Target | Notes |
|---|---|---|
| `stm32h743_nucleo/` | STM32H743ZI Nucleo-144 | Primary HWIL and Renode SWIL target |
| `stm32f446_custom/` | Custom ECS gateway board | CAN + EtherCAT slave, TwinCAT 3 interface |
| `native_linux/` | x86_64 Linux (PREEMPT_RT) | POSIX shims over FreeRTOS API for desktop testing |
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
