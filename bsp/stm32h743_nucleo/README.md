# bsp/stm32h743_nucleo/

**Status:** Hardware design reference only. No BSP code exists in this
directory yet — no `CMakeLists.txt`, no `include/hemerion/hal/`
implementation, no linker script, no startup file. `bsp/README.md`'s
"BSP internal layout" is the target shape; this README fixes the
board-level hardware facts ahead of writing it.

This is the primary HWIL/SWIL target (see `bsp/README.md`'s "Available
BSPs" table) — referred to as **FCU-H743** in the hardware design
reference this content is transcribed from.

---

## MCU

| Parameter | Value |
|---|---|
| Part | STM32H743ZIT (Nucleo-144) |
| Core | Cortex-M7 @ 480 MHz |
| Flash | 2 MB, dual-bank |
| SRAM | 1 MB (DTCM, AXI SRAM, SRAM1–3) |
| FPU | Double-precision |
| DSP | SIMD instructions |
| I/D-Cache | 16 KB + 16 KB |
| Timers | 35 |

## Sensors on this board

| Part | Role | Interface |
|---|---|---|
| u-blox M9N | GPS | UART4, 921600 baud, UBX+NMEA, 18 Hz, DMA1 Ch0 |
| ICM-42688-P | 6-axis IMU | SPI1 @ 24 MHz, ±16g / ±2000°/s, 8000 Hz, DRDY → EXTI4 |
| MS5611 | Barometer | I²C1 @ 400 kHz, IRQ |

These are the sensors `modules/sensors` drivers target on this board —
see `modules/sensors`' actual `GpsDriver`/`UbxParser` for the real
(protocol-dispatch) driver shape, which differs from any
manufacturer-class-plus-transport-interface pattern that may appear in
older design notes.

---

## Connector / interface map

| Connector | Interface | Speed | Use | Notes |
|---|---|---|---|---|
| J1 GPS | UART4 | 921600 bps | u-blox M9N, UBX | DMA Rx |
| J2 IMU | SPI1 | 24 MHz | ICM-42688-P | DMA Tx/Rx |
| J3 Baro | I²C1 | 400 kHz | MS5611 | IRQ |
| J4 CAN1 | FDCAN1 | 1/5 Mbps | ESC / actuator bus | CAN FD |
| J5 CAN2 | FDCAN2 | 1 Mbps | Payload bus | CAN 2.0B |
| J6 ETH | ETH RMII | 100 Mbps | Aetherion bridge / Renode SWIL | UDP/IP |
| J7 Debug | USART1 | 115200 bps | RTT / syslog | Free-form |
| J8 PWM | TIM1/8 | 400 Hz / 8 kHz | 8× servo / DShot600 | DShot / PWM |
| J9 RC | UART5 | 100 kHz | SBUS/CRSF receiver | Inverted DMA |

## Power architecture

| Rail | Spec |
|---|---|
| Input | 4S–6S LiPo (14.8–25.2 V) |
| MCU/digital | 3.3 V @ 500 mA (LDO) |
| Analog | 3.3 VA, separate LDO |
| Servo/actuator | 5 V @ 3 A (buck) |
| Payload | 5 V / 12 V selectable |
| Backup | 5 F supercapacitor |

## Protection circuits

| Concern | Mitigation |
|---|---|
| Overcurrent | Per-channel eFuse |
| Reverse polarity | P-MOS + TVS |
| CAN isolation | ISO1050 digital isolator |
| Watchdog | Independent IWDG, 2 s timeout — see `modules/fault`'s `WatchdogSupervisor` for the software-side channel supervision this feeds |
| Thermal | NTC sensor + cutoff |
| Failsafe | Hardware PWM deadman timer |

---

## What's not implemented yet

Per `bsp/README.md`'s layout convention, this directory still needs:
`CMakeLists.txt`, `include/hemerion/hal/{gpio,uart,spi,i2c,can,timer,board}.h`
implementations, `src/startup_stm32h743xx.s`, `src/system_stm32h743.c`,
`linker/stm32h743_flash.ld`, and `FreeRTOSConfig.h`. The `renode-h743`
preset already references `HEMERION_BSP=stm32h743_nucleo`; until the
files above exist, the root `CMakeLists.txt`'s `EXISTS` guard skips this
directory silently rather than failing.
