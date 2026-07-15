# apps/led_blink

Minimal FreeRTOS application for the NUCLEO-H743ZI2 board. A single task
toggles **LD1** (green LED, PB0) every 500 ms and prints `LED ON` / `LED OFF`
over **USART3** (the ST-LINK virtual COM port). It is also the project's
primary SWIL example — `tests/swil/test_led_blink.py` boots this firmware
inside Renode and asserts on the UART output.

---

## What it does

```
hal_board_init()
hal_uart_init(USART3, 115200)
hal_gpio_set_mode(PB0, OUTPUT_PUSH_PULL)

FreeRTOS scheduler
└── led_blink_task  (priority: idle+1)
    └── forever:
          toggle PB0
          print "LED ON" / "LED OFF" → USART3
          vTaskDelay(500 ms)
```

---

## Build

Cross-compile for the STM32H743 target:

```powershell
cmake --preset renode-h743
cmake --build --preset renode-h743 --target led_blink
```

Outputs (under `build/renode-h743/apps/led_blink/`):

| File | Description |
|---|---|
| `led_blink` | ELF — used by Renode SWIL and `arm-none-eabi-*` tools |
| `led_blink.hex` | Intel HEX — for STM32CubeProgrammer / OpenOCD |
| `led_blink.bin` | Raw binary — for `st-flash` / DFU |

---

## Flash to hardware

**STM32CubeProgrammer (GUI or CLI):**

```bash
STM32_Programmer_CLI -c port=SWD -w build/renode-h743/apps/led_blink/led_blink.hex -v -rst
```

**OpenOCD:**

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/renode-h743/apps/led_blink/led_blink.hex verify reset exit"
```

After flashing, LD1 (green, top-left of the Nucleo-144 baseboard) blinks at 1 Hz.

---

## Monitor serial output

USART3 appears as the ST-LINK VCP (USB-CDC). Baud rate: **115200 8N1**.

```bash
# Linux / WSL
minicom -b 115200 -D /dev/ttyACM0

# Windows (PowerShell)
putty.exe -serial COM<N> -sercfg 115200,8,n,1,N
```

Expected output:

```
LED ON
LED OFF
LED ON
LED OFF
...
```

---

## Run under Renode (SWIL)

No hardware needed — Renode simulates the STM32H743 and pyrenode3 observes
the UART output from Python.

> **Windows users:** pyrenode3 requires Renode's CoreCLR/Linux build and must
> run from **WSL2**. See `doc/swil_windows_setup.rst` (or the rendered docs) for
> the one-time WSL + Renode + pyrenode3 setup.

From WSL2, with the firmware already built on the Windows host:

```bash
cd /mnt/d/Dev/Hemerion/tests/swil
~/swilvenv/bin/python3 -m pytest test_led_blink.py -v
```

The test boots the firmware in Renode, waits for `LED ON` then `LED OFF` then
`LED ON` over simulated USART3, and passes when all three are received within
the 5-second timeout.

For how the test is structured, see `tests/swil/test_led_blink.py` and
`tests/README.md`.
