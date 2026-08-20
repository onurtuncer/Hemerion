# apps/baro_logger — BMP390 over I2C1, printed over the VCP

The first firmware consumer of the on-target BMP390 driver
(`modules/sensors/include/Hemerion/baro/bmp390/`): one FreeRTOS task probes a
BMP390 on **I2C1 (PB8 = SCL, PB9 = SDA — the Nucleo-144's Arduino D15/D14
header pins)**, reads its calibration NVM, programs ×4/×1 oversampling at
50 Hz, and prints one line per conversion over USART3 (the ST-LINK Virtual
COM Port, 115200 8N1):

```
BARO up: BMP390 identified, calibrated, normal mode at 50 Hz
BARO t=163840 us p=101321 Pa T=23150 mC
```

Values are integers (Pa, milli-°C, and microseconds derived from the part's
own `SENSORTIME` counter, which wraps every 512 s) so newlib-nano's
float-less `printf` suffices.

This is the same `Bmp390Driver` + `Bmp390Compensator` object code the host
co-simulation runs in `examples/rocket_gps_ecos` — the only difference is
the board layer: `Bmp390HalI2cBus` over the BSP's `hal_i2c_mem_read/write`
here, the shared-memory I2C bus (`sim/i2c_shm`) there.

## Wiring

| BMP390 breakout | Nucleo-H743ZI2 |
|---|---|
| SCL | PB8 (CN7 D15) |
| SDA | PB9 (CN7 D14) |
| SDO | GND (selects address 0x76) |
| VDD/VDDIO | 3V3 |
| GND | GND |

The bus needs real pull-ups (most breakouts carry 4.7 kΩ on board; the BSP
enables the weak internal ones only as an assist). The part's INT pin is
left unwired — the driver's `STATUS` poll is authoritative without it.

A probe failure prints its reason (`CHIP_ID mismatch` vs. `no ack`) and
retries every second, so a mis-wired bench session reads as a diagnosis
rather than a dead board.

## Building

```
cmake --preset renode-h743
cmake --build build/renode-h743 --target baro_logger
```

`baro_logger.hex` / `.bin` land next to the ELF.

## Running it under Renode

This app *does* run in emulation: `sim/renode/i2c_bridge` closes the gap
between Renode's emulated I2C and the shared-memory bus the BMP390 device
model answers on (the IMU's SPI path still has it). The chain is

```
baro_logger (emulated) -> Renode STM32F7_I2C -> HemerionI2cShmBridge (C#)
    -> TCP -> i2c_shm_tcp_bridge -> sim/i2c_shm -> bmp390_shm_peripheral
```

`tests/swil/test_baro_logger.py` drives exactly that and asserts the
`BARO up:` banner plus two compensated pressure readings against the ISA at
the peripheral's truth altitude. The two host tools it needs are native
binaries a cross build cannot produce, so they come from a native preset:

```
cmake --build --preset test-native --target i2c_shm_tcp_bridge bmp390_shm_peripheral
```

See `sim/renode/i2c_bridge/DESIGN.md` for the wire protocol and the
platform-overlay details, and `tests/README.md` for running the harness.
The host co-simulation in `examples/rocket_gps_ecos` remains the other way
to exercise the same driver.
