# examples/

Host-only, end-to-end demonstration programs. Unlike `apps/` (firmware executables cross-compiled for a BSP)
and `sim/` (reusable host-side simulation libraries), an example here is a complete, runnable scenario that
wires several pieces of the framework together and documents what comes out.

Built only when `HEMERION_BUILD_EXAMPLES=ON` — use the `examples-native` preset:

```
cmake --preset examples-native
cmake --build build/examples-native
```

| Example | What it demonstrates |
|---|---|
| [`f16_trim_ecos/`](f16_trim_ecos/) | Ecos FMI co-simulation of NASA TM-2015-218675 check-case 11 (F-16 subsonic trim flyout over Kitty Hawk): Aetherion's `F16Plant.fmu` truth feeding **all five** Hemerion sensor FMUs — GPS, IMU, BMP390, MMC5983MA and the radar altimeter. The only scenario where every environment-dependent stack is in band for the whole flight, which makes it the cross-sensor consistency reference |
| [`rocket_gps_ecos/`](rocket_gps_ecos/) | Ecos FMI co-simulation: Aetherion's `TwoStageRocket.fmu` truth feeding Hemerion's GPS and IMU hardware-simulator FMUs — a COCOM-limited u-blox receiver emitting UBX-NAV-PVT over UDP, and a MEMS IMU answering a shared-memory SPI bus — both driven by the same sensor stacks that run on the STM32H743 flight computer |

Each example has its own README and a matching page in the Sphinx documentation (`doc/`).
