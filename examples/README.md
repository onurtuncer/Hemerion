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
| [`rocket_gps_ecos/`](rocket_gps_ecos/) | Ecos FMI co-simulation: Aetherion's `TwoStageRocket.fmu` truth feeding Hemerion's GPS hardware-simulator FMU, whose UBX-NAV-PVT byte stream is decoded by the same GPS stack that runs on the STM32H743 flight computer |

Each example has its own README and a matching page in the Sphinx documentation (`doc/`).
