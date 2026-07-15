# examples/rocket_gps_ecos — two-stage rocket → GPS FMU → STM32 flight software, co-simulated with Ecos

End-to-end sensor-in-the-loop scenario built from three independently developed pieces:

```
┌──────────────────────┐   FMI 2.0 variables    ┌──────────────────────┐   UBX-NAV-PVT     ┌─────────────────────────┐
│  TwoStageRocket.fmu  │  (Ecos connections)    │ hemerion_gps_fmu.fmu │   over UDP        │   gps_flight_computer   │
│  (Aetherion 6-DoF    ├───────────────────────>│ (u-blox M9N hardware ├──────────────────>│  GpsDriver + UbxParser  │
│   plant, Radau IIA   │  lat, lon, alt,        │  simulator: noise +  │  127.0.0.1:5762   │  — the same modules/    │
│   on SE(3))          │  NED velocity          │  UBX encoder)        │  1 frame / step   │  sensors code the STM32 │
└──────────────────────┘                        └──────────────────────┘                   │  H743 firmware runs     │
        └────────────── rocket_gps_cosim (Ecos master, fixed-step 10 Hz) ──────────┘       └─────────────────────────┘
```

* **`TwoStageRocket.fmu`** — Aetherion's two-stage rocket plant (NASA TM-2015-218675 Scenario 17: DAVE-ML
  aero/propulsion/inertia tables, J2 gravity, stage separation), installed with Aetherion under
  `<prefix>/share/Aetherion/fmu/`. It is the *truth* source.
* **`hemerion_gps_fmu.fmu`** — this repo's GPS hardware simulator
  (`modules/sensors/include/Hemerion/gps/fmu/`), packaged into a proper FMI 2.0 archive by the
  `hemerion_gps_fmu_package` target. Each co-simulation step it applies u-blox-M9N-grade noise to the truth
  inputs and emits one wire-exact UBX-NAV-PVT frame over UDP. It has **no FMI outputs** — the byte stream *is*
  its output, exactly like a real receiver's UART.
* **`gps_flight_computer`** — host stand-in for the STM32H743 flight computer's GPS ingest path. It feeds the
  received bytes through the unmodified `GpsDriver`/`UbxParser` from `modules/sensors` — the same code the
  firmware cross-compiles — and logs every decoded fix. Only the transport differs from the target: UDP socket
  here, UART RX (or Renode's emulated UART fed by the same FMU, see `tests/swil/`) on hardware.
* **[Ecos](https://github.com/Ecos-platform/ecos)** — the co-simulation master. `rocket_gps_cosim` uses the
  Ecos C++ API (`simulation_structure`, `fixed_step_algorithm`, `csv_writer`) to load both FMUs, wire truth to
  the GPS inputs, and log the truth trajectory.

The unit mismatches between the two FMUs are handled where they belong, in the orchestration layer: the rocket
reports lat/lon in **radians**, the GPS FMU takes **degrees**, so the two conversions ride on the Ecos
connections as modifiers; velocity is wired 1:1 through the GPS FMU's NED-velocity inputs
(`v_north_mps`/`v_east_mps`/`v_down_mps`), from which it derives speed-over-ground and course itself.

## Building

Requires: native toolchain (see the repo README), network access at configure time (Ecos and its FMU loader
fmi4c are fetched from source), and an Aetherion install for `TwoStageRocket.fmu` (set `AETHERION_ROOT` if it
is not in a default location — the example still builds without it; you then pass `--rocket` at runtime).

```
cmake --preset examples-native
cmake --build build/examples-native
```

This produces, under `build/examples-native/`:

| Artifact | Location |
|---|---|
| `rocket_gps_cosim` | `examples/rocket_gps_ecos/` |
| `gps_flight_computer` | `examples/rocket_gps_ecos/` |
| `hemerion_gps_fmu.fmu` | `modules/sensors/include/Hemerion/gps/fmu/` |

## Running

Two terminals, both in `build/examples-native/examples/rocket_gps_ecos/`. Start the flight computer first (it
binds the GPS FMU's default UDP destination, port 5762):

```
# terminal 1 — the "STM32" side
./gps_flight_computer

# terminal 2 — the co-simulation
./rocket_gps_cosim
```

The co-simulation runs a 240 s flight (staging at t ≈ 37 s, apogee ≈ 780 km at t ≈ 232 s; the Scenario 17
plant holds its state constant after apogee, so longer runs only add a flat tail) at a 0.1 s communication
step (= 10 Hz GPS, a typical u-blox navigation rate) and needs no interaction; the flight computer exits by
itself a few seconds after the UBX stream stops.
Useful knobs (`--help` lists all):

* `--rocket <path>` / `--gps <path>` — FMU locations, if the configure-time defaults don't apply.
* `--rtf 1` — pace the co-simulation to wall-clock speed (default is as-fast-as-possible, about 1.7× real
  time on a typical desktop), e.g. to watch the fix stream come in live.
* `HEMERION_GPS_FMU_UDP_HOST` / `HEMERION_GPS_FMU_UDP_PORT` — environment variables read by the GPS FMU at
  instantiation; point them at another host to feed a remote consumer (e.g. Renode on a different machine).

Outputs land in `results/`:

* `rocket_truth.csv` — Ecos `csv_writer` log of the rocket's outputs (altitude, position, NED velocity, Mach,
  dynamic pressure, thrust, mass, staging flag) at every communication point.
* `gps_fixes.csv` — every fix the flight software decoded: position, speed/course, receiver-reported
  accuracies, satellite count, fix type.

## Plots

`plot_results.py` (matplotlib) turns the two CSVs into the figures used by the Sphinx page
(`doc/rocket_gps_ecos_cosim.rst`): altitude, ground track, speed over ground, and the decoded-fix error
against truth:

```
python plot_results.py            # reads results/, writes plots/
```

## Notes and design decisions

* **Why UDP and not an FMI output?** The point of the GPS FMU is to exercise the *byte-level* driver stack.
  A real receiver hands the flight computer a UART byte stream, not floating-point lat/lon; keeping the FMU's
  output a wire-exact UBX stream means the firmware-side parser is tested against exactly what it will see on
  hardware, checksums and all.
* **Fix timestamps.** One NAV-PVT frame is emitted per communication step, so the flight computer maps fix
  index → simulation time (`--fix-period`, default 0.1 s) when logging. When the co-simulation runs faster
  than real time, wall-clock arrival times are meaningless for plotting; the index mapping is exact as long as
  no datagram is dropped (loopback UDP; the summary line reports decoded-fix count vs. the master's step count).
* **fmi4c compatibility.** Ecos loads FMUs through fmi4c, which resolves the *complete* FMI 2.0 export table
  up front. The GPS FMU therefore exports stub implementations (returning `fmi2Error`) of the
  state-management/derivative functions its capability flags disable. fmi4c's XML parser also rejects
  double-hyphen sequences inside XML comments (they are illegal per the XML spec), which constrains the
  comment style in `model_description.xml`.
* **Next step: IMU.** The rocket FMU already exposes body rates (`out.p/q/r_rad_s`), attitude, and enough
  state to derive specific force; a future `imu.fmu` will tap the same truth bus alongside the GPS FMU and
  feed the flight computer a second sensor stream (see `modules/sensors` IMU conversion, already built).
