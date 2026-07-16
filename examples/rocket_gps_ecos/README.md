# examples/rocket_gps_ecos — two-stage rocket → GPS + IMU FMUs → STM32 flight software, co-simulated with Ecos

End-to-end sensor-in-the-loop scenario built from four independently developed pieces:

```
┌──────────────────────┐   FMI 2.0 variables    ┌──────────────────────┐   UBX-NAV-PVT     ┌─────────────────────────┐
│  TwoStageRocket.fmu  │  (Ecos connections)    │ hemerion_gps_fmu.fmu │   over UDP        │   gps_flight_computer   │
│  (Aetherion 6-DoF    ├───────────────────────>│ (u-blox M9N hardware ├──────────────────>│  GpsDriver + UbxParser  │
│   plant, Radau IIA   │  lat, lon, alt,        │  simulator: noise +  │  127.0.0.1:5762   │  + ImuPacketParser +    │
│   on SE(3))          │  NED velocity          │  UBX encoder)        │  1 frame / step   │  convert_raw_to_si      │
│                      │                        └──────────────────────┘                   │  — the same modules/    │
│                      │   p/q/r (connections), ┌──────────────────────┐  raw-count frames │  sensors code the STM32 │
│                      │   specific force       │ hemerion_imu_fmu.fmu │   over UDP        │  H743 firmware runs     │
│                      ├───────────────────────>│ (MEMS IMU hardware   ├──────────────────>│                         │
│                      │   (host-computed)      │  simulator: noise +  │  127.0.0.1:5763   │                         │
└──────────────────────┘                        │  register counts)    │  10 frames / step │                         │
        │                                       └──────────────────────┘                   └─────────────────────────┘
        └────────────── rocket_gps_cosim (Ecos master, fixed-step 10 Hz) ──────────┘
```

* **`TwoStageRocket.fmu`** — Aetherion's two-stage rocket plant (NASA TM-2015-218675 Scenario 17: DAVE-ML
  aero/propulsion/inertia tables, J2 gravity, stage separation), installed with Aetherion under
  `<prefix>/share/Aetherion/fmu/`. It is the *truth* source.
* **`hemerion_gps_fmu.fmu`** — this repo's GPS hardware simulator
  (`modules/sensors/include/Hemerion/gps/fmu/`), packaged into a proper FMI 2.0 archive by the
  `hemerion_gps_fmu_package` target. Each co-simulation step it applies u-blox-M9N-grade noise to the truth
  inputs and emits one wire-exact UBX-NAV-PVT frame over UDP. It has **no FMI outputs** — the byte stream *is*
  its output, exactly like a real receiver's UART.
* **`hemerion_imu_fmu.fmu`** — this repo's IMU hardware simulator
  (`modules/sensors/include/Hemerion/imu/fmu/`), packaged by `hemerion_imu_fmu_package`. It takes true
  body-frame specific force and angular rate, applies a tactical-grade-MEMS error model (per-run turn-on bias +
  white noise), quantizes to 16-bit register counts at ±40 g / ±2000 °/s sensitivity, and streams Hemerion IMU
  raw-sample frames over UDP at `sample_rate_hz` (default 100 Hz — 10 frames per 0.1 s communication step,
  truth zero-order-held across the step). Like the GPS FMU it has no FMI outputs.
* **`gps_flight_computer`** — host stand-in for the STM32H743 flight computer's sensor ingest paths. It feeds
  the received GPS bytes through the unmodified `GpsDriver`/`UbxParser` and the IMU bytes through the
  unmodified `ImuPacketParser` + `convert_raw_to_si()` from `modules/sensors` — the same code the firmware
  cross-compiles — and logs every decoded fix and IMU sample. Only the transport differs from the target: UDP
  sockets here, UART/SPI RX (or Renode's emulated peripherals fed by the same FMUs, see `tests/swil/`) on
  hardware.
* **[Ecos](https://github.com/Ecos-platform/ecos)** — the co-simulation master. `rocket_gps_cosim` uses the
  Ecos C++ API (`simulation_structure`, `fixed_step_algorithm`, `csv_writer`) to load all three FMUs, wire
  truth to the sensor inputs, and log the truth trajectory.

The unit and interface mismatches between the FMUs are handled where they belong, in the orchestration layer:
the rocket reports lat/lon in **radians**, the GPS FMU takes **degrees**, so the two conversions ride on the
Ecos connections as modifiers; velocity is wired 1:1 through the GPS FMU's NED-velocity inputs
(`v_north_mps`/`v_east_mps`/`v_down_mps`), from which it derives speed-over-ground and course itself. Body
rates wire 1:1 to the IMU FMU's `p/q/r_rad_s` inputs. Specific force — what an accelerometer actually
measures — has no direct rocket output and involves three of them (`f = (thrust + F_aero) / mass`, an Ecos
connection modifier sees only one source variable), so the host computes it after every step and writes the
IMU's `f_x/f_y/f_z_mps2` inputs through Ecos properties, with the same one-step transport delay a connection
would have.

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
| `hemerion_imu_fmu.fmu` | `modules/sensors/include/Hemerion/imu/fmu/` |

## Running

Two terminals, both in `build/examples-native/examples/rocket_gps_ecos/`. Start the flight computer first (it
binds the sensor FMUs' default UDP destinations, ports 5762/GPS and 5763/IMU):

```
# terminal 1 — the "STM32" side
./gps_flight_computer

# terminal 2 — the co-simulation
./rocket_gps_cosim
```

The co-simulation runs a 240 s flight (staging at t ≈ 37 s, apogee ≈ 780 km at t ≈ 232 s; the Scenario 17
plant holds its state constant after apogee, so longer runs only add a flat tail) at a 0.1 s communication
step (= 10 Hz GPS, a typical u-blox navigation rate; the IMU emits at 100 Hz within each step) and needs no
interaction; the flight computer exits by itself a few seconds after both sensor streams stop.
Useful knobs (`--help` lists all):

* `--rocket <path>` / `--gps <path>` / `--imu <path>` — FMU locations, if the configure-time defaults don't
  apply.
* `--imu-rate <hz>` — IMU output data rate (default 100; becomes the FMU's `sample_rate_hz` parameter).
* `--rtf 1` — pace the co-simulation to wall-clock speed (default is as-fast-as-possible, about 1.7× real
  time on a typical desktop), e.g. to watch the fix stream come in live.
* `HEMERION_GPS_FMU_UDP_HOST` / `HEMERION_GPS_FMU_UDP_PORT` and `HEMERION_IMU_FMU_UDP_HOST` /
  `HEMERION_IMU_FMU_UDP_PORT` — environment variables read by the sensor FMUs at instantiation; point them at
  another host to feed a remote consumer (e.g. Renode on a different machine).

Outputs land in `results/`:

* `rocket_truth.csv` — Ecos `csv_writer` log of the rocket's outputs (altitude, position, NED velocity, body
  rates, Mach, dynamic pressure, thrust, mass, staging flag) plus the host-computed specific force the IMU FMU
  received, at every communication point.
* `gps_fixes.csv` — every fix the flight software decoded: position, speed/course, receiver-reported
  accuracies, satellite count, fix type.
* `imu_samples.csv` — every IMU sample the flight software decoded, already converted back to SI units by
  `convert_raw_to_si()`: specific force and angular rate per axis, timestamped from the frame payload.

## Plots

`plot_results.py` (matplotlib) turns the three CSVs into the figures used by the Sphinx page
(`doc/rocket_gps_ecos_cosim.rst`): altitude, ground track, speed over ground, the decoded-fix error against
truth, and the decoded IMU specific force and body rates against truth:

```
python plot_results.py            # reads results/, writes plots/
```

## Notes and design decisions

* **Why UDP and not an FMI output?** The point of the sensor FMUs is to exercise the *byte-level* driver
  stacks. A real receiver hands the flight computer a UART byte stream, not floating-point lat/lon, and a real
  IMU hands it register counts, not m/s²; keeping each FMU's output a wire-exact byte stream means the
  firmware-side parsers are tested against exactly what they will see on hardware, checksums and all.
* **What an accelerometer measures.** The IMU FMU's inputs are *specific force* — the non-gravitational force
  per unit mass — not coordinate acceleration. On the pad an unmodeled real part would read +1 g of ground
  reaction; in this plant (no ground-contact model) it reads thrust + aero over mass during boost and
  **zero during coast**, which the decoded stream reproduces: ~55 m/s² at ignition rising to ~265 m/s² at
  stage-2 burnout, then free fall at 0 m/s² all the way to (and past) apogee.
* **Scale is configuration, not wire data.** The IMU frames carry raw counts only, exactly like real silicon;
  the flight computer converts with the same ±40 g / ±2000 °/s `ImuScale` the FMU quantized with, standing in
  for the driver knowing the full-scale range it configured into the part's registers.
* **Fix/sample timestamps.** One NAV-PVT frame is emitted per communication step, so the flight computer maps
  fix index → simulation time (`--fix-period`, default 0.1 s) when logging. IMU frames carry the simulation
  clock in their payload, so their timestamps come straight off the wire. When the co-simulation runs faster
  than real time, wall-clock arrival times are meaningless for plotting; these mappings are exact as long as
  no datagram is dropped (loopback UDP; the summary line reports decoded counts vs. the master's step count).
* **fmi4c compatibility.** Ecos loads FMUs through fmi4c, which resolves the *complete* FMI 2.0 export table
  up front. Both sensor FMUs therefore export stub implementations (returning `fmi2Error`) of the
  state-management/derivative functions their capability flags disable. fmi4c's XML parser also rejects
  double-hyphen sequences inside XML comments (they are illegal per the XML spec), which constrains the
  comment style in `model_description.xml`.
