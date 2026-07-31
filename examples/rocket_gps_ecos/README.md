# examples/rocket_gps_ecos — two-stage rocket → GPS + IMU FMUs → STM32 flight software, co-simulated with Ecos

End-to-end sensor-in-the-loop scenario built from four independently developed pieces:

```
┌──────────────────────┐   FMI 2.0 variables    ┌──────────────────────┐   UBX-NAV-PVT     ┌─────────────────────────┐
│  TwoStageRocket.fmu  │  (Ecos connections)    │ hemerion_gps_fmu.fmu │   over UDP        │   gps_flight_computer   │
│  (Aetherion 6-DoF    ├───────────────────────>│ (u-blox M9N sim:     ├──────────────────>│  GpsDriver + UbxParser  │
│   plant, Radau IIA   │  lat, lon, alt,        │  noise + COCOM/4 g   │  127.0.0.1:5762   │                         │
│   on SE(3))          │  NED velocity          │  envelope + UBX enc) │  1 frame / step   │  ImuSpiDriver +         │
│                      │                        └──────────────────────┘                   │  ImuPacketParser +      │
│                      │   p/q/r (connections), ┌──────────────────────┐  SPI transfers    │  convert_raw_to_si      │
│                      │   specific force       │ hemerion_imu_fmu.fmu │<──────────────────┤                         │
│                      ├───────────────────────>│ (MEMS IMU sim: noise │   over shared     │  — the same modules/    │
│                      │   (host-computed)      │  + registers + FIFO) ├─── memory ───────>│  sensors code the STM32 │
└──────────────────────┘                        └──────────────────────┘  bursts of counts │  H743 firmware runs     │
        │                                                                                  └─────────────────────────┘
        └────────────── rocket_gps_cosim (Ecos master, fixed-step 10 Hz) ──────────┘
```

The two arrow directions are the point: **the receiver talks, the IMU is polled.** A GPS module pushes NAV
solutions down a UART whenever it has them; an inertial part sits there holding samples until a flight
computer asserts chip select and comes to get them. Modelling both as pushed byte streams would have
exercised the packet parsers and nothing else.

* **`TwoStageRocket.fmu`** — Aetherion's two-stage rocket plant (NASA TM-2015-218675 Scenario 17: DAVE-ML
  aero/propulsion/inertia tables, J2 gravity, stage separation), installed with Aetherion under
  `<prefix>/share/Aetherion/fmu/`. It is the *truth* source.
* **`hemerion_gps_fmu.fmu`** — this repo's GPS hardware simulator
  (`modules/sensors/include/Hemerion/gps/fmu/`), built and packaged into a proper FMI 2.0 archive by
  `generateFMU()` (`cmake/generate_fmu.cmake`). Each co-simulation step it applies u-blox-M9N-grade noise to
  the truth inputs, gates the result through the receiver's dynamics envelope, and emits one wire-exact
  UBX-NAV-PVT frame over UDP. It has **no FMI outputs** — the byte stream *is* its output, exactly like a real
  receiver's UART.
* **`hemerion_imu_fmu.fmu`** — this repo's IMU hardware simulator
  (`modules/sensors/include/Hemerion/imu/fmu/`), packaged the same way. It takes true body-frame specific
  force and angular rate, applies a tactical-grade-MEMS error model (per-run turn-on bias + white noise),
  quantizes to 16-bit register counts at ±40 g / ±2000 °/s sensitivity, and buffers Hemerion IMU raw-sample
  frames into the part's 16 KiB FIFO at `sample_rate_hz` (default 100 Hz — 10 per 0.1 s step, truth
  zero-order-held across the step). It then behaves like the chip it models: `WHO_AM_I`, `STATUS`,
  `FIFO_COUNT`, `CONTROL` and a non-incrementing `FIFO_DATA` burst port, answered over a shared-memory SPI bus
  (`sim/spi_shm/`), with a data-ready line for the controller to sample.
* **`gps_flight_computer`** — host stand-in for the STM32H743 flight computer's sensor ingest paths. GPS bytes
  go through the unmodified `GpsDriver`/`UbxParser`; the IMU is driven by the unmodified **on-target**
  `ImuSpiDriver` — probe, DRDY, `STATUS`+`FIFO_COUNT` in one transfer, burst the FIFO, feed every byte to
  `ImuPacketParser` + `convert_raw_to_si()`. Only the two transport shims differ from the target: a UDP socket
  where the receiver's UART would be, and a shared-memory bus where `HAL_SPI_TransmitReceive` plus the
  CS/DRDY GPIOs would be (or Renode's emulated peripherals fed by the same FMUs, see `tests/swil/`).
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

## The receiver loses its fix, on purpose

`rocket_gps_cosim` writes the GPS FMU's dynamics envelope into its parameter set explicitly rather than
inheriting the defaults silently — whether the receiver keeps a fix through this flight is the scenario's most
consequential setting:

```cpp
launch_site["gps::dynamic_platform"] = -1;     // no platform envelope: COCOM alone
launch_site["gps::cocom_limits_enabled"] = true;
launch_site["gps::reacquisition_time_s"] = 2.0;
```

The FMU models two independent mechanisms and the example enables only one. **COCOM export limits** stop
navigation output above 18 000 m *and* 515 m/s — an AND, which is why a high-altitude balloon and a fast low
sled both keep their fix while a sounding rocket does not. That is not a configuration choice; it is in every
receiver you can buy. The **navigation-engine platform model** (`dynModel`) is a choice — a register a
firmware engineer writes — and it is off here. Both are evaluated against *truth*, not against the noisy fix:
it is the vehicle's real motion that breaks carrier tracking, not the receiver's estimate of it.

**What COCOM costs, measured: 312 fixes out of 2001 epochs, the last at t = 31.2 s.** The receiver works
normally for 31 seconds — position, speed, course, `sats=11` — then stops at 17.9 km and 1.6 km/s and never
reports again, leaving the remaining 169 s including all of stage 2 unnavigated. The AND is what sets the
moment: 515 m/s is passed at t = 10.1 s with nothing happening, because the vehicle is only 2 km up.

Adding the platform model back (`--dyn-model 8`, airborne <4 g, what a launch vehicle ships with) is more
realistic and says much less: its acceleration limit trips on the **second** epoch, because thrust alone is
~54 m/s² and coordinate acceleration off the pad is ~4.6 g, so the receiver reports one usable fix in 2001
epochs and COCOM never gets to be the reason for anything.

The receiver keeps emitting one NAV-PVT frame per epoch throughout, it just stops claiming a solution — fix
type drops to `kNoFix`, satellites to zero, accuracies inflate. The flight computer gates on fix type, logs
the epoch with its `fix_type` but without the meaningless position, and reports the outage window.

`--no-cocom` clears the export limits too, for a waivered receiver that reports through the whole flight.

## The plant is checked, not assumed

None of the sensor results mean anything if the truth driving them is flying the wrong mission — and a plant
with the wrong staging profile still produces perfectly clean-looking GPS and IMU streams, which is what makes
that error easy to miss. NASA TM-2015-218675 publishes check-case output for Scenario 17 and Aetherion ships
it, so `verify_trajectory.py` checks the run against it:

```
python verify_trajectory.py \
    --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_04.csv \
    --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_05.csv \
    --reference <aetherion>/data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_06.csv
```

```
compared 2001 samples of rocket_truth.csv against 3 reference(s)
  references disagree with each other by up to 16.97 km (at t = 200.0 s) -- the envelope is that wide
  t=  37.4 s  stage 1 burnout / separation    ours    26.77 km   reference    26.75..26.76 km
  t= 131.8 s  stage 2 ignition                ours   138.52 km   reference   138.24..138.34 km
  t= 193.0 s  stage 2 burnout                 ours   220.15 km   reference   217.97..230.00 km
OK: every sample inside the reference envelope, within 1.0% + 100 m
```

The criterion is the **envelope** of the three published trajectories rather than any one of them: they are
independent simulators run against the same case and they disagree with each other by 7% at t = 200 s, so
matching one exactly would mean matching that simulator's idiosyncrasies. Our run lies between them at all
2001 samples, and within 20 m of all three at stage-1 burnout.

That check is what caught the example's worst bug. The `TwoStageRocket` FMU's `stg2.ignition_time_s`
parameter defaults to `0` = *ignite stage 2 the moment stage 1 separates*, and this example never set it — so
it flew continuous thrust to t = 99 s with no coast and reached 638 km at t = 200 s against a reference
envelope topping out at 251 km. Every sensor figure still looked entirely plausible. `rocket_gps_cosim` now
sets it explicitly (`--stg2-ignition`, default 131.8 s), along with Scenario 17's equatorial launch site.

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
| `hemerion_gps_fmu.fmu` | `fmus/fmi2/` (also exported to `fmus/fmi3/`) |
| `hemerion_imu_fmu.fmu` | `fmus/fmi2/` (also exported to `fmus/fmi3/`) |

Each sensor FMU is exported for both FMI generations from the same sources. This example uses the **FMI 2.0**
pair, since Ecos imports through fmi4c; `--gps`/`--imu` can point at any archive you like.

## Running

Two terminals, both in `build/examples-native/examples/rocket_gps_ecos/`. Start the flight computer first: it
binds the GPS FMU's UDP destination (5762), then waits for the IMU's SPI bus to appear, so either start order
works — but a controller that probes after the FMU has begun stepping resets the part's FIFO and discards
whatever was buffered before it took over.

```
# terminal 1 — the "STM32" side
./gps_flight_computer

# terminal 2 — the co-simulation
./rocket_gps_cosim
```

The co-simulation runs Scenario 17's own 200 s window at a 0.1 s communication step (= 10 Hz GPS, a typical
u-blox navigation rate; the IMU latches at 100 Hz within each step) and needs no interaction. Stage 1 burns
out and separates at t = 37.4 s, the vehicle **coasts for 94 s**, stage 2 ignites at t = 131.8 s and burns out
at t = 193 s, and the run ends 7 s later at 236 km with the vehicle still climbing — it is a rocket *to
orbit*, so there is no apogee inside the window. The flight computer exits by itself when the master
terminates the IMU FMU — the simulated part powering down is a signal in its own right — or a few seconds
after both streams go quiet.

**Use `--rtf 1` for a lossless IMU stream.** Unpaced, this co-simulation is not uniformly fast: powered,
in-atmosphere flight costs real time (DAVE-ML table lookups on every implicit Radau stage) while thin air and
coasting are cheap. The IMU FMU keeps latching ten samples per step throughout, and where the master
accelerates it can outrun the flight computer's poll loop by more than the loop recovers from — a measured
unpaced run overran the part's 16 KiB FIFO and lost a few hundred of 20010 samples. The sticky overflow bit
reports it and no frame is ever truncated, which is exactly the behaviour a FIFO plus an overflow flag exists
to give you; it is also an artifact of the simulation rather than of the system under test, since a real
plant cannot outrun its consumer. Paced, both reference runs are lossless.

Useful knobs (`--help` lists all):

* `--rocket <path>` / `--gps <path>` / `--imu <path>` — FMU locations, if the configure-time defaults don't
  apply.
* `--imu-rate <hz>` — IMU output data rate (default 100; becomes the FMU's `sample_rate_hz` parameter).
* `--dyn-model <code>` / `--no-cocom` / `--reacq <s>` — the receiver's dynamics envelope (see above).
* `--rtf 1` — pace the co-simulation to wall-clock speed (default is as-fast-as-possible, about 1.7× real
  time on a typical desktop), e.g. to watch the fix stream come in live.
* `HEMERION_GPS_FMU_UDP_HOST` / `HEMERION_GPS_FMU_UDP_PORT` — read by the GPS FMU at instantiation; point them
  at another host to feed a remote consumer (e.g. Renode on a different machine).
* `HEMERION_IMU_FMU_SPI_BUS` (FMU side) and `--imu-bus` (flight-computer side) — the shared-memory SPI bus
  name, default `hemerion_imu_spi`. Set both to run two co-simulations side by side on one machine. A
  shared-memory bus is local by construction; a remote consumer needs Renode's emulated SPI instead.

Ecos does not link a zip library — it spawns `tar`, or `unzip` when `SHELL` names a bash — and reports any
failure as `Failed to unzip ... to tempdir`, which says nothing about why. Two environmental traps produce
that, both common on Windows: Git for Windows puts a **GNU tar** on `PATH` ahead of `System32`'s bsdtar, and
GNU tar reads the leading `C:` of an absolute path as an rsh-style *host* (`tar: Cannot connect to C: resolve
failed`); or running from Git Bash sets `SHELL`, which makes Ecos pick `unzip`, which Git for Windows does
not always ship.

`rocket_gps_cosim` checks for both before the run starts — it extracts one FMU with Ecos' own `unzip()` into
a temporary directory and, if that fails, names the archiver it used and the fix:

```
error: cannot unpack C:/dev/Hemerion/build/examples-native/fmus/fmi2/hemerion_gps_fmu.fmu
       Ecos unpacks every .fmu by spawning 'tar'; that call failed before the simulation started.
       Most likely a GNU tar (Git for Windows' usr\bin) is ahead of Windows' own
       tar on PATH. ...
           set PATH=C:\Windows\System32;%PATH%
```

Outputs land in `results/`:

* `rocket_truth.csv` — Ecos `csv_writer` log of the rocket's outputs (altitude, position, NED velocity, body
  rates, Mach, dynamic pressure, thrust, mass, staging flag) plus the host-computed specific force the IMU FMU
  received, at every communication point.
* `gps_fixes.csv` — every NAV-PVT epoch the flight software decoded, valid or not. Epochs that carried a
  solution get position, speed/course, receiver-reported accuracies and satellite count; epochs the dynamics
  envelope invalidated get **empty** position fields, keeping only the index, time and `fix_type`. A real
  receiver does put numbers in those fields during a dropout and the parser does decode them, but they are
  not measurements of anything, so they are not logged as if they were.
* `rocket_truth.csv` — written by the co-simulation host rather than Ecos' `csv_writer`, at full
  double precision. Six decimal places of a *radian* is 6.4 m of ground position, which would swamp the 1.5 m
  the GPS noise model injects and make the decoded-fix error figure a picture of the log's own rounding.
* `imu_samples.csv` — every IMU sample the flight software decoded, already converted back to SI units by
  `convert_raw_to_si()`: specific force and angular rate per axis, timestamped from the frame payload.

## Plots

`plot_results.py` (matplotlib) turns the three CSVs into the figures used by the Sphinx page
(`doc/rocket_gps_ecos_cosim.rst`): GPS availability against the receiver's envelope, altitude, speed over
ground, the decoded-fix error against truth, and the decoded IMU specific force and body rates against truth.

```
python plot_results.py            # reads results/, writes plots/
```

All six come from the default configuration. The GPS ones cover the 31 s the export limits leave and shade
the rest of the flight as the outage it is; the script skips a fix-dependent figure rather than drawing it
empty if a configuration leaves fewer than two usable epochs (`--dyn-model 8` leaves one).

Every figure is stamped at the foot with the receiver configuration that produced it, read from the `.config`
sidecar `rocket_gps_cosim` writes next to the truth log. Run the same script against `--no-cocom` output and
you get plots identical in form that say the opposite thing; a PNG detached from its caption has no other way
to tell you which it is.

## Notes and design decisions

* **Why byte-level transports and not FMI outputs?** The point of the sensor FMUs is to exercise the
  *byte-level* driver stacks. A real receiver hands the flight computer a UART byte stream, not floating-point
  lat/lon, and a real IMU hands it register contents over SPI, not m/s²; keeping each FMU's output a wire-exact
  byte stream on the bus the part really uses means the firmware-side drivers and parsers are tested against
  exactly what they will see on hardware, checksums and all.
* **Why the IMU is not just another UDP emitter.** An inertial part is polled, and everything interesting
  about driving one is in the polling: the register map, the FIFO that decouples sample rate from poll rate,
  the sticky overflow bit that tells you when you were too slow, the data-ready line, and bursts that land
  mid-frame because a controller cannot align its reads to frame boundaries. `sim/spi_shm` carries
  chip-select-framed transfers between the two processes; one transfer is one handshake, which is exactly the
  granularity a `HAL_SPI_TransmitReceive` call has on the target.
* **Where the split is drawn.** `ImuSpiSlave` is the datasheet (registers, FIFO, shift register) and names no
  transport. `SpiPeripheralEndpoint` is the board (bus naming, lifecycle, service thread) and names no sensor.
  `fmu_main.cpp` is the part number, and mentions no shared-memory type. The FMU boundary stays at the
  physical component, because an IMU chip genuinely is a sensing element *and* an SPI peripheral.
* **What an accelerometer measures.** The IMU FMU's inputs are *specific force* — the non-gravitational force
  per unit mass — not coordinate acceleration. On the pad an unmodeled real part would read +1 g of ground
  reaction; in this plant (no ground-contact model) it reads thrust + aero over mass during boost and
  **zero during coast**, which the decoded stream reproduces: ~55 m/s² at ignition rising to ~265 m/s² at
  stage-2 burnout, then free fall at 0 m/s² all the way to (and past) apogee.
* **Scale is configuration, not wire data.** The IMU frames carry raw counts only, exactly like real silicon;
  the flight computer converts with the same ±40 g / ±2000 °/s `ImuScale` the FMU quantized with, standing in
  for the driver knowing the full-scale range it configured into the part's registers.
* **Fix/sample timestamps.** One NAV-PVT frame is emitted per communication step — including epochs with no
  solution — so the flight computer maps *epoch* index → simulation time (`--fix-period`, default 0.1 s) when
  logging. Counting only valid fixes would shift every later timestamp by the length of the outage. IMU frames
  carry the simulation clock in their payload, so their timestamps come straight off the wire; a controller
  that polls late therefore gets older samples, correctly stamped, not wrong ones.
* **fmi4c compatibility.** Ecos loads FMUs through fmi4c, which resolves the *complete* FMI 2.0 export table
  up front and refuses a binary that omits any function — including the state-management/derivative functions
  whose capability flags are `false`. Neither sensor FMU implements that table by hand: the vendored fmu4cpp
  export layer exports all of it, and `modelDescription.xml` is generated from the variables `fmu_main.cpp`
  registers, so the description and the binary cannot drift apart (the GUID is derived from the model metadata,
  and `fmi2Instantiate` rejects a mismatch).
