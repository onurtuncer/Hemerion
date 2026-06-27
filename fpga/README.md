# fpga/

Programmable-logic IP blocks for the optional Zynq-7020 AMP mezzanine. This
directory is **planned, not implemented** — no RTL has landed in this tree
yet. It exists ahead of code so the intended structure is fixed before the
first IP block is imported.

The mezzanine itself is an **evaluated alternative** to the STM32H743 path,
not a committed replacement. `bsp/stm32h743_nucleo` remains the primary
HWIL/SWIL target; see `bsp/README.md` for how the Zynq AMP targets relate to
it.

### Design history — two mezzanine iterations

The exploratory design work went through two distinct shapes; only the
second is what `bsp/zynq_core0`/`zynq_core1` and this directory target.

1. **ZMX-1 — Ethernet-bridged add-on (superseded).** STM32H743 stays as
   the primary MCU; Zynq is a separate board reachable only over the
   existing Aetherion UDP/Ethernet link. Hemerion would gain a
   `ZmxBridge` driver behind an `IEthTransport` interface, selected by an
   `HEMERION_ZMX_SUPPORT` build flag and a `zynq-ps-arm` preset. **None of
   this — `ZmxBridge`, `IEthTransport`, `HEMERION_ZMX_SUPPORT`,
   `zynq-ps-arm` — should be implemented.** It's recorded here only so the
   names don't resurface as a mystery later.
2. **Zynq-7020 AMP split (current target).** STM32H743 is retired
   entirely; Zynq's two PS cores split Linux (Core 0) and FreeRTOS
   (Core 1, running Hemerion's existing tasks near-unchanged) per
   `bsp/README.md`'s "AMP targets" section. This is what every IP block
   below assumes.

The IMU FIR spec changed between the two iterations too: ZMX-1 specified
128-tap with ×4 decimation (8 kHz → 2 kHz); the RTL actually prototyped
and Icarus-verified afterwards is 64-tap with no decimation (see
`ip/imu_fir/README.md`). The verified version is authoritative.

---

## IP blocks

| Block | Purpose | Phase |
|---|---|---|
| `ip/sensor_sync/` | GPS PPS–aligned timestamp synchronizer for IMU/baro/GPS sample alignment | 1 |
| `ip/imu_fir/` | 64-tap symmetric FIR, 6 channels (accel + gyro) on DSP48E1, anti-aliasing ahead of the IMU sample path | 2 |
| `ip/pwm_dshot/` | Multi-channel PWM / DShot waveform generator, FPGA-timed | 2 |
| `ip/uart_mux/` | 16-channel UART multiplexer — GPS, telemetry, payload, debug consolidated into one IP, one PS interface | 2–3 *(unscheduled)* |
| `ip/can_fd/` | CAN FD controller core | 2–3 *(unscheduled)* |
| `ip/hil_accel/` | HIL loop accelerator — low-latency Aetherion ↔ PL data path | 3 |
| `ip/data_logger/` | High-rate PL-side ring buffer logger, drains to Core 0 over AXI | 3 |
| `ip/eth_packet/` | Hardware packet classifier/offload ahead of the Aetherion ETH bridge | 3 |
| `ip/ekf_predict/` | EKF predict step as a Vivado HLS block, derived from the same CppADCodeGen Jacobian as the software EKF | 3 |

Phase numbers follow the roadmap below, **except `imu_fir`**: it was
prototyped ahead of schedule (Icarus-verified RTL exists from an
exploratory session) before `sensor_sync`, which the roadmap nominally
schedules first. Treat the Phase column as planning intent, not a strict
gate — `ip/<name>/README.md` Status lines are the source of truth for
what actually exists.

---

## Platform selection

Evaluated against four alternatives; **Zynq-7020** is the target.

| Platform | PS | PL | RAM | Toolchain | Verdict |
|---|---|---|---|---|---|
| Zynq-7010 | 2× A9 @ 667 MHz | 28K LUT | 512 MB | Vivado (closed) | PL too small |
| **Zynq-7020** | 2× A9 @ 667 MHz | 85K LUT, 220 DSP48E1 | 512 MB DDR3 | Vivado (closed) | **Selected** |
| Zynq-7045 | 2× A9 @ 800 MHz | 218K LUT | 1 GB DDR3 | Vivado (closed) | Overpowered for this scope |
| UltraScale+ ZU3 | 4× A53 @ 1.2 GHz | 71K LUT | 2 GB DDR4 | Vitis (closed) | Candidate for a later platform generation, not now |
| Lattice ECP5 | none (PL only) | 84K LUT | — | nextpnr/Yosys (open) | No PS — doesn't fit the AMP split |

Rationale: 85K LUT / 220 DSP48E1 covers the FIR + motor-control + HIL IP
budget; GigE MAC and AXI DMA are on-die; the platform is well-known in the
target research-institution market (DLR, ONERA, university programs use
Zynq-7020 dev boards like MicroZed, Cora Z7, PYNQ-Z2 already), which
matters more for adoption than raw LUT count.

---

## Roadmap (planning reference, not a schedule commitment)

| Phase | Window | Board | Cost | Scope | Exit criteria |
|---|---|---|---|---|---|
| 1 | 0–3 mo | PYNQ-Z2 | ~€200 | ETH bridge, `sensor_sync` (GPS PPS) | Validated against Renode SWIL |
| 2 | 3–8 mo | MicroZed + carrier | ~€800 | `imu_fir`, `pwm_dshot`, 8 kHz→2 kHz pipeline | Integration test against `bsp/stm32h743_nucleo` (or `zynq_core1` if the AMP split has landed by then) |
| 3 | 8–18 mo | same | — | `hil_accel`, `data_logger`, `ekf_predict` (→ Vivado HLS) | Full SWIL→HWIL transition; this is also where commercial/institutional licensing of the mezzanine would start, if pursued |
| 4 | 18–30 mo | ZU5EG (UltraScale+, codename ZMX-2) | — | Platform migration: R5 real-time core + A53 Linux, optional DPU/AI block | Targets research-institution licensing (DLR/ONERA/TU Delft-class), not a Hemerion architecture change per se |

---

## Internal layout (per IP block)

```
fpga/ip/<name>/
├── README.md         # Status, interface, parameters
├── rtl/               # Synthesizable Verilog
├── tb/                # Icarus Verilog testbenches (standalone RTL verification)
└── verilator/         # Optional: Verilator C++ harness for Catch2/CTest integration
```

---

## Verification strategy

Two independent levels, matching the rest of Hemerion's test pyramid:

1. **RTL-level** (`tb/`) — Icarus Verilog testbenches, run standalone
   (`make sim` per block). Fast iteration on the waveform itself.
2. **Harness-level** (`verilator/`) — Verilator-compiled C++ model driven by
   Catch2, intended to fold into the same `test-native` / CTest pipeline as
   the rest of Hemerion. See `fpga/verilator/README.md`. Not wired into
   `HEMERION_BUILD_TESTS` yet.

Synthesis target is Vivado (Zynq-7020 / `xc7z020`); a `make vivado-check`
target per block is the long-term CI gate once a board file exists.

---

## Relationship to bsp/ and modules/

PL IP blocks are consumed by `bsp/zynq_core1/` (the FreeRTOS-side AMP
target) over AXI/EMIO. Module code in `modules/` never references `fpga/`
directly — same separation `bsp/README.md` enforces for hardware-specific
headers. A module that wants PL acceleration goes through the BSP's HAL
abstraction, not this directory.

---

## Adding a new IP block

1. Copy the layout above into `fpga/ip/<your_block>/`.
2. Write the Icarus testbench first (`tb/`) — don't skip straight to
   Verilator.
3. Add a row to the table above and link the block's README.

This mirrors `modules/README.md`'s "Adding a new module" flow intentionally
— same discipline, different toolchain.
