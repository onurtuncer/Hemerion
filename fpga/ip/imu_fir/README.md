# fpga/ip/imu_fir/

**Status:** Prototyped outside this tree during an exploratory design
session (Icarus-verified, 5/5 testbench scenarios passing). Not yet
imported here — this README fixes the interface ahead of that import.

## Purpose

Anti-aliasing low-pass filter ahead of the IMU sample path. Filters all 6
axes (accel x/y/z, gyro x/y/z) before they reach Hemerion's `sensors`
module, so software never sees raw 8 kHz ICM-42688-P noise.

## Parameters

- 64-tap symmetric FIR, Q1.15 fixed point
- DSP48E1 cascade: pre-add (1 cycle) → multiply (1 cycle) → adder tree
  (1 cycle) = 3-cycle pipeline latency
- No decimation — filtering only, sample rate unchanged
- 6 parallel channels (`fir_multichannel`), sharing common
  `valid_in`/`valid_out`

## Interface sketch

```verilog
module fir_multichannel (
    input  wire        clk,
    input  wire        rst_n,
    input  wire         valid_in,
    input  wire signed [15:0] accel_x_in, accel_y_in, accel_z_in,
    input  wire signed [15:0] gyro_x_in,  gyro_y_in,  gyro_z_in,
    output wire         valid_out,
    output wire signed [15:0] accel_x_out, accel_y_out, accel_z_out,
    output wire signed [15:0] gyro_x_out,  gyro_y_out,  gyro_z_out
);
```

## Pending import

Generated and verified in the design session, not yet committed:

- `rtl/fir_core.v` — single-channel 64-tap symmetric FIR core
- `rtl/fir_multichannel.v` — 6× `fir_core` instances
- `rtl/sine_tables.vh` — precomputed 100 Hz / 1 kHz sine ROMs (for testbench
  frequency-selectivity checks; `$sin` is unreliable under Icarus)
- `tb/tb_fir_multichannel.v` — 5 scenarios: zero input, DC steady-state,
  6-channel independence, LP frequency selectivity, valid timing
- `Makefile` — `make sim` (Icarus), `make wave` (GTKWave), `make
  vivado-check` (synthesis)

Verified results from the design session: 100 Hz passthrough at 16053
(input amplitude 16000), 1 kHz attenuated to 2070 (7.7× attenuation), DC
rounding error of 1 LSB.

## Phase

2 in the roadmap (see `fpga/README.md`), but prototyped ahead of schedule
— this is the only block with verified RTL so far, built before
`ip/sensor_sync/` (nominally Phase 1).

## Spec history

An earlier design pass specified 128-tap with ×4 decimation (8 kHz →
2 kHz). The parameters above — 64-tap, no decimation — are what was
actually built and Icarus-verified, and supersede the 128-tap spec.

## Next step

Build the Verilator C++ harness (`verilator/`, see
`fpga/verilator/README.md`) before wiring this block into
`HEMERION_BUILD_PL_TESTS`.
