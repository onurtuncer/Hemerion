# fpga/ip/uart_mux/

**Status:** Concept only — no RTL written yet.

## Purpose

16-channel UART multiplexer in PL. Consolidates GPS, telemetry, payload,
and debug UARTs into one IP block with a single PS-facing interface,
instead of each channel needing its own peripheral and interrupt.

## Phase

2–3, unscheduled in the roadmap — lower priority than `ip/imu_fir/` and
`ip/pwm_dshot/`.

## Open questions

- Per-channel buffer depth
- Whether channel count is fixed at synthesis time or runtime-configurable
- PS-side interface: single AXI-Lite register block vs. one DMA stream per
  channel

No interface has been fixed.
