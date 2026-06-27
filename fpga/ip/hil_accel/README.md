# fpga/ip/hil_accel/

**Status:** Concept only — no RTL written yet.

## Purpose

Low-latency PL-side data path for the Aetherion HIL loop
(Aetherion → PL → Hemerion → PL → Aetherion), bypassing software-timed
transport for the highest-rate sensors.

## Phase

3

## Open questions

- Depends on `ip/eth_packet/` or a dedicated AXI path to Aetherion — not
  decided
- Whether this is a distinct block or folds into `ip/eth_packet/`

No interface has been fixed. Do not start this block before
`ip/imu_fir/` and `ip/sensor_sync/` (Phase 1) are working end to end.
