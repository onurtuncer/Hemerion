# fpga/ip/data_logger/

**Status:** Concept only — no RTL written yet.

## Purpose

High-rate PL-side ring buffer for sensor data that Core 1 (FreeRTOS) can't
log fast enough in software. Drains to Core 0 (Linux) over AXI for
storage, complementing `modules/datalogger` rather than replacing it.

## Phase

3

## Open questions

- Buffer depth and BRAM budget
- Drain protocol to Core 0 — polled or interrupt-driven
- Whether this is generic (any AXI-streamed source) or IMU-FIR-specific

No interface has been fixed.
