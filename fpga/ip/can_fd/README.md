# fpga/ip/can_fd/

**Status:** Concept only — no RTL written yet.

## Purpose

CAN FD controller core in PL, as a complement to (not a replacement for)
`modules/comms`' CAN framing on the MCU/Core1 side.

## Phase

2

## Open questions

- Whether this wraps an existing open CAN FD core or is written from
  scratch
- How frames cross to `modules/comms` — AXI-Lite register interface vs.
  shared memory ring

No interface has been fixed.
