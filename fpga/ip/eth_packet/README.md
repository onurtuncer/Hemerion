# fpga/ip/eth_packet/

**Status:** Concept only — no RTL written yet.

## Purpose

Hardware packet classification/offload ahead of the existing UDP-based
Aetherion bridge (`sim/udp_bridge`), to free Core 0 from per-packet
software parsing at high HIL rates.

## Phase

3

## Open questions

- Whether this targets the existing UDP protocol (ports 5760/5761 from the
  design-reference docs) as-is, or a revised framing
- Split of work between this block and `ip/hil_accel/` — they may turn out
  to be the same block

No interface has been fixed.
