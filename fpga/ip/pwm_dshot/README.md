# fpga/ip/pwm_dshot/

**Status:** Concept only — no RTL written yet.

## Purpose

Multi-channel PWM / DShot waveform generation in PL instead of MCU timers,
to remove cross-channel jitter on systems with many actuator channels.

## Phase

2

## Open questions

- Channel count
- DShot variant (150 / 300 / 600 / 1200) or PWM-only
- Whether channels are independently configurable at runtime or fixed at
  synthesis time

No interface has been fixed.
