# fpga/ip/sensor_sync/

**Status:** Concept only — no RTL written yet.

## Purpose

Align IMU `DRDY`, GPS PPS, and barometer sample edges to a common
timestamp in hardware, so Hemerion's EKF doesn't have to compensate for
sensor-to-sensor skew in software.

## Phase

1 (alongside `ip/imu_fir/`)

## Open questions

- Edge-detection width and debounce strategy for each input
- Output timestamp resolution and counter width
- Whether this block also tags samples for `ip/data_logger/`, or stays a
  pure timestamp generator consumed elsewhere

No interface has been fixed. Do not start RTL until these are answered.
