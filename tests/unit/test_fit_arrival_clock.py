#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""scripts/fit_arrival_clock.py against synthetic flight-computer logs whose clock is known.

    python -m unittest tests/unit/test_fit_arrival_clock.py
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import fit_arrival_clock as fit  # noqa: E402

GPS_HEADER = ("fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m,ground_speed_mps,course_deg,"
              "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type,vel_north_mps,vel_east_mps,"
              "vel_down_mps,speed_accuracy_mps")
IMU_HEADER = "sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s"
BARO_HEADER = "sample_index,part_time_s,host_time_s,pressure_pa,temperature_c"
MAG_HEADER = "sample_index,imu_part_time_s,host_time_s,mag_x_ut,mag_y_ut,mag_z_ut"


def write_run(root: Path, host, gps_epochs: int = 200, baro_part=None) -> Path:
    """Logs whose host_time_s is host(reference). baro_part maps the reference time to the logged SENSORTIME."""
    root.mkdir(parents=True, exist_ok=True)
    lines = [GPS_HEADER]
    for i in range(1, gps_epochs + 1):
        t = (i - 1) * 0.1
        # A no-fix epoch keeps its times and leaves the solution empty, as the flight computer logs it.
        fix = "36,-75,3000,170,45,1.5,3,11,2,1,1,0,0.07" if i % 10 else ",,,,,,,0,0,,,,"
        lines.append(f"{i},{t},{host(t)},{fix}")
    (root / "gps_fixes.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    lines = [IMU_HEADER] + [f"{k},{k * 0.01},{host(k * 0.01)},0,0,-9.8,0,0,0" for k in range(1, 2001)]
    (root / "imu_samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    baro_part = baro_part or (lambda t: t)
    lines = [BARO_HEADER] + [f"{k},{baro_part(k * 0.02)},{host(k * 0.02)},70000,5" for k in range(1, 1001)]
    (root / "baro_samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    lines = [MAG_HEADER] + [f"{k},{k * 0.02},{host(k * 0.02) + 3.0},20,0,40" for k in range(1, 1001)]
    (root / "mag_samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return root


class FitArrivalClock(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def run_main(self, *args: str) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            status = fit.main([*args])
        return status, out.getvalue()

    def test_an_exact_clock_is_recovered_on_every_stream(self):
        run = write_run(self.root / "exact", lambda t: 2.5 + 1.25 * t)
        fits = fit.fit_run(run)
        self.assertEqual(sorted(fits), ["baro", "gps", "imu", "mag"])
        for stream, f in fits.items():
            self.assertAlmostEqual(f["scale"], 1.25, places=9, msg=stream)
            self.assertAlmostEqual(f["offset_s"], 5.5 if stream == "mag" else 2.5, places=9, msg=stream)
            self.assertLess(f["rms_s"], 1e-9, msg=stream)
        self.assertEqual(fits["gps"]["rows"], 200)  # no-fix epochs keep their times and count

    def test_residual_of_a_known_sawtooth(self):
        # Arrival quantised to a 0.01 s step: a uniform residual, rms 0.01 / sqrt(12) = 2.9 ms, max 5 ms.
        run = write_run(self.root / "saw", lambda t: math.floor(t / 0.01 + 1e-9) * 0.01 + 0.004)
        f = fit.fit_run(run)["imu"]
        self.assertAlmostEqual(f["scale"], 1.0, places=6)
        self.assertLess(f["rms_s"], 1e-6)  # the IMU's own times sit on the grid, so nothing is left
        f = fit.fit_run(run)["gps"]
        self.assertLess(f["max_abs_s"], 0.006)

    def test_sensortime_wrap_is_unwrapped(self):
        # 600 s of baro at 0.6 s spacing would cross one wrap; build it directly to keep the file small.
        run = write_run(self.root / "wrap", lambda t: 1.0 + t, baro_part=lambda t: (t * 30.0) % fit.SENSORTIME_WRAP_S)
        # host = 1 + t, part = 30 t (mod 512): unwrapped, host = 1 + part / 30.
        f = fit.fit_run(run)["baro"]
        self.assertAlmostEqual(f["scale"], 1.0 / 30.0, places=9)
        self.assertLess(f["rms_s"], 1e-9)
        self.assertEqual(fit.unwrap([510.0, 511.5, 1.0, 2.0], 512.0), [510.0, 511.5, 513.0, 514.0])

    def test_check_mode_and_outputs(self):
        good = write_run(self.root / "good", lambda t: 0.3 + t)
        self.assertEqual(self.run_main(str(good), "--max-scale-error", "0.02", "--max-rms-s", "0.02")[0], 0)
        lagging = write_run(self.root / "lag", lambda t: 0.3 + 1.3 * t + 0.3 * math.sin(t))
        status, text = self.run_main(str(lagging), "--max-scale-error", "0.02", "--max-rms-s", "0.02",
                                     "--json", str(self.root / "fits.json"))
        self.assertEqual(status, 1)
        self.assertRegex(text, r"FAIL: gps: scale 1\.29\d+ is more than 0\.02 from 1")
        self.assertIn("exceeds 20 ms", text)
        self.assertEqual(sorted(json.loads((self.root / "fits.json").read_text())), ["baro", "gps", "imu", "mag"])
        empty = self.root / "empty"
        empty.mkdir()
        self.assertEqual(self.run_main(str(empty))[0], 2)

    def test_unusable_stream_is_reported_not_raised(self):
        run = write_run(self.root / "flat", lambda t: 1.0)
        (run / "imu_samples.csv").write_text(IMU_HEADER + "\n1,5,1,0,0,0,0,0,0\n", encoding="utf-8")
        fits = fit.fit_run(run)
        self.assertIn("error", fits["imu"])
        self.assertEqual(fit.violations(fits, None, None), ["imu: fewer than two rows"])


if __name__ == "__main__":
    unittest.main()
