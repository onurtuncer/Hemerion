#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""scripts/compare_run_values.py: two runs' value columns, compared exactly, timing columns left out.

    python -m unittest tests/unit/test_compare_run_values.py
"""
from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import compare_run_values as cmp  # noqa: E402


def write_run(root: Path, stamp_offset: float = 0.0, pacing_lag: str | None = None) -> Path:
    """A small run whose arrival stamps are shifted by stamp_offset; every value is fixed."""
    root.mkdir(parents=True)
    gps = ["fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m"]
    gps += [f"{i},{(i - 1) * 0.1},{(i - 1) * 0.1 + stamp_offset},36.0{i},-75.6,3052.{i}" for i in range(1, 21)]
    (root / "gps_fixes.csv").write_text("\n".join(gps) + "\n", encoding="utf-8")
    imu = ["sample_index,part_time_s,host_time_s,accel_x_mps2"]
    imu += [f"{k},{k * 0.01},{k * 0.01 + stamp_offset},0.{k}" for k in range(1, 101)]
    (root / "imu_samples.csv").write_text("\n".join(imu) + "\n", encoding="utf-8")
    mag = ["sample_index,imu_part_time_s,host_time_s,mag_x_ut"]
    mag += [f"{k},{k * 0.02 + stamp_offset},{k * 0.02 + 2 * stamp_offset},20.{k}" for k in range(1, 51)]
    (root / "mag_samples.csv").write_text("\n".join(mag) + "\n", encoding="utf-8")
    truth = ["iterations, time, f16::out.alt_m[REAL]"] + [f"{k}, {k * 0.01}, 3052.{k}" for k in range(0, 101)]
    (root / "f16_truth.csv").write_text("\n".join(truth) + "\n", encoding="utf-8")
    sidecar = "seed=5\nrealtime_factor=1\n" + (f"pacing_max_lag_s={pacing_lag}\n" if pacing_lag else "")
    (root / "f16_truth.config").write_text(sidecar, encoding="utf-8")
    return root


class CompareRunValues(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def run_main(self, *args) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            status = cmp.main([str(arg) for arg in args])
        return status, out.getvalue()

    def test_timing_columns_and_pacing_keys_are_not_values(self):
        # Different arrival stamps, IMU alignment and pacing report; the same values.
        a = write_run(self.root / "a", pacing_lag="0.004")
        b = write_run(self.root / "b", stamp_offset=0.37, pacing_lag="59.7")
        status, text = self.run_main(a, b)
        self.assertEqual(status, 0, text)
        self.assertIn("same values: 5 files compared", text)

    def test_new_sidecar_keys_are_noted_not_failed(self):
        a = write_run(self.root / "a")
        b = write_run(self.root / "b", pacing_lag="1.5")
        status, text = self.run_main(a, b)
        self.assertEqual(status, 0)
        self.assertIn("keys only in the second run: pacing_max_lag_s", text)

    def test_a_changed_value_is_caught_to_the_last_digit(self):
        a = write_run(self.root / "a")
        b = write_run(self.root / "b")
        path = b / "imu_samples.csv"
        path.write_text(path.read_text(encoding="utf-8").replace(",0.42\n", ",0.420001\n"), encoding="utf-8")
        status, text = self.run_main(a, b)
        self.assertEqual(status, 1)
        self.assertIn("DIFFERS: imu_samples.csv part_time_s 0.42: accel_x_mps2 '0.42' vs '0.420001'", text)

    def test_a_lost_sample_and_a_changed_configuration_are_caught(self):
        a = write_run(self.root / "a")
        b = write_run(self.root / "b")
        lines = (b / "imu_samples.csv").read_text(encoding="utf-8").splitlines()
        (b / "imu_samples.csv").write_text("\n".join(lines[:1] + lines[2:]) + "\n", encoding="utf-8")
        (b / "f16_truth.config").write_text("seed=6\nrealtime_factor=1\n", encoding="utf-8")
        status, text = self.run_main(a, b)
        self.assertEqual(status, 1)
        self.assertIn("imu_samples.csv: 100 rows against 99", text)
        self.assertIn("f16_truth.config: seed = '5' vs '6'", text)

    def test_skip_and_missing(self):
        a = write_run(self.root / "a")
        b = write_run(self.root / "b")
        (b / "mag_samples.csv").write_text("sample_index,imu_part_time_s,host_time_s,mag_x_ut\n", encoding="utf-8")
        self.assertEqual(self.run_main(a, b)[0], 1)
        self.assertEqual(self.run_main(a, b, "--skip", "mag")[0], 0)
        (b / "mag_samples.csv").unlink()
        self.assertIn("missing from", self.run_main(a, b)[1])
        empty = self.root / "empty"
        empty.mkdir()
        self.assertEqual(self.run_main(empty, b)[0], 2)


if __name__ == "__main__":
    unittest.main()
