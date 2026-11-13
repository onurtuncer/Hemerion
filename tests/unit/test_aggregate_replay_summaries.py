#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""scripts/aggregate_replay_summaries.py against summaries shaped like the pinned EKFReplay's, and the one
threshold scripts/run_integration_case.py derives from the run length.

    python -m unittest tests/unit/test_aggregate_replay_summaries.py
"""
from __future__ import annotations

import contextlib
import csv
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import aggregate_replay_summaries as agg  # noqa: E402
import run_integration_case as gate  # noqa: E402

# Verbatim from a run of the pinned EKFReplay (Aetherion 49e75b8) on case 11, seed 11, 20 s.
SUMMARY = """convention: right-invariant
time base: host = -0.737688 + 1.23296 * part over 1994 IMU samples, residual rms 160.956 ms, max 680.155 ms
duration: 19.93 s, 1993 IMU steps, 199 GNSS position, 199 GNSS velocity, 122 barometer updates, 0 fixes skipped
not used: 0 IMU samples not finite, 0 IMU samples out of order, 0 truth records not finite, 0 measurements with no \
finite time, 0 updates refused by the filter
second half: ANEES (nav core, /9) 0.452661, NIS/dim GNSS pos 0.942716, GNSS vel 1.08823, baro 0.109845
second half RMS: position 0.551479 m, velocity 0.212467 m/s, tilt 0.216214 deg, yaw 1.10821 deg
(bias NEES needs the realised biases, which the sensor FMUs do not publish yet)
"""


def write_run(root: Path, name: str, seed: int, summary: str | None, passed: bool = True,
              turbulence: float = 8.0) -> Path:
    run = root / name
    run.mkdir(parents=True)
    manifest = {"case": {"seed": seed, "stop_s": 200.0, "turbulence_w20_mps": turbulence,
                         "turbulence_seed": seed if turbulence > 0 else None},
                "passed": passed, "hemerion_commit": "h" * 40, "aetherion_commit": "a" * 40,
                "checks": {"cosim_wall_s": 201.5}}
    (run / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (run / "f16_truth.config").write_text(f"seed={seed}\nturbulence_w20_mps={turbulence:g}\nturbulence_seed="
                                          f"{seed if turbulence > 0 else 1}\nstop_s=200\n", encoding="utf-8")
    if summary is not None:
        (run / "EKFReplay_summary.txt").write_text(summary, encoding="utf-8")
    return run


class ParseSummary(unittest.TestCase):
    def test_reads_every_figure_of_the_pinned_replay(self):
        figures = agg.parse_summary(SUMMARY)
        self.assertEqual(figures["convention"], "right-invariant")
        self.assertEqual(figures["duration_s"], 19.93)
        self.assertEqual((figures["imu_steps"], figures["gnss_pos_updates"], figures["gnss_vel_updates"],
                          figures["baro_updates"], figures["fixes_skipped"]), (1993, 199, 199, 122, 0))
        self.assertEqual(figures["anees_nav"], 0.452661)
        self.assertEqual((figures["nis_gnss_pos"], figures["nis_gnss_vel"], figures["nis_baro"]),
                         (0.942716, 1.08823, 0.109845))
        self.assertEqual((figures["rms_position_m"], figures["rms_velocity_mps"], figures["rms_tilt_deg"],
                          figures["rms_yaw_deg"]), (0.551479, 0.212467, 0.216214, 1.10821))

    def test_a_new_measurement_gets_its_own_column(self):
        figures = agg.parse_summary(SUMMARY.replace("baro 0.109845", "baro 0.109845, mag 0.97")
                                    .replace("0 fixes skipped", "0 fixes skipped, 50 magnetometer updates"))
        self.assertEqual(figures["nis_mag"], 0.97)
        self.assertEqual(figures["n_magnetometer_updates"], 50)

    def test_non_finite_values_are_kept_as_non_finite(self):
        figures = agg.parse_summary(SUMMARY.replace("/9) 0.452661", "/9) -nan(ind)").replace("yaw 1.10821", "yaw inf"))
        self.assertTrue(math.isnan(figures["anees_nav"]))
        self.assertTrue(math.isinf(figures["rms_yaw_deg"]))

    def test_a_relabelled_figure_is_not_read_as_the_old_one(self):
        figures = agg.parse_summary(SUMMARY.replace("barometer updates", "baro updates"))
        self.assertNotIn("baro_updates", figures)
        self.assertEqual(figures["n_baro_updates"], 122)


class Aggregate(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def run_main(self, *args: str) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            status = agg.main([*args])
        return status, out.getvalue()

    def test_rows_by_seed_with_a_failed_replay_kept_and_statistics_over_the_rest(self):
        write_run(self.root, "case11-seed10", 10, SUMMARY.replace("0.452661", "0.6"))
        write_run(self.root, "case11-seed2", 2, SUMMARY)
        write_run(self.root, "case11-seed3", 3, None, passed=False)
        csv_path = self.root / "out" / "table.csv"
        status, text = self.run_main(str(self.root), "--csv", str(csv_path))
        self.assertEqual(status, 0)

        with csv_path.open(newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual([row["seed"] for row in rows], ["2", "3", "10"])
        self.assertEqual([row["passed"] for row in rows], ["yes", "no", "yes"])
        self.assertEqual(rows[1]["summary"], "no")
        self.assertEqual(rows[1]["anees_nav"], "")
        self.assertEqual(rows[0]["turbulence_seed"], "2")
        self.assertEqual(rows[0]["turbulence_w20_mps"], "8")
        self.assertIn("3 run(s): 2 passed every check, 2 have an EKFReplay summary.", text)
        self.assertIn("| anees_nav | 2 | 0.526331 |", text)

    def test_calm_runs_name_no_turbulence_seed(self):
        write_run(self.root, "calm", 11, SUMMARY, turbulence=0.0)
        _, text = self.run_main(str(self.root / "calm"))
        self.assertIn("| calm | 11 | 0 |  | 200 | yes |", text)

    def test_strict_and_empty(self):
        write_run(self.root, "ok", 1, SUMMARY)
        self.assertEqual(self.run_main(str(self.root), "--strict")[0], 0)
        write_run(self.root, "bad", 2, None, passed=False)
        self.assertEqual(self.run_main(str(self.root), "--strict")[0], 1)
        empty = self.root / "empty"
        empty.mkdir()
        self.assertEqual(self.run_main(str(empty))[0], 2)


class GateThreshold(unittest.TestCase):
    def test_the_gate_still_wants_150_fixes_and_a_longer_run_proportionally_more(self):
        self.assertEqual(gate.min_3d_fixes(gate.CASE), 150)
        self.assertEqual(gate.min_3d_fixes(dict(gate.CASE, stop_s=200.0)), 1500)


if __name__ == "__main__":
    unittest.main()
