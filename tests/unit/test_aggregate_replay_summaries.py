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
        self.assertIn("| calm |  | 11 | 0 |  | 200 | yes |", text)

    def test_case_from_the_sidecar_and_no_nis_for_a_measurement_never_applied(self):
        no_fix = (SUMMARY.replace("199 GNSS position, 199 GNSS velocity", "0 GNSS position, 0 GNSS velocity")
                  .replace("GNSS pos 0.942716, GNSS vel 1.08823", "GNSS pos 0, GNSS vel 0"))
        run = write_run(self.root, "case12-seed1", 1, no_fix)
        with (run / "f16_truth.config").open("a", encoding="utf-8") as f:
            f.write("check_case=12\n")
        csv_path = self.root / "table.csv"
        self.assertEqual(self.run_main(str(run), "--csv", str(csv_path))[0], 0)
        with csv_path.open(newline="") as f:
            row = next(csv.DictReader(f))
        self.assertEqual(row["case"], "12")
        self.assertEqual((row["nis_gnss_pos"], row["nis_gnss_vel"], row["nis_baro"]), ("", "", "0.109845"))

    def test_strict_and_empty(self):
        write_run(self.root, "ok", 1, SUMMARY)
        self.assertEqual(self.run_main(str(self.root), "--strict")[0], 0)
        write_run(self.root, "bad", 2, None, passed=False)
        self.assertEqual(self.run_main(str(self.root), "--strict")[0], 1)
        empty = self.root / "empty"
        empty.mkdir()
        self.assertEqual(self.run_main(str(empty))[0], 2)

    def read_csv(self, *paths: Path) -> list[dict[str, str]]:
        csv_path = self.root / "clock.csv"
        self.assertEqual(self.run_main(*map(str, paths), "--csv", str(csv_path))[0], 0)
        with csv_path.open(newline="") as f:
            return list(csv.DictReader(f))

    def test_clock_columns_from_the_manifest_and_the_sidecar(self):
        # A seed that kept real time and one that fell behind, as the runner records them.
        kept = write_run(self.root, "case11-seed2", 2, SUMMARY)
        behind = write_run(self.root, "case11-seed1", 1, SUMMARY)
        fits = {"kept": {"gps": {"scale": 1.00001, "rms_s": 0.0011}, "imu": {"scale": 0.99999, "rms_s": 0.0016}},
                "behind": {"gps": {"scale": 1.3034, "rms_s": 0.3077}, "imu": {"scale": 1.3034, "rms_s": 0.3075},
                           "mag": {"error": "fewer than two rows"}}}
        for run, key, ok, lag, honoured in ((kept, "kept", True, 0.004, 1), (behind, "behind", False, 59.7, 0)):
            manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
            manifest.update({"clock_fit": fits[key], "clock_ok": ok})
            (run / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            with (run / "f16_truth.config").open("a", encoding="utf-8") as f:
                f.write(f"pacing_rtf_honoured={honoured}\npacing_max_lag_s={lag}\n")
        rows = self.read_csv(self.root)
        self.assertEqual([row["seed"] for row in rows], ["1", "2"])
        self.assertEqual([row["clock_ok"] for row in rows], ["no", "yes"])
        self.assertEqual([row["pacing_rtf_honoured"] for row in rows], ["no", "yes"])
        self.assertEqual(rows[0]["clock_scale"], "1.3034")    # the stream furthest from 1
        self.assertEqual(rows[1]["clock_scale"], "1.00001")
        self.assertEqual(rows[0]["clock_rms_s"], "0.3077")    # the largest residual; the unusable stream skipped
        self.assertEqual(rows[0]["pacing_max_lag_s"], "59.7")
        # The clock columns sit after the accuracy figures, before the trailer.
        names = list(rows[0])
        self.assertLess(names.index("rms_yaw_deg"), names.index("clock_scale"))
        self.assertLess(names.index("pacing_rtf_honoured"), names.index("cosim_wall_s"))

    def test_clock_fitted_from_the_logs_of_an_older_run(self):
        # A run from before the runner recorded clock_fit: the aggregator fits the logs it has.
        run = write_run(self.root, "old", 3, SUMMARY)
        rows = ["sample_index,part_time_s,host_time_s,pressure_pa,temperature_c"]
        rows += [f"{k},{0.02 * k},{0.5 + 1.3 * 0.02 * k},70000,5" for k in range(1, 501)]
        (run / "baro_samples.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
        row = self.read_csv(run)[0]
        self.assertAlmostEqual(float(row["clock_scale"]), 1.3, places=6)
        self.assertEqual(row["clock_ok"], "no")
        self.assertEqual(row["pacing_rtf_honoured"], "")  # this host wrote no pacing keys

    def test_no_logs_and_no_fit_leave_the_clock_columns_empty(self):
        row = self.read_csv(write_run(self.root, "bare", 4, SUMMARY))[0]
        self.assertEqual((row["clock_scale"], row["clock_rms_s"], row["clock_ok"]), ("", "", ""))


class GateClock(unittest.TestCase):
    """The runner's clock assessment: recorded and flagged, never a failure."""

    def test_assess_clock_flags_out_of_bounds_and_records_the_fit(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            rows = ["sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_rad_s,"
                    "gyro_y_rad_s,gyro_z_rad_s"]
            rows += [f"{k},{0.01 * k},{0.2 + 0.01 * k},0,0,-9.8,0,0,0" for k in range(1, 1001)]
            (out / "imu_samples.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
            clock = gate.assess_clock(out, gate.CLOCK_MAX_SCALE_ERROR, gate.CLOCK_MAX_RMS_S)
            self.assertTrue(clock["clock_ok"])
            self.assertAlmostEqual(clock["clock_fit"]["imu"]["scale"], 1.0, places=9)
            self.assertEqual(clock["clock_bounds"], {"max_scale_error": 0.02, "max_rms_s": 0.02})

            rows = rows[:1] + [f"{k},{0.01 * k},{0.2 + 0.013 * k},0,0,-9.8,0,0,0" for k in range(1, 1001)]
            (out / "imu_samples.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
            clock = gate.assess_clock(out, 0.02, 0.02)
            self.assertFalse(clock["clock_ok"])
            self.assertEqual(len(clock["clock_flags"]), 1)
            self.assertIn("scale 1.3", clock["clock_flags"][0])
            # Looser bounds, as --clock-max-scale-error would give: no flag.
            self.assertTrue(gate.assess_clock(out, 0.5, 0.02)["clock_ok"])

    def test_nothing_to_fit_is_flagged(self):
        with tempfile.TemporaryDirectory() as tmp:
            clock = gate.assess_clock(Path(tmp), 0.02, 0.02)
            self.assertFalse(clock["clock_ok"])
            self.assertEqual(clock["clock_flags"], ["no flight-computer logs to fit"])


class GateThreshold(unittest.TestCase):
    def test_the_gate_still_wants_150_fixes_and_a_longer_run_proportionally_more(self):
        self.assertEqual(gate.min_3d_fixes(gate.CASE), 150)
        self.assertEqual(gate.min_3d_fixes(dict(gate.CASE, stop_s=200.0)), 1500)

    def test_every_case_has_a_host_and_only_case_12_flies_without_a_fix(self):
        self.assertEqual(gate.CASES[gate.CASE["case"]]["host"], "f16_trim_cosim")
        self.assertEqual([name for name, case in gate.CASES.items() if not case["gnss_fix"]], ["12"])
        self.assertEqual({case["host"] for name, case in gate.CASES.items() if name.startswith("13.")},
                         {"f16_autopilot_cosim"})

    def test_a_case_with_no_fix_wants_every_epoch_and_no_3d_fix(self):
        with tempfile.TemporaryDirectory() as tmp:
            gps = Path(tmp) / "gps_fixes.csv"
            header = ("fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m,ground_speed_mps,"
                      "course_deg,horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type,vel_north_mps,"
                      "vel_east_mps,vel_down_mps,speed_accuracy_mps\n")
            no_fix = "".join(f"{i},{(i - 1) * 0.1},{i * 0.1},,,,,,,,0,0,,,,\n" for i in range(1, 201))
            gps.write_text(header + no_fix, encoding="utf-8")
            failures: list[str] = []
            self.assertEqual(gate.check_gnss_velocity(gps, failures, 0, max_fixes=0, min_epochs=150), 0)
            self.assertEqual(failures, [])
            gate.check_gnss_velocity(gps, failures, 0, max_fixes=0, min_epochs=250)
            self.assertEqual(len(failures), 1)
            fix = "201,20.0,20.1,36,-75,3000,600,45,2,3,12,2,400,400,0,0.1\n"
            gps.write_text(header + no_fix + fix, encoding="utf-8")
            failures = []
            gate.check_gnss_velocity(gps, failures, 0, max_fixes=0, min_epochs=150)
            self.assertEqual(failures, ["gps_fixes.csv: 1 3D fixes, expected at most 0"])


if __name__ == "__main__":
    unittest.main()
