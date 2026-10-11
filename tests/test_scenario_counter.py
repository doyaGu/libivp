#!/usr/bin/env python3
"""Explicit scenario counters must not change any emitted physics output."""
import json
from pathlib import Path
import subprocess
import sys
import unittest

RUNNER = sys.argv.pop(1)
SCENARIOS = Path(sys.argv.pop(1))


class ScenarioCounterTests(unittest.TestCase):
    def diagnostic_record(self, scenario):
        result = subprocess.check_output(
            [RUNNER, "--scenario", scenario, "--dt", "0.005555555555555556",
             "--steps", "480", "--benchmark", "--samples", "1", "--warmup", "0",
             "--min-sample-ms", "0.001", "--counter", "noop"], text=True)
        return json.loads(result.splitlines()[-1])

    def test_all_scenario_outputs_are_counter_independent(self):
        for line in SCENARIOS.read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            name, dt, steps = line.split()
            with self.subTest(scenario=name):
                command = [RUNNER, "--scenario", name, "--dt", dt,
                           "--steps", steps, "--assert-continue"]
                expected = subprocess.run(command, capture_output=True, check=True)
                for mode in ("default", "noop", "disabled"):
                    actual = subprocess.run(command + ["--counter", mode],
                                            capture_output=True, check=True)
                    self.assertEqual((actual.stdout, actual.stderr),
                                     (expected.stdout, expected.stderr), mode)

    def test_benchmark_emits_counter_method(self):
        for mode in ("default", "noop", "disabled"):
            result = subprocess.check_output(
                [RUNNER, "--scenario", "freefall", "--benchmark", "--samples", "1",
                 "--warmup", "0", "--min-sample-ms", "1", "--counter", mode], text=True)
            # Debug builds also emit the reference's engine statistics.
            rows = [json.loads(line) for line in result.splitlines() if line.startswith("{")]
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["counter_mode"], mode)
            self.assertEqual(row["timing_method"], f"scenario-{mode}-v1")

    def test_invalid_counter_is_rejected(self):
        for args in (["--counter"], ["--counter", "invalid"]):
            result = subprocess.run([RUNNER, "--scenario", "freefall", *args], capture_output=True)
            self.assertEqual(result.returncode, 2)

    def test_contact_recalc_diagnostics_are_deterministic_and_consistent(self):
        first = self.diagnostic_record("cubes")
        if "contact_recalc_operations" not in first:
            self.skipTest("intrusive benchmark diagnostics are disabled")
        second = self.diagnostic_record("cubes")
        self.assertEqual(first["contact_recalc_operations"],
                         second["contact_recalc_operations"])
        for scenario in ("cubes", "motor", "check_distance"):
            counts = (first if scenario == "cubes" else
                      self.diagnostic_record(scenario))["contact_recalc_operations"]
            self.assertEqual(counts["calls"], sum(counts[f"branch_{name}"]
                                                  for name in ("pp", "pk", "pf", "kk")))
            self.assertEqual(counts["side_visits"], counts["calls"] * 2)
            self.assertLessEqual(counts["same_time_calls"], counts["calls"])
            self.assertLessEqual(counts["same_full_input_fingerprint"],
                                 counts["same_kinematic_fingerprint"])
            self.assertLessEqual(counts["same_kinematic_fingerprint"],
                                 counts["same_geometry_fingerprint"])
            self.assertGreaterEqual(counts["max_calls_per_contact_at_time"], 1)
            self.assertLessEqual(counts["max_calls_per_contact_at_time"],
                                 counts["same_time_calls"] + 1)
            self.assertEqual(counts["tracking_overflows"], 0)
            for prefix in ("object_pose", "core_state", "point_world",
                           "edge_world", "face_normal", "face_tangent"):
                total = counts["side_visits"] if prefix in ("object_pose", "core_state") \
                    else counts[f"{prefix}_uses"]
                self.assertEqual(total, counts[f"{prefix}_first_uses"] +
                                 counts[f"{prefix}_reuses"])
            self.assertEqual(counts["face_normal_uses"], counts["branch_pf"])
            self.assertEqual(counts["face_tangent_uses"], counts["branch_pf"])
            self.assertGreater(counts["face_normal_reuses"], 0)


if __name__ == "__main__":
    unittest.main()
