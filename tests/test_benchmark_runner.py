#!/usr/bin/env python3
"""Exercise diagnostic timing boundaries against the real native runner."""
import json
import subprocess
import sys
import unittest

RUNNER = sys.argv.pop(1)


class BenchmarkRunnerTests(unittest.TestCase):
    def records(self, case):
        result = subprocess.run(
            [RUNNER, "--case", case, "--samples", "1", "--min-sample-ms", "1",
             "--format", "jsonl"], capture_output=True, text=True, check=True)
        return [json.loads(line) for line in result.stdout.splitlines()]

    def test_phase_boundaries_exclude_setup_and_sleep_warmup(self):
        for case in ("constraint_chain", "sleeping_world"):
            with self.subTest(case=case):
                summary = self.records(case)[-1]
                wall = summary["diagnostic_operation_ns"]
                phases = sum(summary["phases_ns"].values())
                self.assertLessEqual(phases, wall)
                self.assertGreater(phases, wall * 0.9)
                self.assertEqual(summary["phase_scope"], "simulation")
                self.assertGreater(summary["psi_latency_ns"]["max"], 0)
                self.assertEqual(summary["psi_latency_ns"]["count"],
                                 sum(w["steps"] for w in summary["windows"]))
                self.assertEqual(summary["residual_bytes"], 0)

    def test_hotspot_mode_keeps_hash_and_rejects_ambiguous_case(self):
        expected = self.records("constraint_chain")[-1]["state_hash"]
        result = subprocess.run(
            [RUNNER, "--case", "constraint_chain", "--hotspot-ms", "1",
             "--format", "jsonl"], capture_output=True, text=True, check=True)
        record = json.loads(result.stdout)
        self.assertEqual(record["kind"], "native-profile")
        self.assertEqual(record["state_hash"], expected)
        bad = subprocess.run([RUNNER, "--hotspot-ms", "1"], capture_output=True)
        self.assertEqual(bad.returncode, 2)

    def test_churn_excludes_validation_from_operation(self):
        sample, summary = self.records("object_churn")
        self.assertGreater(sample["validation_ns"], 0)
        self.assertGreaterEqual(sample["total_ns"], sample["operation_ns"] + sample["validation_ns"])
        self.assertEqual(sample["timing_contract"], "churn-excluding-validation-v2")
        self.assertEqual(summary["psi_latency_ns"]["scope"], "single-world-counter-disabled")


if __name__ == "__main__":
    unittest.main()
