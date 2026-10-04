#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark  # noqa: E402


class BenchmarkToolTests(unittest.TestCase):
    def test_summarize_and_compare(self):
        records = [
            {"kind": "scenario", "implementation": "libivp", "scenario": "x",
             "worlds": 2, "steps_per_world": 10, "simulate_ns": 2000},
            {"kind": "scenario", "implementation": "reference", "scenario": "x",
             "worlds": 1, "steps_per_world": 10, "simulate_ns": 1250},
        ]
        summary = benchmark.summarize(records)
        compared = benchmark.comparisons(summary)
        self.assertEqual(len(compared), 1)
        self.assertAlmostEqual(compared[0]["libivp_to_reference_ratio"], 0.8)

    def test_parse_rejects_unknown_schema(self):
        with self.assertRaises(benchmark.BenchmarkError):
            benchmark.parse_jsonl('{"schema":"old"}\n', "fixture")

    def test_baseline_marks_machine_mismatch(self):
        current = {
            "metadata": {"machine": {"cpu": "new"}},
            "summary": [{"kind": "native", "implementation": "libivp",
                         "name": "case", "scale": 1, "median_ns": 110.0}],
        }
        baseline = {
            "schema": benchmark.SCHEMA,
            "metadata": {"machine": {"cpu": "old"}},
            "summary": [{"kind": "native", "implementation": "libivp",
                         "name": "case", "scale": 1, "median_ns": 100.0}],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "baseline.json"
            path.write_text(json.dumps(baseline), encoding="utf-8")
            result = benchmark.baseline_comparison(current, path)
        self.assertFalse(result["machine_match"])
        self.assertAlmostEqual(result["deltas"][0]["delta_percent"], 10.0)

    def test_summarize_uses_median_of_run_medians_and_state_hash(self):
        records = [
            {"kind": "native", "implementation": "libivp", "case": "x", "scale": 4,
             "run": 0, "operation_ns": 10, "operations": 1},
            {"kind": "native", "implementation": "libivp", "case": "x", "scale": 4,
             "run": 0, "operation_ns": 20, "operations": 1},
            {"kind": "native", "implementation": "libivp", "case": "x", "scale": 4,
             "run": 1, "operation_ns": 100, "operations": 1},
            {"kind": "native-summary", "implementation": "libivp", "case": "x", "scale": 4,
             "run": 0, "state_hash": "abcd", "peak_bytes": 10, "residual_bytes": 0},
            {"kind": "native-summary", "implementation": "libivp", "case": "x", "scale": 4,
             "run": 1, "state_hash": "abcd", "peak_bytes": 12, "residual_bytes": 0},
        ]
        item = benchmark.summarize(records)[0]
        self.assertEqual(item["runs"], 2)
        self.assertEqual(item["run_medians_ns"], [15.0, 100.0])
        self.assertEqual(item["median_ns"], 57.5)
        self.assertEqual(item["state_hash"], "abcd")
        self.assertEqual(item["peak_bytes"], 12)

    def test_checked_baseline_rejects_state_change(self):
        metadata = {
            "machine": {"cpu": "same"}, "compiler": {"id": "c"},
            "build_type": "Release", "ivp_real_double": True,
            "libivp": {"dirty": False},
        }
        item = {"kind": "native", "implementation": "libivp", "name": "case",
                "scale": 1, "median_ns": 50.0, "state_hash": "new",
                "peak_bytes": 10, "residual_bytes": 0}
        current = {"schema": benchmark.SCHEMA, "profile": "standard", "runs": 3,
                   "metadata": metadata, "summary": [item]}
        old_item = dict(item, median_ns=100.0, state_hash="old")
        baseline = {"schema": benchmark.SCHEMA, "profile": "standard", "runs": 3,
                    "metadata": metadata, "summary": [old_item]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "baseline.json"
            path.write_text(json.dumps(baseline), encoding="utf-8")
            with self.assertRaisesRegex(benchmark.BenchmarkError, "state hash differs"):
                benchmark.enforce_baseline(current, path)


if __name__ == "__main__":
    unittest.main()
