#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
