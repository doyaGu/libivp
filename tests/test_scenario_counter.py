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

    def test_benchmark_reports_counter_contract(self):
        for mode in ("default", "noop", "disabled"):
            result = subprocess.check_output(
                [RUNNER, "--scenario", "freefall", "--benchmark", "--samples", "1",
                 "--warmup", "0", "--min-sample-ms", "1", "--counter", mode], text=True)
            # Debug builds also emit the reference's engine statistics.
            rows = [json.loads(line) for line in result.splitlines() if line.startswith("{")]
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["counter_mode"], mode)
            self.assertEqual(row["timing_contract"], f"scenario-{mode}-v1")

    def test_invalid_counter_is_rejected(self):
        for args in (["--counter"], ["--counter", "invalid"]):
            result = subprocess.run([RUNNER, "--scenario", "freefall", *args], capture_output=True)
            self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    unittest.main()
