#!/usr/bin/env python3
"""Regression tests for parity comparison expectation handling."""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]
COMPARE = REPO / "tools" / "parity" / "compare.sh"
SCENARIO = "freefall"


def frame(x):
    zero = {"x": 0.0, "y": 0.0, "z": 0.0}
    return {
        "frame": 0,
        "objects": [{
            "pos": {"x": x, "y": 0.0, "z": 0.0},
            "quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "lin_vel": zero,
            "ang_vel": zero,
        }],
    }


class ParityCompareTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.reference = self.root / "reference"
        self.candidate = self.root / "candidate"
        self.reference.mkdir()
        self.candidate.mkdir()
        self.write_run(self.reference, 0.0, 0)
        self.write_run(self.candidate, 0.0, 0)

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def write_run(directory, x, return_code):
        payload = json.dumps(frame(x)) + "\n"
        (directory / f"{SCENARIO}.jsonl").write_text(payload)
        (directory / f"{SCENARIO}.out").write_text(payload)
        (directory / f"{SCENARIO}.err").write_text("")
        (directory / f"{SCENARIO}.rc").write_text(f"{return_code}\n")

    def compare(self, *options):
        environment = os.environ.copy()
        environment["SCENARIOS"] = SCENARIO
        return subprocess.run(
            [str(COMPARE), *options,
             str(self.reference), str(self.candidate)],
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def expectation_file(self, scenario=SCENARIO):
        path = self.root / "expected.txt"
        path.write_text(f"{scenario}\n")
        return path

    def test_identical_runs_pass_without_expectations(self):
        result = self.compare("--text", "--stderr")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("1/1 identical", result.stdout)

    def test_unlisted_difference_fails(self):
        self.write_run(self.candidate, 1.0, 0)
        result = self.compare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("status:UNEXPECTED_DIFF", result.stdout)

    def test_listed_difference_passes(self):
        self.write_run(self.candidate, 1.0, 0)
        result = self.compare(
            "--expect-differences", str(self.expectation_file()))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("status:EXPECTED_DIFF", result.stdout)
        self.assertIn("1/1 expected differences", result.stdout)

    def test_missing_listed_difference_fails(self):
        result = self.compare(
            "--expect-differences", str(self.expectation_file()))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("status:MISSING_EXPECTED_DIFF", result.stdout)

    def test_allow_list_accepts_identical_scenario(self):
        result = self.compare(
            "--allow-differences", str(self.expectation_file()))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("status:OK", result.stdout)

    def test_allow_list_accepts_listed_difference(self):
        self.write_run(self.candidate, 1.0, 0)
        result = self.compare(
            "--allow-differences", str(self.expectation_file()))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("status:ALLOWED_DIFF", result.stdout)

    def test_return_code_difference_is_never_expected(self):
        self.write_run(self.candidate, 1.0, 1)
        result = self.compare(
            "--expect-differences", str(self.expectation_file()))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("status:UNEXPECTED_RC", result.stdout)

    def test_missing_artifact_is_never_expected(self):
        self.write_run(self.candidate, 1.0, 0)
        (self.candidate / f"{SCENARIO}.rc").unlink()
        result = self.compare(
            "--expect-differences", str(self.expectation_file()))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("status:UNEXPECTED_MISSING", result.stdout)

    def test_unknown_expected_scenario_fails(self):
        result = self.compare(
            "--expect-differences", str(self.expectation_file("not_a_scene")))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unknown expected scenario", result.stderr)


if __name__ == "__main__":
    unittest.main()
