#!/usr/bin/env python3
"""Optional native C++ smoke; not registered in reference-free CTest."""
import subprocess
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark
import benchmark_trace

LIBIVP, REFERENCE = sys.argv[1:3]
del sys.argv[1:3]


class NativeReferenceTests(unittest.TestCase):
    def test_matching_trajectories(self):
        for case, scale, extra in (("fast_impacts", "1", []),
                                   ("contact_pile", "64", []),
                                   ("contact_pile", "64", ["--contact-height", "4"])):
            traces = []
            for binary in (LIBIVP, REFERENCE):
                output = subprocess.check_output([binary, "--case", case, "--scale", scale,
                                                  "--verify", *extra], text=True)
                traces.append(benchmark.parse_jsonl(output, binary))
            self.assertTrue(benchmark_trace.compare(*traces, cross_engine=True)["passed"])

    def test_reference_rejects_partial_numbers_and_invalid_modes(self):
        base = [REFERENCE, "--case", "fast_impacts", "--scale", "1", "--verify"]
        for extra in (["--samples", "1x"], ["--scale", "1x"], ["--min-sample-ms", "nan"],
                      ["--min-sample-ms", "inf"], ["--samples", "0"], ["--contact-height", "4"],
                      ["--unknown", "1"], ["--scale"]):
            self.assertEqual(subprocess.run([*base, *extra], capture_output=True).returncode, 2)


if __name__ == "__main__":
    unittest.main()
