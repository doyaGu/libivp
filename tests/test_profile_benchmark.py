#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark
import profile_benchmark as profiler


class ProfileTests(unittest.TestCase):
    def test_sample_recursion_scope_and_exclusive_accounting(self):
        text = """Call graph:
    100 Thread_1 DispatchQueue_1: com.apple.main-thread (serial)
      100 start  (in dyld) + 1
        100 run_once  (in ivp_benchmark) + 1
          + 60 bench_simulate  (in ivp_benchmark) + 1
          + ! 40 solver  (in ivp_benchmark) + 1
          + ! : 20 solver  (in ivp_benchmark) + 2
          + ! : | 10 leaf  (in ivp_benchmark) + 1
          + ! 20 integrator  (in ivp_benchmark) + 1
          + 30 bench_destroy  (in ivp_benchmark) + 1
          + ! 30 free  (in system) + 1
          + 10 hash_environment  (in ivp_benchmark) + 1
          + ! 10 memcpy  (in system) + 1
    90 Thread_2 other
      90 idle  (in system) + 1
Total number in stack (recursive counted multiple):
  999 ignored
"""
        report = profiler.aggregate_stacks(profiler.sample_stacks(text))
        self.assertEqual(report["samples"], 100)
        sim = report["scopes"]["simulation"]
        self.assertEqual(sim["samples"], 60)
        solver = next(f for f in sim["functions"] if f["function"] == "solver")
        self.assertEqual(solver["inclusive_samples"], 40)
        self.assertEqual(solver["self_samples"], 30)
        self.assertEqual(solver["max_speedup_if_self_eliminated"], 2.0)
        self.assertEqual(sum(s["samples"] for s in sim["stacks"]), 60)
        self.assertEqual(report["scopes"]["teardown"]["samples"], 30)

    def test_perf_leaf_first_and_unknown_symbols(self):
        text = """ivp_benchmark 123 1.0: cycles:
        abc leaf+0x4 (/path/ivp_benchmark)
        def bench_simulate (/path/ivp_benchmark)
        123 run_once (/path/ivp_benchmark)

ivp_benchmark 123 1.1: cycles:
        abc [unknown] ([unknown])
        def bench_simulate (/path/ivp_benchmark)
"""
        report = profiler.aggregate_stacks(profiler.perf_stacks(text))
        self.assertEqual(report["scopes"]["simulation"]["unresolved_leaf_samples"], 1)
        self.assertEqual(report["samples"], 2)

    def test_invalid_profiles_fail_instead_of_showing_empty_hotspots(self):
        for parse in (profiler.sample_stacks, profiler.perf_stacks):
            with self.assertRaises(benchmark.BenchmarkError):
                parse("not a profiler report")
        with self.assertRaisesRegex(benchmark.BenchmarkError, "inconsistent"):
            profiler.sample_stacks("Call graph:\n    1 Thread_1 com.apple.main-thread\n      2 foo  (in lib)\n")

    def test_source_locations_and_sleep_settling_are_separate(self):
        text = """Call graph:
    3 Thread_1 com.apple.main-thread
      3 run_once  (in bench) + 1
        3 bench_sleep_warmup  (in bench) + 1
          3 leaf  (in bench) + 1 [0x10] /path/to/source.c:42
"""
        report = profiler.aggregate_stacks(profiler.sample_stacks(text, locations=True))
        item = report["scopes"]["settling"]["functions"][0]
        self.assertEqual(item["self_locations"], [{"source": "/path/to/source.c:42", "samples": 3}])


if __name__ == "__main__":
    unittest.main()
