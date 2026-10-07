#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark
import profile_benchmark as profiler


class ProfileTests(unittest.TestCase):
    def test_broadphase_work_counts_are_consistent(self):
        fields = ("rechecks", "recursive_rechecks", "max_depth", "candidates", "old_collisions",
                  "max_candidates", "max_old_collisions", "potential_search_pairs", "searches",
                  "found", "misses", "entries_compared", "max_scan", "swaps")
        empty = dict.fromkeys(fields, 0)
        profiler.validate_broadphase_counts(empty, "empty-world")
        counts = dict(empty, rechecks=2, max_depth=1, candidates=8, old_collisions=6,
                      max_candidates=5, max_old_collisions=4, potential_search_pairs=20,
                      searches=7, found=5, misses=2, entries_compared=10, max_scan=3, swaps=4)
        profiler.validate_broadphase_counts(counts, "test")
        for broken in ({"found": 6}, {"swaps": 6}, {"entries_compared": 4},
                       {"max_scan": 5}, {"recursive_rechecks": 3}, {"searches": 9},
                       {"misses": -1}, {"rechecks": "2"}):
            with self.assertRaises(benchmark.BenchmarkError):
                profiler.validate_broadphase_counts(dict(counts, **broken), "test")
        with self.assertRaises(benchmark.BenchmarkError):
            profiler.validate_broadphase_counts({}, "invalid")

    def test_indexed_matching_has_separate_probe_counts(self):
        counts = dict(rechecks=2, recursive_rechecks=0, max_depth=1, candidates=8, old_collisions=6,
                      max_candidates=5, max_old_collisions=4, potential_search_pairs=20,
                      searches=7, found=5, misses=2, entries_compared=2, max_scan=2, swaps=4,
                      indexed_rechecks=1, indexed_searches=6, indexed_found=4, index_entries=4,
                      index_probes=8, max_index_probes=3, duplicate_fallbacks=0,
                      unsafe_fallbacks=0, oom_fallbacks=0)
        profiler.validate_broadphase_counts(counts, "index")
        for broken in ({"indexed_searches": 8}, {"indexed_found": 6}, {"index_probes": 5},
                       {"max_index_probes": 9}, {"indexed_rechecks": 3}, {"index_entries": 7},
                       {"oom_fallbacks": -1}, {"entries_compared": 0}):
            with self.assertRaises(benchmark.BenchmarkError):
                profiler.validate_broadphase_counts(dict(counts, **broken), "index")
        del counts["index_entries"]
        with self.assertRaises(benchmark.BenchmarkError):
            profiler.validate_broadphase_counts(counts, "partial-index-record")

    def test_finger_counts_are_a_subset_not_an_extra_add_category(self):
        counts = {"adds": 10, "tree_adds": 1, "skip_adds": 8, "fallback_adds": 1,
                  "population_buckets": [10], "finger_attempts": 7, "finger_hits": 6}
        profiler.validate_operation_counts(counts, "test")
        for attempts, hits in ((9, 6), (7, 8), (7, -1)):
            with self.assertRaisesRegex(benchmark.BenchmarkError, "finger"):
                profiler.validate_operation_counts(dict(counts, finger_attempts=attempts, finger_hits=hits), "test")
        del counts["finger_attempts"]
        del counts["finger_hits"]
        profiler.validate_operation_counts(counts, "older-report")

    def test_hull_counts_are_validated(self):
        counts = {
            "manager_checks": 10, "callbacks": 8, "max_callbacks_per_check": 3,
            "polygon_callbacks": 5, "anchor_callbacks": 1,
            "oo_watcher_callbacks": 1, "oo_connector_callbacks": 1,
            "synapse_inserts": 2, "lazy_synapse_inserts": 1,
            "synapse_updates": 7, "lazy_synapse_updates": 0,
            "synapse_removes": 3, "halfspace_extensions": 3,
            "exact_promotions": 1, "phantom_promotions": 0,
            "recursive_events": 1, "oo_events": 1,
            "callback_bucket_upper_bounds": [0, 1, 3, 7, 15, 31, None],
            "callback_buckets": [3, 2, 2, 1, 1, 1, 0],
        }
        profiler.validate_hull_counts(counts, "test")
        for field, value in (("callbacks", 7), ("max_callbacks_per_check", 9),
                             ("halfspace_extensions", 6), ("oo_events", 2),
                             ("callback_buckets", [3, 2, 2, 1, 1, 0, 0])):
            with self.subTest(field=field):
                with self.assertRaises(benchmark.BenchmarkError):
                    profiler.validate_hull_counts(dict(counts, **{field: value}), "test")

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
