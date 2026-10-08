#!/usr/bin/env python3
import sys
import unittest
import io
import json
from contextlib import redirect_stderr, redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark
import profile_benchmark as profiler


class ProfileTests(unittest.TestCase):
    def test_scenario_scopes_exclude_world_setup(self):
        stacks = [(30, ("main", "ivp_environment_simulate_until", "solver")),
                  (20, ("main", "IVP_Time_Manager::event_loop(IVP_Environment*, IVP_Time)", "solver")),
                  (10, ("main", "run_benchmark_world", "create_environment"))]
        report = profiler.aggregate_stacks(stacks, scope_classifier=profiler.scenario_scope_for)
        self.assertEqual(report["scopes"]["simulation"]["samples"], 50)
        self.assertEqual(report["scopes"]["lifecycle_or_harness"]["samples"], 10)

    def test_scenario_commands_match_fixture_and_counter_contract(self):
        scene = ("cubes", "0.005555555555555556", 480)
        c17 = profiler.scenario_command(Path("/tmp/c17"), scene, "libivp", seconds=5)
        ref = profiler.scenario_command(Path("/tmp/ref"), scene, "reference", seconds=5)
        self.assertEqual(c17[1:-2], ref[1:])
        self.assertEqual(c17[-2:], ["--counter", "noop"])
        self.assertEqual(c17[c17.index("--steps") + 1], "480")
        self.assertEqual(c17[c17.index("--min-sample-ms") + 1], "7000")
        snapshot = profiler.scenario_command(Path("/tmp/c17"), scene, "libivp")
        self.assertNotIn("--benchmark", snapshot)

    def test_scenario_profile_rejects_delegation_and_wrong_fixture(self):
        scene = ("cubes", "0.005555555555555556", 480)
        record = dict(schema=benchmark.SCHEMA, kind="scenario", implementation="reference",
                      scenario="cubes", sample=0, worlds=1, steps_per_world=480,
                      dt=float(scene[1]), simulate_ns=100, total_ns=120)
        profiler.validate_scenario_record(record, scene, "reference")
        for broken in ({"implementation": "libivp"}, {"scenario": "motor"},
                       {"steps_per_world": 479}, {"worlds": 0}, {"dt": 0.01},
                       {"simulate_ns": 0}, {"total_ns": 50}):
            with self.subTest(broken=broken):
                with self.assertRaises(benchmark.BenchmarkError):
                    profiler.validate_scenario_record(dict(record, **broken), scene, "reference")
        c17 = dict(record, implementation="libivp", counter_mode="noop", timing_contract="scenario-noop-v1")
        profiler.validate_scenario_record(c17, scene, "libivp")
        with self.assertRaises(benchmark.BenchmarkError):
            profiler.validate_scenario_record(dict(c17, counter_mode="default"), scene, "libivp")

    def test_reference_snapshots_must_match_stdout_and_stderr(self):
        scene = ("cubes", "0.005555555555555556", 480)
        runners = {"libivp": Path("/tmp/c17"), "reference": Path("/tmp/ref")}
        same = SimpleNamespace(returncode=0, stdout=b'{"state":1}\n', stderr=b'')
        with patch.object(profiler.subprocess, "run", return_value=same):
            result = profiler.verify_scenario_snapshots(runners, scene)
        self.assertTrue(result["reference_byte_identical"])
        for stdout, stderr in ((b'{"state":2}\n', b''), (same.stdout, b'warning')):
            different = SimpleNamespace(returncode=0, stdout=stdout, stderr=stderr)
            with patch.object(profiler.subprocess, "run", side_effect=[same, different]):
                with self.assertRaisesRegex(benchmark.BenchmarkError, "snapshots differ"):
                    profiler.verify_scenario_snapshots(runners, scene)

    def test_single_runner_does_not_claim_reference_parity(self):
        scene = ("motor", "0.005555555555555556", 480)
        output = SimpleNamespace(returncode=0, stdout=b'{"state":1}\n', stderr=b'')
        with patch.object(profiler.subprocess, "run", return_value=output):
            result = profiler.verify_scenario_snapshots({"libivp": Path("/tmp/c17")}, scene)
        self.assertIsNone(result["reference_byte_identical"])
        with patch.object(profiler.subprocess, "run", return_value=SimpleNamespace(
                returncode=0, stdout=b'', stderr=b'')):
            with self.assertRaisesRegex(benchmark.BenchmarkError, "empty snapshot"):
                profiler.verify_scenario_snapshots({"libivp": Path("/tmp/c17")}, scene)

    def test_scenario_capture_checks_record_and_sample_coverage(self):
        scene = ("motor", "0.005555555555555556", 480)
        args = SimpleNamespace(backend="sample", seconds=3, min_samples=10)
        record = dict(schema=benchmark.SCHEMA, kind="scenario", implementation="libivp",
                      scenario="motor", sample=0, worlds=1, steps_per_world=480,
                      dt=float(scene[1]), simulate_ns=100, total_ns=120,
                      counter_mode="noop", timing_contract="scenario-noop-v1")
        stacks = [(9, ("main", "ivp_time_manager_event_loop", "solver")),
                  (90, ("main", "create_environment"))]
        with patch.object(profiler, "capture_command", return_value=(json.dumps(record), stacks)):
            result = profiler.capture_scenario(Path("/tmp/c17"), args, scene, "libivp", Path("/tmp"), {})
        self.assertFalse(result["adequate_samples"])
        self.assertTrue(result["raw"].endswith("motor-libivp.sample.txt"))
        self.assertEqual(result["scopes"]["simulation"]["samples"], 9)
        with redirect_stdout(io.StringIO()):
            profiler.print_report({"cases": [result]}, 2)
        with patch.object(profiler, "capture_command", return_value=(json.dumps(record) + '\n' + json.dumps(record), stacks)):
            with self.assertRaisesRegex(benchmark.BenchmarkError, "expected one scenario record"):
                profiler.capture_scenario(Path("/tmp/c17"), args, scene, "libivp", Path("/tmp"), {})

    def test_invalid_scenario_options_fail_before_build_or_sampling(self):
        for argv in (["--compare-reference"], ["--scenario", "invalid"],
                     ["--scenario", "cubes", "--case", "contact_pile"],
                     ["--scenario", "cubes", "--scale", "10"]):
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    profiler.main(argv)
                self.assertEqual(error.exception.code, 2)

    def test_contact_recalc_counts_distinguish_exact_and_shared_work(self):
        counts = {
            "scope": "per-simulation-time-v1",
            "fingerprint_contract": "contact-input-128-v1",
            "calls": 10,
            "branch_pp": 1, "branch_pk": 2, "branch_pf": 3, "branch_kk": 4,
            "same_time_calls": 2, "same_geometry_fingerprint": 3,
            "same_kinematic_fingerprint": 2, "same_full_input_fingerprint": 1,
            "max_calls_per_contact_at_time": 2,
            "side_visits": 20, "object_pose_first_uses": 7,
            "object_pose_reuses": 13, "core_state_first_uses": 8,
            "core_state_reuses": 12,
            "point_world_uses": 4, "point_world_first_uses": 3,
            "point_world_reuses": 1, "edge_world_uses": 5,
            "edge_world_first_uses": 3, "edge_world_reuses": 2,
            "face_normal_uses": 3, "face_normal_first_uses": 2,
            "face_normal_reuses": 1, "face_tangent_uses": 3,
            "face_tangent_first_uses": 3, "face_tangent_reuses": 0,
            "tracking_overflows": 0,
        }
        profiler.validate_contact_recalc_counts(counts, "contact")
        for broken in ({"calls": 9}, {"same_time_calls": 11},
                       {"same_full_input_fingerprint": 3}, {"side_visits": 19},
                       {"object_pose_reuses": 14}, {"core_state_first_uses": -1},
                       {"face_normal_reuses": 2},
                       {"tracking_overflows": 1}):
            with self.subTest(broken=broken):
                with self.assertRaises(benchmark.BenchmarkError):
                    profiler.validate_contact_recalc_counts(dict(counts, **broken), "contact")

    def test_scenario_contact_counts_require_unchanged_snapshot(self):
        scene = ("cubes", "0.005555555555555556", 480)
        snapshot = b'{"state":1}\n'
        verification = {
            "stdout_sha256": profiler.hashlib.sha256(snapshot).hexdigest(),
            "stderr_sha256": profiler.hashlib.sha256(b"").hexdigest(),
        }
        counts = {
            "scope": "per-simulation-time-v1",
            "fingerprint_contract": "contact-input-128-v1",
            "calls": 1,
            "branch_pp": 0, "branch_pk": 0, "branch_pf": 1, "branch_kk": 0,
            "same_time_calls": 0, "same_geometry_fingerprint": 0,
            "same_kinematic_fingerprint": 0, "same_full_input_fingerprint": 0,
            "max_calls_per_contact_at_time": 1, "side_visits": 2,
            "object_pose_first_uses": 2, "object_pose_reuses": 0,
            "core_state_first_uses": 2, "core_state_reuses": 0,
            "point_world_uses": 1, "point_world_first_uses": 1,
            "point_world_reuses": 0, "edge_world_uses": 0,
            "edge_world_first_uses": 0, "edge_world_reuses": 0,
            "face_normal_uses": 1, "face_normal_first_uses": 1,
            "face_normal_reuses": 0, "face_tangent_uses": 1,
            "face_tangent_first_uses": 1, "face_tangent_reuses": 0,
            "tracking_overflows": 0,
        }
        record = dict(schema=benchmark.SCHEMA, kind="scenario", implementation="libivp",
                      scenario="cubes", sample=0, worlds=1, steps_per_world=480,
                      dt=float(scene[1]), simulate_ns=100, total_ns=120,
                      counter_mode="noop", timing_contract="scenario-noop-v1",
                      contact_recalc_operations=counts)
        completed = SimpleNamespace(returncode=0, stdout=snapshot, stderr=b"")
        with patch.object(profiler.subprocess, "run", return_value=completed), \
             patch.object(profiler.benchmark, "run", return_value=json.dumps(record)):
            self.assertEqual(profiler.scenario_contact_counts(
                Path("/tmp/diagnostic"), scene, verification), counts)
        changed = SimpleNamespace(returncode=0, stdout=b'{"state":2}\n', stderr=b"")
        with patch.object(profiler.subprocess, "run", return_value=changed):
            with self.assertRaisesRegex(benchmark.BenchmarkError, "changed the snapshot"):
                profiler.scenario_contact_counts(Path("/tmp/diagnostic"), scene, verification)

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
