#!/usr/bin/env python3
"""Exercise diagnostic timing boundaries against the real native runner."""
import json
import subprocess
import sys
import unittest

RUNNER = sys.argv.pop(1)
EXPECT_SERIAL_ISLANDS = "--expect-serial-islands" in sys.argv
if EXPECT_SERIAL_ISLANDS:
    sys.argv.remove("--expect-serial-islands")
if "--expect-parallel-islands" in sys.argv:
    sys.argv.remove("--expect-parallel-islands")


class BenchmarkRunnerTests(unittest.TestCase):
    def records(self, case, *args):
        result = subprocess.run(
            [RUNNER, "--case", case, "--samples", "1", "--min-sample-ms", "1",
             "--format", "jsonl", *args], capture_output=True, text=True, check=True)
        return [json.loads(line) for line in result.stdout.splitlines()]

    def test_phase_boundaries_exclude_setup_and_sleep_warmup(self):
        for case in ("constraint_chain", "sleeping_world"):
            with self.subTest(case=case):
                summary = self.records(case)[-1]
                wall = summary["diagnostic_operation_ns"]
                phases = sum(summary["phases_ns"].values())
                self.assertLessEqual(phases, wall)
                # In an entirely idle world the outer loop/clock cost can be
                # comparable to the instrumented engine work. Do not require
                # a fixed percentage of wall time to land inside phase clocks.
                self.assertGreater(phases, 0)
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

    def test_parallel_statistics_exclude_sleep_warmup(self):
        for workers in (1, 4):
            with self.subTest(workers=workers):
                summary = self.records("sleeping_world", "--scale", "1024",
                                       "--workers", str(workers))[-1]
                self.assertEqual(summary["parallel_scope"], "operation-v1")
                self.assertTrue(all(value == 0 for value in summary["parallel"].values()))
                self.assertNotIn("max_workers_used", summary["parallel"])
                if workers > 1 and not EXPECT_SERIAL_ISLANDS:
                    self.assertGreater(summary["parallel_before_operation"]["island_dispatches"], 0)
                if workers == 1:
                    self.assertEqual(summary["parallel_lifetime_max_workers_used"], 0)

    def test_churn_excludes_validation_from_operation(self):
        sample, summary = self.records("object_churn")
        self.assertGreater(sample["validation_ns"], 0)
        self.assertGreaterEqual(sample["total_ns"], sample["operation_ns"] + sample["validation_ns"])
        self.assertEqual(sample["timing_contract"], "churn-excluding-validation-v2")
        self.assertEqual(summary["psi_latency_ns"]["scope"], "single-world-counter-disabled")

    def test_verification_covers_each_psi_and_is_repeatable(self):
        for case in ("fast_impacts", "constraint_chain", "object_churn"):
            with self.subTest(case=case):
                command = [RUNNER, "--case", case, "--scale", "4", "--verify",
                           "--format", "jsonl"]
                first = subprocess.check_output(command, text=True)
                self.assertEqual(first, subprocess.check_output(command, text=True))
                records = [json.loads(line) for line in first.splitlines()][:-1]
                self.assertEqual([r["step"] for r in records], list(range(len(records))))
                self.assertGreater(len({r["trajectory_hash"] for r in records}), 1)
                if case == "fast_impacts":
                    self.assertGreater(records[-1]["event_count"], 0)
                    self.assertNotEqual(records[0]["event_hash"], records[-1]["event_hash"])
                if case == "constraint_chain":
                    self.assertNotEqual(records[0]["constraint_hash"], records[-1]["constraint_hash"])
                sample = subprocess.check_output(
                    [RUNNER, "--case", case, "--scale", "4", "--samples", "1",
                     "--min-sample-ms", "1", "--format", "jsonl"], text=True)
                summary = json.loads(sample.splitlines()[-1])
                self.assertEqual(summary["trajectory_hash"], records[-1]["trajectory_hash"])
                self.assertEqual(summary["verification_steps"], len(records))

    def test_verification_rejects_ambiguous_modes(self):
        for args in ([], ["--case", "convex_build"],
                     ["--case", "fast_impacts", "--check"],
                     ["--case", "fast_impacts", "--hotspot-ms", "1"]):
            self.assertEqual(subprocess.run([RUNNER, "--verify", *args],
                                           capture_output=True).returncode, 2)

    def test_contact_geometry_control_is_explicit_and_verified(self):
        command = [RUNNER, "--case", "contact_pile", "--scale", "16", "--verify"]
        original = json.loads(subprocess.check_output(command, text=True).splitlines()[-1])
        shorter = json.loads(subprocess.check_output(command + ["--contact-height", "4"], text=True).splitlines()[-1])
        self.assertEqual(shorter["contact_height"], 4)
        self.assertNotEqual(original["trajectory_hash"], shorter["trajectory_hash"])
        self.assertEqual(subprocess.run(command + ["--contact-height", "17"], capture_output=True).returncode, 2)

    def test_parallel_island_workload_is_worker_count_invariant(self):
        listed = subprocess.check_output(
            [RUNNER, "--profile", "large", "--list"], text=True).splitlines()
        self.assertIn("constraint_islands", listed)
        self.assertIn("mixed_constraint_islands", listed)
        self.assertIn("heterogeneous_constraint_islands", listed)
        for case, scale in (("constraint_islands", "256"),
                            ("mixed_constraint_islands", "512"),
                            ("heterogeneous_constraint_islands", "2048")):
            with self.subTest(case=case):
                common = [RUNNER, "--profile", "large", "--case", case,
                          "--scale", scale, "--samples", "1",
                          "--min-sample-ms", "1", "--format", "jsonl"]
                serial = json.loads(subprocess.check_output(
                    common + ["--workers", "1"], text=True).splitlines()[-1])
                self.assertEqual(serial["parallel_workers"], 1)
                self.assertEqual(serial["parallel"], {
                    "island_dispatches": 0,
                    "island_tasks": 0,
                    "broadphase_dispatches": 0,
                    "broadphase_tasks": 0,
                    "solver_dispatches": 0,
                    "solver_tasks": 0,
                })
                for workers in (2, 4):
                    parallel = json.loads(subprocess.check_output(
                        common + ["--workers", str(workers)],
                        text=True).splitlines()[-1])
                    self.assertEqual(parallel["parallel_workers"], workers)
                    self.assertEqual(serial["state_hash"],
                                     parallel["state_hash"])
                    self.assertEqual(serial["trajectory_hash"],
                                     parallel["trajectory_hash"])
                    if EXPECT_SERIAL_ISLANDS:
                        self.assertEqual(parallel["parallel"]["island_dispatches"], 0)
                        self.assertEqual(parallel["parallel"]["island_tasks"], 0)
                    else:
                        self.assertGreater(parallel["parallel"]["island_dispatches"], 0)
                        self.assertGreater(parallel["parallel"]["island_tasks"], 0)
                        self.assertEqual(parallel["parallel_lifetime_max_workers_used"],
                                         workers)


if __name__ == "__main__":
    unittest.main()
