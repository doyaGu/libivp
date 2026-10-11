#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import benchmark  # noqa: E402
import benchmark_trace  # noqa: E402
import native_benchmark  # noqa: E402
import build_native_probes  # noqa: E402


class BenchmarkToolTests(unittest.TestCase):
    def test_probe_rejects_missing_or_duplicate_source_anchor(self):
        pattern = r"void run\(void\)"
        self.assertIn("count();", build_native_probes.inject("void run(void) {}", pattern, "count();"))
        for source in ("", "void run(void) {} void run(void) {}"):
            with self.assertRaises(benchmark.BenchmarkError):
                build_native_probes.inject(source, pattern, "count();")

    def test_native_comparison_rejects_instrumented_measurement_build(self):
        config = dict(cmake_build_type="Release", ivp_real_double="ON")
        native_benchmark.measurement_build(config)
        for changed in (dict(config, cmake_build_type="Debug"), dict(config, ivp_real_double="OFF"),
                        dict(config, ivp_benchmark_diagnostics="ON"), dict(config, ivp_enable_asan="ON")):
            with self.assertRaises(benchmark.BenchmarkError): native_benchmark.measurement_build(changed)

    def test_native_comparison_requires_complete_compatible_samples(self):
        row = dict(kind="native", implementation="reference", case="fast_impacts", scale=4,
                   sample=0, timing_method="native-noop-v1", iterations=2, operations=360,
                   operation_ns=100, total_ns=120)
        native_benchmark.validate_samples([row], "reference", "fast_impacts", 4, 1)
        for invalid in ([row, row], [], [dict(row, operations=359)],
                        [dict(row, timing_method="native-v1")], [dict(row, total_ns=10)]):
            with self.assertRaises(benchmark.BenchmarkError):
                native_benchmark.validate_samples(invalid, "reference", "fast_impacts", 4, 1)

    def test_incremental_gate_does_not_reset_overall_target(self):
        metadata = {"machine": {"cpu": "same"}, "compiler": {"id": "c"},
                    "build_type": "Release", "ivp_real_double": True, "libivp": {"dirty": False}}
        row = dict(kind="native", implementation="libivp", name="fast_impacts", scale=512,
                   median_ns=100, state_hash="a" * 16, peak_bytes=10, residual_bytes=0,
                   trajectory_schema=benchmark_trace.TRACE_SCHEMA, verification_steps=181,
                   trajectory_hash="b" * 16, contact_height=0)
        current = dict(schema=benchmark.SCHEMA, profile="large", runs=3, metadata=metadata, summary=[row])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "baseline.json"
            path.write_text(json.dumps(current), encoding="utf-8")
            self.assertTrue(benchmark.enforce_baseline(current, path, require_targets=False)["passed"])
            with self.assertRaisesRegex(benchmark.BenchmarkError, "large target"):
                benchmark.enforce_baseline(current, path)
            for field, value, message in (("trajectory_hash", "c" * 16, "verification differs"),
                                          ("contact_height", 4, "verification differs"),
                                          ("median_ns", 106, "performance regression"),
                                          ("residual_bytes", 1, "residual allocation"),
                                          ("peak_bytes", 14, "peak allocation")):
                changed = dict(current, summary=[dict(row, **{field: value})])
                with self.assertRaisesRegex(benchmark.BenchmarkError, message):
                    benchmark.enforce_baseline(changed, path, require_targets=False)

    def test_trace_catches_intermediate_state_and_event_order_changes(self):
        import copy
        step = dict(schema=benchmark.SCHEMA, kind="native-step", implementation="libivp",
                    case="fast_impacts", scale=1, objects=2, event_count=0,
                    trajectory_schema=benchmark_trace.TRACE_SCHEMA)
        step.update({field: "0" * 16 for field in benchmark_trace.HASH_FIELDS})
        records = [dict(step, step=i) for i in range(3)]
        records.append(dict(kind="native-verification", implementation="libivp",
                            case="fast_impacts", scale=1, steps_per_world=2, dt=0.01,
                            trajectory_schema=benchmark_trace.TRACE_SCHEMA,
                            trajectory_hash="0" * 16))
        self.assertTrue(benchmark_trace.compare(records, records)["passed"])
        for field in ("object_hash", "constraint_hash", "event_hash", "statistics_hash"):
            changed = copy.deepcopy(records)
            changed[1][field] = "1" * 16
            difference = benchmark_trace.compare(records, changed)
            self.assertEqual(difference["step"], 1)
            self.assertEqual(difference["field"], field)
            self.assertFalse(difference["passed"])
        for broken in (records[:-1], records[:1] + records[2:], [records[1], records[0], *records[2:]]):
            with self.assertRaises(benchmark.BenchmarkError): benchmark_trace.validate(broken)

    def test_failed_absolute_gate_preserves_parseable_result(self):
        record = {"schema": benchmark.SCHEMA, "kind": "native-summary", "gate_ok": False}
        command = [sys.executable, "-c", f"print({json.dumps(record)!r}); raise SystemExit(1)"]
        records = benchmark.run_native(command, cwd=Path.cwd(), source="fixture")
        self.assertEqual(records, [record])
        with self.assertRaises(benchmark.BenchmarkError):
            benchmark.run_native([sys.executable, "-c", "raise SystemExit(1)"],
                                 cwd=Path.cwd(), source="fixture")
    def test_diagnostics_reject_setup_time_and_report_scale_growth(self):
        records = [{"kind": "native", "implementation": "libivp", "case": "x",
                    "scale": n, "operation_ns": t, "operations": 1, "total_ns": t * 2}
                   for n, t in ((4, 10), (16, 160))]
        summary = benchmark.summarize(records)
        self.assertEqual(summary[0]["outside_operation_percent"], 50)
        self.assertEqual(benchmark.scaling_analysis(summary)[0]["exponent"], 2)
        records.append({"kind": "native-summary", "implementation": "libivp",
                        "case": "x", "scale": 4, "state_hash": "abcd", "peak_bytes": 0,
                        "residual_bytes": 0, "phase_scope": "simulation",
                        "diagnostic_operation_ns": 10, "phases_ns": {"start": 100}})
        with self.assertRaisesRegex(benchmark.BenchmarkError, "phases exceed"):
            benchmark.summarize(records)

    def test_baseline_rejects_changed_churn_timing_method(self):
        item = {"kind": "native", "implementation": "libivp", "name": "object_churn",
                "scale": 1, "median_ns": 10}
        old = {"schema": benchmark.SCHEMA, "summary": [item]}
        new = {"summary": [dict(item, timing_method="churn-excluding-validation-v2")]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            with self.assertRaisesRegex(benchmark.BenchmarkError, "timing method differs"):
                benchmark.baseline_comparison(new, path)

    def test_baseline_rejects_changed_parallel_worker_count(self):
        item = {"kind": "native", "implementation": "libivp", "name": "active_bodies",
                "scale": 4096, "median_ns": 10, "parallel_workers": 1}
        old = {"schema": benchmark.SCHEMA, "summary": [item]}
        new = {"summary": [dict(item, parallel_workers=4)]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            with self.assertRaisesRegex(benchmark.BenchmarkError, "worker count differs"):
                benchmark.baseline_comparison(new, path)

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

    def test_reference_comparison_rejects_mixed_counters(self):
        common = dict(kind="scenario", name="freefall", scale=0, median_ns=100)
        for other in ("scenario-default-v1", "scenario-disabled-v1"):
            with self.assertRaisesRegex(benchmark.BenchmarkError, "reference timing method"):
                benchmark.comparisons([
                    dict(common, implementation="libivp", timing_method="scenario-noop-v1"),
                    dict(common, implementation="reference", timing_method=other),
                ])

    def test_compiler_metadata_reads_generated_cmake_file_and_actual_options(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / "CMakeCache.txt"
            cache.write_text("CMAKE_C_COMPILER:FILEPATH=/bin/cc\nCMAKE_BUILD_TYPE:STRING=Debug\n"
                             "IVP_REAL_DOUBLE:BOOL=OFF\nIVP_BENCHMARK_DIAGNOSTICS:BOOL=ON\n")
            generated = root / "CMakeFiles" / "3.20" / "CMakeCCompiler.cmake"
            generated.parent.mkdir(parents=True)
            generated.write_text('set(CMAKE_C_COMPILER_ID "Clang")\nset(CMAKE_C_COMPILER_VERSION "21.0")\n')
            flags = root / "CMakeFiles" / "ivp.dir" / "flags.make"
            flags.parent.mkdir(parents=True)
            flags.write_text("C_FLAGS = -O3 -ffp-contract=off\nC_DEFINES = -DIVP_REAL_DOUBLE\n")
            result = benchmark.read_compiler(cache)
        self.assertEqual(result["cmake_c_compiler_version"], "21.0")
        self.assertEqual(result["cmake_build_type"], "Debug")
        self.assertEqual(result["ivp_real_double"], "OFF")
        self.assertEqual(result["ivp_benchmark_diagnostics"], "ON")
        self.assertEqual(result["ivp_c_flags"], "-O3 -ffp-contract=off")

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
