#!/usr/bin/env python3
"""Unit and integration tests for tools/run_verification.py."""

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import platform
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "run_verification.py"
SPEC = importlib.util.spec_from_file_location("run_verification", SCRIPT)
verification = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verification)


class VerificationRunnerTests(unittest.TestCase):
    def test_repository_suite_manifest_is_valid(self):
        data = verification.load_json(
            ROOT / "tools" / "verification_suites.json"
        )
        test_sets, suites = verification.validate_manifest(data)
        self.assertIn("unit-common", test_sets)
        self.assertEqual(
            {
                "asan-ubsan-unit",
                "debug-float-unit",
                "examples-headless",
                "parity",
                "release-unit",
                "scenario-invariants",
                "tsan-threading",
            },
            set(suites),
        )

    def test_cache_parser_and_boolean_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            cache_path = Path(directory) / "CMakeCache.txt"
            cache_path.write_text(
                "// comment\n"
                "CMAKE_BUILD_TYPE:STRING=Debug\n"
                "IVP_ENABLE_ASAN:BOOL=TRUE\n"
                "IVP_QHULL_MODE:STRING=REENTRANT\n",
                encoding="utf-8",
            )
            cache = verification.parse_cmake_cache(cache_path)
        checks = verification.validate_cache(
            cache,
            {"IVP_ENABLE_ASAN": "ON", "IVP_QHULL_MODE": "REENTRANT"},
        )
        self.assertEqual(2, len(checks))
        self.assertEqual(
            "single-config",
            verification.validate_configuration(cache, "Debug")["generator"],
        )

    def test_multi_config_validation_requires_available_configuration(self):
        cache = {"CMAKE_CONFIGURATION_TYPES": "Debug;Release"}
        result = verification.validate_configuration(cache, "Release")
        self.assertEqual("multi-config", result["generator"])
        with self.assertRaises(verification.VerificationError):
            verification.validate_configuration(cache, "RelWithDebInfo")

    def test_label_selection_must_equal_expected_inventory(self):
        inventory = {
            "test_alpha": {"disabled": False, "labels": {"unit"}},
            "test_beta": {"disabled": False, "labels": {"threading", "unit"}},
            "test_gamma": {"disabled": False, "labels": {"perf"}},
        }
        selection = {"exclude": [], "labels": ["unit"], "tests": []}
        selected = verification.select_tests(inventory, selection)
        expected_spec = {
            "exclude": [],
            "platform_tests": {},
            "scenario_set": None,
            "test_sets": ["sample"],
            "tests": [],
        }
        expected = verification.expected_tests(
            expected_spec, {"sample": ["test_alpha", "test_beta"]}, ROOT,
            platform.system()
        )
        verification.require_exact_test_set(selected, expected)
        with self.assertRaises(verification.VerificationError):
            verification.require_exact_test_set(selected, {"test_alpha"})

    def test_scenario_set_uses_every_non_comment_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "scenarios.txt"
            source.write_text(
                "# scenario list\nfreefall 0.1 10\n\nrope 0.01 20\n",
                encoding="utf-8",
            )
            names = verification.read_scenario_names(
                root, {"source": "scenarios.txt", "prefix": "check_"}
            )
        self.assertEqual(["check_freefall", "check_rope"], names)

    def test_runtime_skip_markers_are_rejected(self):
        output = "1/2 test_one Passed\n2/2 test_two ***Skipped\n"
        self.assertEqual(
            ["2/2 test_two ***Skipped"],
            verification.find_skip_markers(output),
        )

    def test_output_assertions_require_exact_counts(self):
        checks = verification.check_output_assertions(
            "PASS\nPASS\nDONE\n",
            [{"count": 2, "text": "PASS"}, {"count": 1, "text": "DONE"}],
        )
        self.assertEqual(2, checks[0]["actual"])
        with self.assertRaises(verification.VerificationError):
            verification.check_output_assertions(
                "PASS\n", [{"count": 2, "text": "PASS"}]
            )

    def test_command_suite_writes_passing_result(self):
        manifest = {
            "schema_version": 1,
            "test_sets": {"placeholder": ["unused"]},
            "suites": [
                {
                    "id": "probe",
                    "description": "Probe the command runner.",
                    "type": "command",
                    "expected_result": "The child command emits its completion marker.",
                    "command": [sys.executable, "-c", "print('PROBE OK')"],
                    "output_assertions": [{"count": 1, "text": "PROBE OK"}],
                    "platforms": [platform.system()],
                    "required_env": [],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            manifest_path = directory_path / "manifest.json"
            result_path = directory_path / "result.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = verification.main(
                    [
                        "--suite", "probe",
                        "--manifest", str(manifest_path),
                        "--result-file", str(result_path),
                    ]
                )
            run_result = json.loads(result_path.read_text(encoding="utf-8"))
        self.assertEqual(0, result)
        self.assertEqual(2, run_result["schema_version"])
        self.assertEqual("passed", run_result["result"])
        self.assertEqual("probe", run_result["suite"]["id"])
        self.assertEqual(1, run_result["checks"]["output_assertions"][0]["actual"])

    def test_command_suite_preserves_failure_result(self):
        manifest = {
            "schema_version": 1,
            "test_sets": {"placeholder": ["unused"]},
            "suites": [
                {
                    "id": "probe",
                    "description": "Probe failed output validation.",
                    "type": "command",
                    "expected_result": "The child command must emit the required marker.",
                    "command": [sys.executable, "-c", "print('WRONG')"],
                    "output_assertions": [{"count": 1, "text": "EXPECTED"}],
                    "platforms": [platform.system()],
                    "required_env": [],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            manifest_path = directory_path / "manifest.json"
            result_path = directory_path / "result.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = verification.main(
                    [
                        "--suite", "probe",
                        "--manifest", str(manifest_path),
                        "--result-file", str(result_path),
                    ]
                )
            run_result = json.loads(result_path.read_text(encoding="utf-8"))
        self.assertEqual(1, result)
        self.assertEqual(2, run_result["schema_version"])
        self.assertEqual("failed", run_result["result"])
        self.assertEqual("probe", run_result["suite"]["id"])
        self.assertIn("expected 1 occurrence", run_result["error"])


if __name__ == "__main__":
    unittest.main()
