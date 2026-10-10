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
    def test_repository_manifest_is_valid(self):
        data = verification.load_json(
            ROOT / "tools" / "verification_contracts.json"
        )
        test_sets, contracts = verification.validate_manifest(data)
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
            set(contracts),
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
        with self.assertRaises(verification.ContractError):
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
        with self.assertRaises(verification.ContractError):
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
        with self.assertRaises(verification.ContractError):
            verification.check_output_assertions(
                "PASS\n", [{"count": 2, "text": "PASS"}]
            )

    def test_command_contract_writes_passing_evidence(self):
        manifest = {
            "schema_version": 1,
            "test_sets": {"placeholder": ["unused"]},
            "contracts": [
                {
                    "id": "probe",
                    "description": "Probe the command runner.",
                    "type": "command",
                    "oracle": "The child command emits its completion marker.",
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
            evidence_path = directory_path / "evidence.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = verification.main(
                    [
                        "--contract", "probe",
                        "--manifest", str(manifest_path),
                        "--evidence", str(evidence_path),
                    ]
                )
            evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        self.assertEqual(0, result)
        self.assertEqual("passed", evidence["result"])
        self.assertEqual(1, evidence["checks"]["output_assertions"][0]["actual"])

    def test_command_contract_preserves_failure_evidence(self):
        manifest = {
            "schema_version": 1,
            "test_sets": {"placeholder": ["unused"]},
            "contracts": [
                {
                    "id": "probe",
                    "description": "Probe failed output validation.",
                    "type": "command",
                    "oracle": "The child command must emit the required marker.",
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
            evidence_path = directory_path / "evidence.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = verification.main(
                    [
                        "--contract", "probe",
                        "--manifest", str(manifest_path),
                        "--evidence", str(evidence_path),
                    ]
                )
            evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        self.assertEqual(1, result)
        self.assertEqual("failed", evidence["result"])
        self.assertIn("expected 1 occurrence", evidence["error"])


if __name__ == "__main__":
    unittest.main()
