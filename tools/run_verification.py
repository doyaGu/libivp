#!/usr/bin/env python3
"""Run one repository verification suite and save its result.

The suite is checked against the configured build, not against CMake source
text.  CTest suites inspect the JSON test inventory, require the selected
set to match the suite exactly, reject disabled or skipped tests, and then
run that exact set.  Command suites verify their exit status and required
output.  Every run writes a JSON result file, including failures.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MANIFEST = ROOT / "tools" / "verification_suites.json"
SUITE_ID = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*\Z")
PLATFORMS = {"Darwin", "Linux", "Windows"}
BOOLEAN_TRUE = {"1", "ON", "TRUE", "YES", "Y"}
BOOLEAN_FALSE = {"0", "OFF", "FALSE", "NO", "N", "", "NOTFOUND"}


class VerificationError(Exception):
    """A verification suite or one of its checks is invalid."""


def without_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise VerificationError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def load_json(path):
    try:
        with path.open(encoding="utf-8") as stream:
            return json.load(stream, object_pairs_hook=without_duplicate_keys)
    except OSError as error:
        raise VerificationError("cannot read {}: {}".format(path, error))
    except json.JSONDecodeError as error:
        raise VerificationError("invalid JSON in {}: {}".format(path, error))


def require_exact_keys(value, expected, label):
    if not isinstance(value, dict):
        raise VerificationError("{} must be an object".format(label))
    missing = expected - set(value)
    extra = set(value) - expected
    if missing:
        raise VerificationError(
            "{} is missing keys: {}".format(label, ", ".join(sorted(missing)))
        )
    if extra:
        raise VerificationError(
            "{} has unknown keys: {}".format(label, ", ".join(sorted(extra)))
        )


def require_text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise VerificationError("{} must be non-empty text".format(label))
    return value


def require_string_list(value, label, allow_empty=True, require_sorted=True,
                        require_unique=True):
    if not isinstance(value, list) or (not allow_empty and not value):
        qualifier = "a list" if allow_empty else "a non-empty list"
        raise VerificationError("{} must be {}".format(label, qualifier))
    if any(not isinstance(item, str) or not item for item in value):
        raise VerificationError("{} contains an empty or non-string value".format(label))
    if require_unique and len(value) != len(set(value)):
        raise VerificationError("{} contains duplicate values".format(label))
    if require_sorted and value != sorted(value):
        raise VerificationError("{} must be sorted".format(label))
    return value


def validate_selection(value, label):
    require_exact_keys(value, {"exclude", "labels", "tests"}, label)
    require_string_list(value["exclude"], label + ".exclude")
    require_string_list(value["labels"], label + ".labels")
    require_string_list(value["tests"], label + ".tests")
    if not value["labels"] and not value["tests"]:
        raise VerificationError("{} must select at least one label or test".format(label))


def validate_scenario_set(value, label):
    if value is None:
        return
    require_exact_keys(value, {"prefix", "source"}, label)
    source = Path(require_text(value["source"], label + ".source"))
    if source.is_absolute() or ".." in source.parts:
        raise VerificationError("{} must stay inside the repository".format(label + ".source"))
    require_text(value["prefix"], label + ".prefix")


def validate_expected(value, label, test_sets):
    require_exact_keys(
        value,
        {"exclude", "platform_tests", "scenario_set", "test_sets", "tests"},
        label,
    )
    exclusions = require_string_list(value["exclude"], label + ".exclude")
    set_names = require_string_list(value["test_sets"], label + ".test_sets")
    tests = require_string_list(value["tests"], label + ".tests")
    unknown_sets = set(set_names) - set(test_sets)
    if unknown_sets:
        raise VerificationError(
            "{} names unknown test sets: {}".format(
                label, ", ".join(sorted(unknown_sets))
            )
        )
    platform_tests = value["platform_tests"]
    if not isinstance(platform_tests, dict):
        raise VerificationError("{}.platform_tests must be an object".format(label))
    unknown_platforms = set(platform_tests) - PLATFORMS
    if unknown_platforms:
        raise VerificationError(
            "{}.platform_tests has unknown platforms: {}".format(
                label, ", ".join(sorted(unknown_platforms))
            )
        )
    for name, names in platform_tests.items():
        require_string_list(names, "{}.platform_tests.{}".format(label, name))
    validate_scenario_set(value["scenario_set"], label + ".scenario_set")

    known = set(tests)
    for set_name in set_names:
        known.update(test_sets[set_name])
    for names in platform_tests.values():
        known.update(names)
    unknown_exclusions = set(exclusions) - known
    if unknown_exclusions:
        raise VerificationError(
            "{} excludes tests it never includes: {}".format(
                label, ", ".join(sorted(unknown_exclusions))
            )
        )


def validate_manifest(data):
    require_exact_keys(data, {"suites", "schema_version", "test_sets"}, "manifest")
    if data["schema_version"] != 1:
        raise VerificationError("manifest.schema_version must be 1")

    test_sets = data["test_sets"]
    if not isinstance(test_sets, dict):
        raise VerificationError("manifest.test_sets must be an object")
    for name, tests in test_sets.items():
        if not SUITE_ID.fullmatch(name):
            raise VerificationError("invalid test set name: {}".format(name))
        require_string_list(tests, "test_sets.{}".format(name), allow_empty=False)

    suites = data["suites"]
    if not isinstance(suites, list) or not suites:
        raise VerificationError("manifest.suites must be a non-empty list")
    result = {}
    common = {"id", "type"}
    ctest_keys = common | {
        "configuration",
        "expected",
        "required_cache",
        "selection",
    }
    command_keys = common | {
        "command",
        "output_assertions",
        "platforms",
        "required_env",
    }
    for index, suite in enumerate(suites):
        label = "suites[{}]".format(index)
        if not isinstance(suite, dict):
            raise VerificationError("{} must be an object".format(label))
        suite_type = suite.get("type")
        expected_keys = ctest_keys if suite_type == "ctest" else command_keys
        if suite_type not in {"ctest", "command"}:
            raise VerificationError("{}.type must be ctest or command".format(label))
        require_exact_keys(suite, expected_keys, label)
        suite_id = require_text(suite["id"], label + ".id")
        if not SUITE_ID.fullmatch(suite_id):
            raise VerificationError("invalid suite id: {}".format(suite_id))
        if suite_id in result:
            raise VerificationError("duplicate suite id: {}".format(suite_id))
        if suite_type == "ctest":
            require_text(suite["configuration"], label + ".configuration")
            cache = suite["required_cache"]
            if not isinstance(cache, dict) or not cache:
                raise VerificationError("{}.required_cache must be a non-empty object".format(label))
            for key, value in cache.items():
                require_text(key, label + ".required_cache key")
                require_text(value, "{}.required_cache.{}".format(label, key))
            validate_selection(suite["selection"], label + ".selection")
            validate_expected(suite["expected"], label + ".expected", test_sets)
        else:
            require_string_list(
                suite["command"], label + ".command", allow_empty=False,
                require_sorted=False, require_unique=False
            )
            require_string_list(suite["required_env"], label + ".required_env")
            platforms = require_string_list(suite["platforms"], label + ".platforms")
            unknown_platforms = set(platforms) - PLATFORMS
            if unknown_platforms:
                raise VerificationError(
                    "{}.platforms has unknown values: {}".format(
                        label, ", ".join(sorted(unknown_platforms))
                    )
                )
            assertions = suite["output_assertions"]
            if not isinstance(assertions, list) or not assertions:
                raise VerificationError("{}.output_assertions must be non-empty".format(label))
            for assertion_index, assertion in enumerate(assertions):
                assertion_label = "{}.output_assertions[{}]".format(label, assertion_index)
                require_exact_keys(assertion, {"count", "text"}, assertion_label)
                require_text(assertion["text"], assertion_label + ".text")
                if not isinstance(assertion["count"], int) or assertion["count"] < 1:
                    raise VerificationError("{}.count must be a positive integer".format(assertion_label))
        result[suite_id] = suite
    return test_sets, result


def parse_cmake_cache(path):
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        raise VerificationError("cannot read {}: {}".format(path, error))
    result = {}
    for line in lines:
        if not line or line.startswith("#") or line.startswith("//") or "=" not in line:
            continue
        left, value = line.split("=", 1)
        if ":" not in left:
            continue
        name, _cache_type = left.rsplit(":", 1)
        result[name] = value
    return result


def normalized_boolean(value):
    upper = value.upper()
    if upper in BOOLEAN_TRUE:
        return True
    if upper in BOOLEAN_FALSE or upper.endswith("-NOTFOUND"):
        return False
    return None


def validate_cache(cache, required):
    checks = []
    errors = []
    for name, expected in required.items():
        actual = cache.get(name)
        matched = False
        if actual is not None:
            expected_bool = normalized_boolean(expected)
            actual_bool = normalized_boolean(actual)
            if expected_bool is not None and actual_bool is not None:
                matched = expected_bool == actual_bool
            else:
                matched = actual == expected
        checks.append({"actual": actual, "expected": expected, "name": name})
        if not matched:
            errors.append("{} expected {!r}, found {!r}".format(name, expected, actual))
    if errors:
        raise VerificationError("CMake cache mismatch: " + "; ".join(errors))
    return checks


def validate_configuration(cache, expected):
    configurations = cache.get("CMAKE_CONFIGURATION_TYPES", "")
    if configurations:
        available = [item for item in configurations.split(";") if item]
        if expected.casefold() not in {item.casefold() for item in available}:
            raise VerificationError(
                "configuration {} is not in CMAKE_CONFIGURATION_TYPES ({})".format(
                    expected, ", ".join(available)
                )
            )
        return {"generator": "multi-config", "selected": expected, "available": available}

    actual = cache.get("CMAKE_BUILD_TYPE")
    if actual is None or actual.casefold() != expected.casefold():
        raise VerificationError(
            "CMAKE_BUILD_TYPE expected {!r}, found {!r}".format(expected, actual)
        )
    return {"generator": "single-config", "selected": actual}


def run_capture(argv, cwd):
    try:
        return subprocess.run(
            argv,
            cwd=str(cwd),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
    except OSError as error:
        raise VerificationError("cannot run {}: {}".format(argv[0], error))


def run_streaming(argv, cwd, env=None):
    try:
        process = subprocess.Popen(
            argv,
            cwd=str(cwd),
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
        )
    except OSError as error:
        raise VerificationError("cannot run {}: {}".format(argv[0], error))
    output = []
    assert process.stdout is not None
    for line in process.stdout:
        output.append(line)
        print(line, end="", flush=True)
    return_code = process.wait()
    return return_code, "".join(output)


def ctest_inventory(ctest, build_dir, configuration):
    command = [ctest, "--test-dir", str(build_dir), "-C", configuration,
               "--show-only=json-v1"]
    completed = run_capture(command, ROOT)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise VerificationError("CTest inventory failed: {}".format(detail))
    try:
        data = json.loads(completed.stdout, object_pairs_hook=without_duplicate_keys)
    except (json.JSONDecodeError, VerificationError) as error:
        raise VerificationError("CTest returned invalid JSON inventory: {}".format(error))
    tests = data.get("tests")
    if not isinstance(tests, list):
        raise VerificationError("CTest inventory has no tests array")
    result = {}
    for test in tests:
        name = test.get("name")
        if not isinstance(name, str) or not name:
            raise VerificationError("CTest inventory contains an unnamed test")
        if name in result:
            raise VerificationError("CTest inventory contains duplicate test {}".format(name))
        properties = {}
        for prop in test.get("properties", []):
            if isinstance(prop, dict) and "name" in prop:
                properties[prop["name"]] = prop.get("value")
        labels = properties.get("LABELS", [])
        if isinstance(labels, str):
            labels = [labels]
        if not isinstance(labels, list):
            raise VerificationError("CTest test {} has invalid LABELS".format(name))
        result[name] = {
            "disabled": bool(properties.get("DISABLED", False)),
            "labels": set(labels),
        }
    return result, command


def read_scenario_names(root, specification):
    source = root / specification["source"]
    try:
        lines = source.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise VerificationError("cannot read {}: {}".format(source, error))
    names = []
    seen = set()
    for line_number, raw_line in enumerate(lines, 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 3:
            raise VerificationError(
                "{}:{}: expected name, dt, and steps".format(
                    specification["source"], line_number
                )
            )
        name = fields[0]
        if name in seen:
            raise VerificationError("duplicate scenario: {}".format(name))
        seen.add(name)
        names.append(specification["prefix"] + name)
    if not names:
        raise VerificationError("{} contains no scenarios".format(specification["source"]))
    return names


def select_tests(inventory, selection):
    labels = set(selection["labels"])
    explicit = set(selection["tests"])
    excluded = set(selection["exclude"])
    selected = {
        name
        for name, details in inventory.items()
        if name in explicit or labels.intersection(details["labels"])
    }
    selected.difference_update(excluded)
    if not selected:
        raise VerificationError("suite selected no configured tests")
    return selected


def expected_tests(expected, test_sets, root, system_name):
    names = set(expected["tests"])
    for set_name in expected["test_sets"]:
        names.update(test_sets[set_name])
    names.update(expected["platform_tests"].get(system_name, []))
    scenario_set = expected["scenario_set"]
    if scenario_set is not None:
        names.update(read_scenario_names(root, scenario_set))
    names.difference_update(expected["exclude"])
    return names


def require_exact_test_set(selected, expected):
    missing = expected - selected
    unexpected = selected - expected
    errors = []
    if missing:
        errors.append("missing: {}".format(", ".join(sorted(missing))))
    if unexpected:
        errors.append("unexpected: {}".format(", ".join(sorted(unexpected))))
    if errors:
        raise VerificationError("selected CTest set differs from suite ({}).".format("; ".join(errors)))


def find_skip_markers(output):
    markers = []
    for line in output.splitlines():
        if "***Skipped" in line or re.search(r"(^|\s)Not Run(\s|$)", line):
            markers.append(line.strip())
    return markers


def check_output_assertions(output, assertions):
    checks = []
    errors = []
    for assertion in assertions:
        actual = output.count(assertion["text"])
        expected = assertion["count"]
        checks.append({"actual": actual, "expected": expected, "text": assertion["text"]})
        if actual != expected:
            errors.append(
                "{!r} expected {} occurrence(s), found {}".format(
                    assertion["text"], expected, actual
                )
            )
    if errors:
        raise VerificationError("command output mismatch: " + "; ".join(errors))
    return checks


def output_summary(output):
    lines = output.splitlines()
    return {
        "line_count": len(lines),
        "sha256": hashlib.sha256(output.encode("utf-8")).hexdigest(),
        "tail": lines[-40:],
    }


def execute_ctest_suite(suite, test_sets, args, run_result):
    build_dir = Path(args.build_dir)
    if not build_dir.is_absolute():
        build_dir = (ROOT / build_dir).resolve()
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        raise VerificationError("configured build not found: {}".format(cache_path))
    cache = parse_cmake_cache(cache_path)
    run_result["checks"]["configuration"] = validate_configuration(
        cache, suite["configuration"]
    )
    run_result["checks"]["cache"] = validate_cache(cache, suite["required_cache"])

    inventory, inventory_command = ctest_inventory(
        args.ctest, build_dir, suite["configuration"]
    )
    run_result["checks"]["inventory_command"] = inventory_command
    run_result["checks"]["inventory_test_count"] = len(inventory)
    selected = select_tests(inventory, suite["selection"])
    expected = expected_tests(
        suite["expected"], test_sets, ROOT, platform.system()
    )
    require_exact_test_set(selected, expected)
    disabled = sorted(name for name in selected if inventory[name]["disabled"])
    if disabled:
        raise VerificationError("suite contains disabled tests: {}".format(", ".join(disabled)))

    selected_names = sorted(selected)
    run_result["checks"]["selected_test_count"] = len(selected_names)
    run_result["checks"]["selected_tests"] = selected_names
    expression = "^({})$".format("|".join(re.escape(name) for name in selected_names))
    command = [
        args.ctest,
        "--test-dir",
        str(build_dir),
        "-C",
        suite["configuration"],
        "-R",
        expression,
        "--no-tests=error",
        "--output-on-failure",
        "--parallel",
        str(args.parallel),
    ]
    run_result["checks"]["command"] = command
    return_code, output = run_streaming(command, ROOT)
    run_result["checks"]["return_code"] = return_code
    run_result["checks"]["output"] = output_summary(output)
    markers = find_skip_markers(output)
    run_result["checks"]["skip_markers"] = markers
    if return_code != 0:
        raise VerificationError("CTest exited with status {}".format(return_code))
    if markers:
        raise VerificationError("CTest skipped or did not run tests: {}".format(" | ".join(markers)))


def execute_command_suite(suite, run_result):
    system_name = platform.system()
    if suite["platforms"] and system_name not in suite["platforms"]:
        raise VerificationError(
            "suite supports {}, current platform is {}".format(
                ", ".join(suite["platforms"]), system_name
            )
        )
    missing_env = [name for name in suite["required_env"] if not os.environ.get(name)]
    if missing_env:
        raise VerificationError(
            "required environment variables are unset: {}".format(", ".join(missing_env))
        )
    command = list(suite["command"])
    first = Path(command[0])
    if not first.is_absolute() and (ROOT / first).exists():
        command[0] = str(ROOT / first)
    run_result["checks"]["command"] = command
    run_result["checks"]["required_env"] = sorted(suite["required_env"])
    return_code, output = run_streaming(command, ROOT, env=os.environ.copy())
    run_result["checks"]["return_code"] = return_code
    run_result["checks"]["output"] = output_summary(output)
    if return_code != 0:
        raise VerificationError("command exited with status {}".format(return_code))
    run_result["checks"]["output_assertions"] = check_output_assertions(
        output, suite["output_assertions"]
    )


def repository_state():
    try:
        commit = run_capture(["git", "rev-parse", "HEAD"], ROOT)
        status = run_capture(["git", "status", "--porcelain"], ROOT)
    except VerificationError:
        return {"commit": None, "dirty": None}
    return {
        "commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "dirty": bool(status.stdout.strip()) if status.returncode == 0 else None,
    }


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_result_file(path, run_result):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(run_result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", required=True, help="suite id to execute")
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--ctest", default="ctest")
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument("--result-file", help="output JSON path")
    args = parser.parse_args(argv)
    if args.parallel < 1:
        parser.error("--parallel must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    result_path = Path(args.result_file) if args.result_file else (
        ROOT / "build" / "verification" / (args.suite + ".json")
    )
    if not result_path.is_absolute():
        result_path = (ROOT / result_path).resolve()
    started = utc_now()
    started_clock = time.monotonic()
    run_result = {
        "checks": {},
        "suite": {"id": args.suite},
        "error": None,
        "finished_at": None,
        "host": {
            "machine": platform.machine(),
            "platform": platform.system(),
            "python": platform.python_version(),
        },
        "repository": repository_state(),
        "result": "failed",
        "schema_version": 3,
        "started_at": started,
    }
    exit_code = 1
    try:
        manifest_path = Path(args.manifest)
        if not manifest_path.is_absolute():
            manifest_path = (ROOT / manifest_path).resolve()
        test_sets, suites = validate_manifest(load_json(manifest_path))
        if args.suite not in suites:
            raise VerificationError(
                "unknown suite {!r}; choose one of {}".format(
                    args.suite, ", ".join(sorted(suites))
                )
            )
        suite = suites[args.suite]
        run_result["suite"] = {"id": suite["id"], "type": suite["type"]}
        if suite["type"] == "ctest":
            execute_ctest_suite(suite, test_sets, args, run_result)
        else:
            execute_command_suite(suite, run_result)
        run_result["result"] = "passed"
        exit_code = 0
    except VerificationError as error:
        run_result["error"] = str(error)
        print("verification suite {}: FAIL: {}".format(args.suite, error), file=sys.stderr)
    except Exception as error:  # Preserve the result for unexpected runner bugs.
        run_result["error"] = "{}: {}".format(type(error).__name__, error)
        print("verification suite {}: ERROR: {}".format(args.suite, error), file=sys.stderr)
    finally:
        run_result["duration_seconds"] = round(time.monotonic() - started_clock, 6)
        run_result["finished_at"] = utc_now()
        try:
            write_result_file(result_path, run_result)
        except OSError as error:
            print("cannot write result {}: {}".format(result_path, error), file=sys.stderr)
            exit_code = 1
    if exit_code == 0:
        print("verification suite {}: PASS".format(args.suite))
    print("result: {}".format(result_path))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
