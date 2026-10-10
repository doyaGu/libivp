#!/usr/bin/env python3
"""Validate and render the repository's feature verification matrix.

The JSON file is the single source of truth.  This tool verifies every named
CTest target, scenario, vocabulary entry, and documentation path before it
renders the human-readable matrix.

    tools/feature_matrix.py          print the generated Markdown
    tools/feature_matrix.py --write  rewrite docs/FEATURE_MATRIX.md
    tools/feature_matrix.py --check  fail if the manifest or Markdown is stale
"""

import argparse
import json
import os
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "tools" / "feature_matrix.json"
OUTPUT = ROOT / "docs" / "FEATURE_MATRIX.md"
SCENARIOS = ROOT / "tools" / "scenarios.txt"

ROOT_KEYS = {
    "schema_version",
    "modules",
    "configurations",
    "properties",
    "features",
}
FEATURE_KEYS = {
    "id",
    "name",
    "summary",
    "modules",
    "properties",
    "tests",
    "scenarios",
    "configurations",
    "docs",
}


def object_without_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def load_manifest():
    with MANIFEST.open(encoding="utf-8") as stream:
        return json.load(stream, object_pairs_hook=object_without_duplicate_keys)


def cmake_test_names():
    names = set()
    pattern = re.compile(
        r"(?:ivp_add_test\s*\(|add_test\s*\(\s*NAME\s+)"
        r"([A-Za-z0-9_]+)(?=\s|\))"
    )
    for path in (ROOT / "CMakeLists.txt", ROOT / "examples" / "CMakeLists.txt"):
        if path.exists():
            text = path.read_text(encoding="utf-8")
            declarations = "\n".join(
                line for line in text.splitlines()
                if not line.lstrip().startswith("#")
            )
            names.update(pattern.findall(declarations))
    return names


def scenario_names(errors):
    names = []
    seen = set()
    for line_number, raw_line in enumerate(
            SCENARIOS.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 3:
            errors.append(
                "{}:{}: expected name, dt, and steps".format(
                    SCENARIOS.relative_to(ROOT), line_number
                )
            )
            continue
        name = fields[0]
        if name in seen:
            errors.append("duplicate scenario: {}".format(name))
            continue
        seen.add(name)
        names.append(name)
    return names


def check_string_map(data, key, errors):
    value = data.get(key)
    if not isinstance(value, dict) or not value:
        errors.append("{} must be a non-empty object".format(key))
        return {}
    if list(value) != sorted(value):
        errors.append("{} keys must be sorted".format(key))
    for name, description in value.items():
        if not isinstance(name, str) or not name:
            errors.append("{} contains an empty or non-string key".format(key))
        if not isinstance(description, str) or not description.strip():
            errors.append("{}.{} must have a description".format(key, name))
    return value


def check_keys(value, expected, label, errors):
    missing = expected - set(value)
    extra = set(value) - expected
    if missing:
        errors.append("{} is missing keys: {}".format(
            label, ", ".join(sorted(missing))))
    if extra:
        errors.append("{} has unknown keys: {}".format(
            label, ", ".join(sorted(extra))))


def check_string_list(feature_id, key, value, errors, allow_empty=False):
    label = "feature {} {}".format(feature_id, key)
    if not isinstance(value, list) or (not value and not allow_empty):
        qualifier = "a list" if allow_empty else "a non-empty list"
        errors.append("{} must be {}".format(label, qualifier))
        return []
    bad = [item for item in value if not isinstance(item, str) or not item]
    if bad:
        errors.append("{} contains empty or non-string values".format(label))
        return []
    if len(value) != len(set(value)):
        errors.append("{} contains duplicates".format(label))
    if value != sorted(value):
        errors.append("{} must be sorted".format(label))
    return value


def check_references(feature_id, label, values, known, errors):
    unknown = set(values) - set(known)
    if unknown:
        errors.append(
            "feature {} names unknown {}: {}".format(
                feature_id, label, ", ".join(sorted(unknown))
            )
        )


def validate(data):
    errors = []
    if not isinstance(data, dict):
        return ["manifest root must be an object"], []
    check_keys(data, ROOT_KEYS, "manifest", errors)
    if data.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    modules = check_string_map(data, "modules", errors)
    configurations = check_string_map(data, "configurations", errors)
    properties = check_string_map(data, "properties", errors)
    known_tests = cmake_test_names()
    scenarios = scenario_names(errors)
    known_scenarios = set(scenarios)

    features = data.get("features")
    if not isinstance(features, list) or not features:
        errors.append("features must be a non-empty list")
        return errors, scenarios

    feature_ids = []
    used_modules = set()
    used_configurations = set()
    used_properties = set()
    for index, feature in enumerate(features):
        if not isinstance(feature, dict):
            errors.append("feature {} must be an object".format(index))
            continue
        feature_id = feature.get("id", "#{}".format(index))
        if not isinstance(feature_id, str) or not re.fullmatch(
                r"[a-z0-9]+(?:-[a-z0-9]+)*", feature_id):
            errors.append("feature {} has an invalid id".format(index))
            feature_id = "#{}".format(index)
        feature_ids.append(feature_id)

        check_keys(feature, FEATURE_KEYS, "feature {}".format(feature_id), errors)
        for key in ("name", "summary"):
            if not isinstance(feature.get(key), str) or not feature[key].strip():
                errors.append("feature {} {} must be text".format(feature_id, key))

        feature_modules = check_string_list(
            feature_id, "modules", feature.get("modules"), errors
        )
        feature_properties = check_string_list(
            feature_id, "properties", feature.get("properties"), errors
        )
        feature_tests = check_string_list(
            feature_id, "tests", feature.get("tests"), errors
        )
        feature_scenarios = check_string_list(
            feature_id, "scenarios", feature.get("scenarios"), errors,
            allow_empty=True,
        )
        feature_configurations = check_string_list(
            feature_id, "configurations", feature.get("configurations"), errors
        )
        feature_docs = check_string_list(
            feature_id, "docs", feature.get("docs"), errors
        )

        check_references(feature_id, "modules", feature_modules, modules, errors)
        check_references(
            feature_id, "properties", feature_properties, properties, errors
        )
        check_references(
            feature_id, "configurations", feature_configurations,
            configurations, errors,
        )
        check_references(
            feature_id, "CTest tests", feature_tests, known_tests, errors
        )

        if "@all" in feature_scenarios and feature_scenarios != ["@all"]:
            errors.append("feature {} must not mix @all with scenario names".format(feature_id))
        check_references(
            feature_id, "scenarios", feature_scenarios,
            known_scenarios | {"@all"}, errors,
        )

        for doc in feature_docs:
            path = Path(doc)
            if path.is_absolute() or ".." in path.parts:
                errors.append("feature {} has unsafe doc path: {}".format(feature_id, doc))
            elif not (ROOT / path).is_file():
                errors.append("feature {} doc does not exist: {}".format(feature_id, doc))

        used_modules.update(feature_modules)
        used_properties.update(feature_properties)
        used_configurations.update(feature_configurations)

    if len(feature_ids) != len(set(feature_ids)):
        errors.append("feature ids must be unique")
    if feature_ids != sorted(feature_ids):
        errors.append("features must be sorted by id")

    for label, registry, used in (
        ("modules", modules, used_modules),
        ("properties", properties, used_properties),
        ("configurations", configurations, used_configurations),
    ):
        unused = set(registry) - used
        if unused:
            errors.append("unused {}: {}".format(label, ", ".join(sorted(unused))))
    return errors, scenarios


def code_list(values):
    return ", ".join("`{}`".format(value) for value in values) if values else "None"


def doc_links(values):
    links = []
    for value in values:
        target = os.path.relpath(ROOT / value, OUTPUT.parent).replace(os.sep, "/")
        links.append("[{}]({})".format(value, target))
    return ", ".join(links)


def render(data, scenarios):
    lines = [
        "# Feature verification matrix",
        "",
        "<!-- generated by tools/feature_matrix.py --write; do not edit -->",
        "",
        "This matrix maps major engine capabilities to the properties, tests,",
        "scenarios, build configurations, and design notes that verify them.",
        "`tools/feature_matrix.json` is the single source of truth. A listed path",
        "means the property has maintained regression coverage; it is not a claim",
        "of formal proof. Lists describe the combined coverage for a feature, not a",
        "Cartesian product in which every test runs in every configuration.",
        "",
        "The checker validates each listed CTest name against the CMake declarations,",
        "each scenario against `tools/scenarios.txt`, every vocabulary reference, and",
        "every documentation path. It also rejects duplicate, unsorted, or stale entries.",
        "",
        "## Verification vocabulary",
        "",
        "### Modules",
        "",
        "| id | responsibility |",
        "|---|---|",
    ]
    for key, description in data["modules"].items():
        lines.append("| `{}` | {} |".format(key, description))

    lines.extend([
        "",
        "### Configurations",
        "",
        "| id | meaning |",
        "|---|---|",
    ])
    for key, description in data["configurations"].items():
        lines.append("| `{}` | {} |".format(key, description))

    lines.extend(["", "### Properties", "", "| id | meaning |", "|---|---|"])
    for key, description in data["properties"].items():
        lines.append("| `{}` | {} |".format(key, description))

    lines.extend(["", "## Feature coverage", ""])
    for feature in data["features"]:
        lines.extend([
            "### {} (`{}`)".format(feature["name"], feature["id"]),
            "",
            feature["summary"],
            "",
            "- Modules: {}".format(code_list(feature["modules"])),
            "- Properties: {}".format(code_list(feature["properties"])),
            "- Tests: {}".format(code_list(feature["tests"])),
        ])
        if feature["scenarios"] == ["@all"]:
            scenario_text = "all {} registered scenarios".format(len(scenarios))
        else:
            scenario_text = code_list(feature["scenarios"])
        lines.extend([
            "- Scenarios: {}".format(scenario_text),
            "- Configurations: {}".format(code_list(feature["configurations"])),
            "- Design notes: {}".format(doc_links(feature["docs"])),
            "",
        ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true", help="rewrite the Markdown matrix")
    mode.add_argument("--check", action="store_true", help="validate and reject stale Markdown")
    args = parser.parse_args()

    try:
        data = load_manifest()
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print("{}: {}".format(MANIFEST.relative_to(ROOT), error), file=sys.stderr)
        return 1

    errors, scenarios = validate(data)
    if errors:
        for error in errors:
            print("feature matrix: {}".format(error), file=sys.stderr)
        return 1

    text = render(data, scenarios)
    if args.write:
        OUTPUT.write_text(text, encoding="utf-8")
        print("wrote {}".format(OUTPUT.relative_to(ROOT)))
    elif args.check:
        current = OUTPUT.read_text(encoding="utf-8") if OUTPUT.exists() else ""
        if current != text:
            print(
                "{} is stale: run tools/feature_matrix.py --write".format(
                    OUTPUT.relative_to(ROOT)
                ),
                file=sys.stderr,
            )
            return 1
        print("{} is valid and up to date".format(OUTPUT.relative_to(ROOT)))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
