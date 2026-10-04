#!/usr/bin/env python3
"""Build, run, and compare libivp performance benchmarks."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import statistics
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

SCHEMA = "libivp-benchmark-v1"
QUICK_SCENARIOS = (
    "cubes",
    "hull_pile",
    "galton",
    "fast_impacts",
    "rope",
    "buoyancy",
    "raycast_car_drive",
    "spawn_remove",
)


class BenchmarkError(RuntimeError):
    pass


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None,
        capture: bool = False) -> str:
    display = " ".join(command)
    if not capture:
        print(f"+ {display}", file=sys.stderr)
    result = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
        check=False,
    )
    if result.returncode:
        detail = (result.stderr or result.stdout or "").strip()
        raise BenchmarkError(f"command failed ({result.returncode}): {display}\n{detail}")
    return result.stdout if capture else ""


def git_info(path: Path) -> dict[str, Any]:
    sha = run(["git", "rev-parse", "HEAD"], cwd=path, capture=True).strip()
    status = run(["git", "status", "--porcelain"], cwd=path, capture=True)
    return {"path": str(path), "sha": sha, "dirty": bool(status.strip())}


def cpu_name() -> str:
    if sys.platform == "darwin":
        result = subprocess.run(
            ["sysctl", "-n", "machdep.cpu.brand_string"], text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
        )
        if result.stdout.strip():
            return result.stdout.strip()
    return platform.processor() or platform.machine()


def read_compiler(cache: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not cache.exists():
        return result
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        for key in ("CMAKE_C_COMPILER", "CMAKE_C_COMPILER_ID", "CMAKE_C_COMPILER_VERSION"):
            if line.startswith(key + ":"):
                result[key.lower()] = line.split("=", 1)[-1]
    return result


def scenarios(path: Path) -> list[tuple[str, str, int]]:
    values: list[tuple[str, str, int]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        name, delta, steps = line.split()
        values.append((name, delta, int(steps)))
    return values


def parse_jsonl(text: str, source: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise BenchmarkError(f"{source}:{number}: invalid JSON: {exc}") from exc
        if record.get("schema") != SCHEMA:
            raise BenchmarkError(f"{source}:{number}: unsupported schema {record.get('schema')!r}")
        records.append(record)
    if not records:
        raise BenchmarkError(f"{source}: produced no benchmark records")
    return records


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int((len(ordered) * probability) + 0.999999) - 1))
    return ordered[index]


def summarize(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str, str, int], list[float]] = defaultdict(list)
    for record in records:
        if record["kind"] == "native":
            key = ("native", record["implementation"], record["case"], int(record["scale"]))
            value = float(record["operation_ns"]) / float(record["operations"])
        elif record["kind"] == "scenario":
            key = ("scenario", record["implementation"], record["scenario"], 0)
            denominator = int(record["worlds"]) * int(record["steps_per_world"])
            value = float(record["simulate_ns"]) / denominator
        else:
            continue
        groups[key].append(value)

    output: list[dict[str, Any]] = []
    for (kind, implementation, name, scale), values in sorted(groups.items()):
        mean = statistics.fmean(values)
        output.append({
            "kind": kind,
            "implementation": implementation,
            "name": name,
            "scale": scale,
            "samples": len(values),
            "median_ns": statistics.median(values),
            "p95_ns": percentile(values, 0.95),
            "min_ns": min(values),
            "max_ns": max(values),
            "coefficient_of_variation": statistics.pstdev(values) / mean if mean else 0.0,
        })
    return output


def comparisons(summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    indexed = {(r["kind"], r["implementation"], r["name"], r["scale"]): r for r in summary}
    output: list[dict[str, Any]] = []
    for item in summary:
        if item["kind"] != "scenario" or item["implementation"] != "libivp":
            continue
        reference = indexed.get(("scenario", "reference", item["name"], 0))
        if not reference:
            continue
        ratio = item["median_ns"] / reference["median_ns"] if reference["median_ns"] else None
        output.append({
            "scenario": item["name"],
            "libivp_median_ns_per_psi": item["median_ns"],
            "reference_median_ns_per_psi": reference["median_ns"],
            "libivp_to_reference_ratio": ratio,
            "delta_percent": (ratio - 1.0) * 100.0 if ratio is not None else None,
        })
    return output


def baseline_comparison(current: dict[str, Any], baseline_path: Path) -> dict[str, Any]:
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    if baseline.get("schema") != SCHEMA:
        raise BenchmarkError(f"baseline has unsupported schema: {baseline.get('schema')!r}")
    old = {
        (r["kind"], r["implementation"], r["name"], int(r["scale"])): r
        for r in baseline.get("summary", [])
    }
    deltas = []
    for item in current["summary"]:
        key = (item["kind"], item["implementation"], item["name"], int(item["scale"]))
        previous = old.get(key)
        if not previous:
            continue
        ratio = item["median_ns"] / previous["median_ns"] if previous["median_ns"] else None
        deltas.append({
            "kind": item["kind"], "implementation": item["implementation"],
            "name": item["name"], "scale": item["scale"],
            "current_median_ns": item["median_ns"],
            "baseline_median_ns": previous["median_ns"],
            "delta_percent": (ratio - 1.0) * 100.0 if ratio is not None else None,
        })
    current_machine = current["metadata"]["machine"]
    baseline_machine = baseline.get("metadata", {}).get("machine")
    return {
        "path": str(baseline_path),
        "machine_match": current_machine == baseline_machine,
        "deltas": deltas,
    }


def print_report(result: dict[str, Any]) -> None:
    print(f"libivp benchmark ({result['profile']})")
    print("\nNative workloads")
    for item in result["summary"]:
        if item["kind"] != "native" or item["implementation"] != "libivp":
            continue
        print(f"  {item['name']:<18} n={item['scale']:<5} "
              f"median={item['median_ns'] / 1000:9.3f} us/op "
              f"p95={item['p95_ns'] / 1000:9.3f} us/op "
              f"cv={item['coefficient_of_variation'] * 100:5.1f}%")
    scenario_rows = [r for r in result["comparisons"]]
    if scenario_rows:
        print("\nScenario comparison")
        for item in scenario_rows:
            print(f"  {item['scenario']:<20} "
                  f"lib={item['libivp_median_ns_per_psi'] / 1000:9.3f} us/PSI "
                  f"ref={item['reference_median_ns_per_psi'] / 1000:9.3f} us/PSI "
                  f"delta={item['delta_percent']:+7.2f}%")
    elif any(r["kind"] == "scenario" for r in result["summary"]):
        print("\nScenario timings (libivp only)")
        for item in result["summary"]:
            if item["kind"] == "scenario":
                print(f"  {item['name']:<20} {item['median_ns'] / 1000:9.3f} us/PSI")
    baseline = result.get("baseline")
    if baseline:
        label = "same machine" if baseline["machine_match"] else "DIFFERENT MACHINE; deltas are informational"
        print(f"\nBaseline comparison ({label})")
        for item in baseline["deltas"]:
            print(f"  {item['name']:<20} n={item['scale']:<5} {item['delta_percent']:+7.2f}%")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard"), default="quick")
    parser.add_argument("--compare-reference", action="store_true")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--case", action="append", dest="cases")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    build_root = root / "build" / "benchmark"
    lib_build = build_root / "lib"
    reference_root = Path(os.environ.get("IVP_REF_SRC", root.parent / "ivp")).resolve()
    reference_build = build_root / "reference"
    output_path = args.output or (build_root / "results.json")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not args.no_build:
        run([
            "cmake", "-B", str(lib_build), "-S", str(root),
            "-DCMAKE_BUILD_TYPE=Release", "-DIVP_BUILD_TESTS=ON",
            "-DIVP_BUILD_BENCHMARKS=ON", "-DIVP_ENABLE_SCENARIO_TESTS=ON",
        ], cwd=root)
        run(["cmake", "--build", str(lib_build), "--target", "ivp_benchmark", "test_scenarios", "-j"], cwd=root)
        if args.compare_reference:
            env = dict(os.environ)
            env["IVP_REF_SRC"] = str(reference_root)
            env["IVP_PARITY_DIR"] = str(reference_build)
            run([str(root / "tools" / "parity" / "build_reference.sh"), "release"], cwd=root, env=env)

    native_runner = lib_build / "ivp_benchmark"
    scenario_runner = lib_build / "test_scenarios"
    reference_runner = reference_build / "ref_release" / "runner" / "legacy_runner"
    for binary in (native_runner, scenario_runner):
        if not binary.exists():
            raise BenchmarkError(f"missing runner: {binary}; omit --no-build to create it")
    if args.compare_reference and not reference_runner.exists():
        raise BenchmarkError(f"missing reference runner: {reference_runner}")

    samples = 5 if args.profile == "quick" else 7
    minimum_ms = 200 if args.profile == "quick" else 250
    native_command = [
        str(native_runner), "--profile", args.profile, "--format", "jsonl",
        "--samples", str(samples), "--min-sample-ms", str(minimum_ms),
    ]
    if args.check:
        native_command.append("--check")
    native_filters = [case for case in (args.cases or []) if case in {
        "active_bodies", "contact_pile", "constraint_chain", "fast_impacts",
        "object_churn", "sleeping_world", "convex_build",
    }]
    records: list[dict[str, Any]] = []
    if native_filters:
        for case in native_filters:
            records.extend(parse_jsonl(run(native_command + ["--case", case], cwd=root, capture=True), case))
    elif not args.cases:
        records.extend(parse_jsonl(run(native_command, cwd=root, capture=True), "ivp_benchmark"))

    all_scenarios = scenarios(root / "tools" / "scenarios.txt")
    chosen_names = set(QUICK_SCENARIOS if args.profile == "quick" else (s[0] for s in all_scenarios))
    if args.cases:
        chosen_names.intersection_update(args.cases)
    chosen = [item for item in all_scenarios if item[0] in chosen_names]
    for name, delta, steps in chosen:
        command = [
            "--scenario", name, "--dt", delta, "--steps", str(steps),
            "--benchmark", "--samples", str(samples), "--warmup", "1",
            "--min-sample-ms", str(minimum_ms),
        ]
        records.extend(parse_jsonl(run([str(scenario_runner), *command], cwd=root, capture=True), f"libivp:{name}"))
        if args.compare_reference:
            records.extend(parse_jsonl(run([str(reference_runner), *command], cwd=root, capture=True), f"reference:{name}"))

    if not records:
        raise BenchmarkError("no native case or scenario matched --case")
    summary = summarize(records)
    metadata: dict[str, Any] = {
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "machine": {
            "system": platform.system(), "release": platform.release(),
            "architecture": platform.machine(), "cpu": cpu_name(),
        },
        "python": platform.python_version(),
        "compiler": read_compiler(lib_build / "CMakeCache.txt"),
        "libivp": git_info(root),
        "build_type": "Release",
        "ivp_real_double": True,
    }
    if args.compare_reference:
        metadata["reference"] = git_info(reference_root)
    result: dict[str, Any] = {
        "schema": SCHEMA,
        "profile": args.profile,
        "metadata": metadata,
        "records": records,
        "summary": summary,
        "comparisons": comparisons(summary),
    }
    if args.baseline:
        result["baseline"] = baseline_comparison(result, args.baseline)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print_report(result)
    print(f"\nRaw result: {output_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BenchmarkError as exc:
        print(f"benchmark: {exc}", file=sys.stderr)
        raise SystemExit(1)
