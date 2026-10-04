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
import math
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
    groups: dict[tuple[str, str, str, int], dict[int, list[float]]] = defaultdict(
        lambda: defaultdict(list)
    )
    native_diagnostics: dict[tuple[str, str, str, int], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        if record["kind"] == "native":
            key = ("native", record["implementation"], record["case"], int(record["scale"]))
            value = float(record["operation_ns"]) / float(record["operations"])
        elif record["kind"] == "scenario":
            key = ("scenario", record["implementation"], record["scenario"], 0)
            denominator = int(record["worlds"]) * int(record["steps_per_world"])
            value = float(record["simulate_ns"]) / denominator
        elif record["kind"] == "native-summary":
            key = ("native", record["implementation"], record["case"], int(record["scale"]))
            native_diagnostics[key].append(record)
            continue
        else:
            continue
        groups[key][int(record.get("run", 0))].append(value)

    output: list[dict[str, Any]] = []
    for (kind, implementation, name, scale), values_by_run in sorted(groups.items()):
        values = [value for run_values in values_by_run.values() for value in run_values]
        run_medians = [statistics.median(run_values) for run_values in values_by_run.values()]
        mean = statistics.fmean(values)
        item = {
            "kind": kind,
            "implementation": implementation,
            "name": name,
            "scale": scale,
            "samples": len(values),
            "runs": len(values_by_run),
            "run_medians_ns": run_medians,
            "median_ns": statistics.median(run_medians),
            "p95_ns": percentile(values, 0.95),
            "min_ns": min(values),
            "max_ns": max(values),
            "coefficient_of_variation": statistics.pstdev(values) / mean if mean else 0.0,
        }
        diagnostics = native_diagnostics.get((kind, implementation, name, scale), [])
        if diagnostics:
            state_hashes = {record.get("state_hash") for record in diagnostics}
            if None in state_hashes or len(state_hashes) != 1:
                raise BenchmarkError(f"native state hash changed across runs: {name}/{scale}")
            item.update({
                "state_hash": next(iter(state_hashes)),
                "peak_bytes": max(int(record["peak_bytes"]) for record in diagnostics),
                "residual_bytes": max(int(record["residual_bytes"]) for record in diagnostics),
            })
        output.append(item)
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


def enforce_baseline(current: dict[str, Any], baseline_path: Path) -> dict[str, Any]:
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    if baseline.get("schema") != SCHEMA:
        raise BenchmarkError(f"baseline has unsupported schema: {baseline.get('schema')!r}")
    if current.get("profile") != baseline.get("profile"):
        raise BenchmarkError("checked baseline profile does not match current profile")
    if int(current.get("runs", 0)) < 3 or int(baseline.get("runs", 0)) < 3:
        raise BenchmarkError("checked baselines require at least three runs")

    current_meta = current["metadata"]
    baseline_meta = baseline.get("metadata", {})
    for field in ("machine", "compiler", "build_type", "ivp_real_double"):
        if current_meta.get(field) != baseline_meta.get(field):
            raise BenchmarkError(f"checked baseline metadata differs: {field}")
    if current_meta.get("libivp", {}).get("dirty") or baseline_meta.get("libivp", {}).get("dirty"):
        raise BenchmarkError("checked baselines require clean libivp worktrees")

    def native_index(result: dict[str, Any]) -> dict[tuple[str, int], dict[str, Any]]:
        return {
            (item["name"], int(item["scale"])): item
            for item in result.get("summary", [])
            if item["kind"] == "native" and item["implementation"] == "libivp"
        }

    now = native_index(current)
    before = native_index(baseline)
    if set(now) != set(before):
        raise BenchmarkError("checked baseline native case/scale set differs")

    speedups: dict[str, float] = {}
    for key, item in now.items():
        previous = before[key]
        if item.get("state_hash") != previous.get("state_hash"):
            raise BenchmarkError(f"state hash differs for {key[0]}/{key[1]}")
        if int(item.get("residual_bytes", -1)) != 0:
            raise BenchmarkError(f"residual allocation bytes for {key[0]}/{key[1]}")
        previous_peak = int(previous.get("peak_bytes", 0))
        if previous_peak and int(item.get("peak_bytes", 0)) > previous_peak * 1.30:
            raise BenchmarkError(f"peak allocation regression exceeds 30% for {key[0]}/{key[1]}")
        speedup = float(previous["median_ns"]) / float(item["median_ns"])
        speedups[key[0]] = speedup
        if speedup < (1.0 / 1.05):
            raise BenchmarkError(f"performance regression exceeds 5% for {key[0]}/{key[1]}")

    if current["profile"] == "large":
        key_cases = ("active_bodies", "contact_pile", "constraint_chain", "fast_impacts")
        for name in key_cases:
            if speedups.get(name, 0.0) < 1.25:
                raise BenchmarkError(f"large target below 1.25x for {name}")
        geometric_mean = math.prod(speedups[name] for name in key_cases) ** (1.0 / len(key_cases))
        if geometric_mean < 2.0:
            raise BenchmarkError(f"large geometric mean target below 2.0x: {geometric_mean:.3f}x")
    else:
        geometric_mean = None

    return {
        "path": str(baseline_path),
        "passed": True,
        "speedups": speedups,
        "large_geometric_mean": geometric_mean,
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
    checked = result.get("checked_baseline")
    if checked:
        print("\nChecked baseline: PASS")
        if checked["large_geometric_mean"] is not None:
            print(f"  large geometric mean: {checked['large_geometric_mean']:.3f}x")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard", "large"), default="quick")
    parser.add_argument("--compare-reference", action="store_true")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--check-baseline", type=Path)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--case", action="append", dest="cases")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    if args.runs <= 0:
        parser.error("--runs must be positive")
    if args.profile == "large" and args.compare_reference:
        parser.error("large is a native-only profile; use standard for reference comparison")

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

    samples = 7 if args.profile == "standard" else 5
    minimum_ms = 200 if args.profile == "quick" else (250 if args.profile == "standard" else 500)
    native_command = [
        str(native_runner), "--profile", args.profile, "--format", "jsonl",
        "--samples", str(samples), "--min-sample-ms", str(minimum_ms),
    ]
    if args.check or args.check_baseline:
        native_command.append("--check")
    native_filters = [case for case in (args.cases or []) if case in {
        "active_bodies", "contact_pile", "constraint_chain", "fast_impacts",
        "object_churn", "sleeping_world", "convex_build",
    }]
    records: list[dict[str, Any]] = []
    for run_index in range(args.runs):
        native_records: list[dict[str, Any]] = []
        if native_filters:
            for case in native_filters:
                native_records.extend(parse_jsonl(
                    run(native_command + ["--case", case], cwd=root, capture=True), case
                ))
        elif not args.cases:
            native_records.extend(parse_jsonl(
                run(native_command, cwd=root, capture=True), "ivp_benchmark"
            ))
        for record in native_records:
            record["run"] = run_index
        records.extend(native_records)

    all_scenarios = scenarios(root / "tools" / "scenarios.txt")
    chosen_names = set(
        () if args.profile == "large"
        else (QUICK_SCENARIOS if args.profile == "quick" else (s[0] for s in all_scenarios))
    )
    if args.cases:
        chosen_names.intersection_update(args.cases)
    chosen = [item for item in all_scenarios if item[0] in chosen_names]
    for run_index in range(args.runs):
        for name, delta, steps in chosen:
            command = [
                "--scenario", name, "--dt", delta, "--steps", str(steps),
                "--benchmark", "--samples", str(samples), "--warmup", "1",
                "--min-sample-ms", str(minimum_ms),
            ]
            scenario_records = parse_jsonl(
                run([str(scenario_runner), *command], cwd=root, capture=True), f"libivp:{name}"
            )
            for record in scenario_records:
                record["run"] = run_index
            records.extend(scenario_records)
            if args.compare_reference:
                reference_records = parse_jsonl(
                    run([str(reference_runner), *command], cwd=root, capture=True), f"reference:{name}"
                )
                for record in reference_records:
                    record["run"] = run_index
                records.extend(reference_records)

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
        "runs": args.runs,
        "metadata": metadata,
        "records": records,
        "summary": summary,
        "comparisons": comparisons(summary),
    }
    if args.baseline:
        result["baseline"] = baseline_comparison(result, args.baseline)
    if args.check_baseline:
        result["checked_baseline"] = enforce_baseline(result, args.check_baseline)
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
