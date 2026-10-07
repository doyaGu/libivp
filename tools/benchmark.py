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
import re
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


def run_native(command: list[str], *, cwd: Path, source: str) -> list[dict[str, Any]]:
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True)
    if result.returncode not in (0, 1) or not result.stdout.strip():
        raise BenchmarkError(f"{source}: runner failed ({result.returncode}): {result.stderr.strip()}")
    records = parse_jsonl(result.stdout, source)
    if result.returncode and not any(r.get("kind") == "native-summary" and
                                     r.get("gate_ok") is False for r in records):
        raise BenchmarkError(f"{source}: runner failed without a reported absolute gate")
    return records


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
        return "unknown"
    if sys.platform.startswith("linux"):
        cpuinfo = Path("/proc/cpuinfo")
        if cpuinfo.exists():
            for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
                if line.startswith(("model name", "Hardware")):
                    return line.split(":", 1)[-1].strip()
    return platform.processor() or platform.machine()


def read_compiler(cache: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not cache.exists():
        return result
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        for key in ("CMAKE_C_COMPILER", "CMAKE_C_COMPILER_ID", "CMAKE_C_COMPILER_VERSION"):
            if line.startswith(key + ":"):
                result[key.lower()] = line.split("=", 1)[-1]
        if line.startswith(("CMAKE_BUILD_TYPE:", "CMAKE_C_FLAGS:", "CMAKE_C_FLAGS_RELEASE:",
                            "IVP_REAL_DOUBLE:", "IVP_ENABLE_ASSERTS:", "IVP_ENABLE_STATS:",
                            "IVP_CONTAINER_DEBUG:", "IVP_ENABLE_ASAN:", "IVP_ENABLE_UBSAN:")):
            result[line.split(":", 1)[0].lower()] = line.split("=", 1)[-1]
        if line.startswith("IVP_BENCHMARK_DIAGNOSTICS:"):
            result["ivp_benchmark_diagnostics"] = line.split("=", 1)[-1]
    # Compiler id/version are normally in this file, not CMakeCache.txt.
    for path in sorted((cache.parent / "CMakeFiles").glob("*/CMakeCCompiler.cmake")):
        for key, value in re.findall(
                r'set\((CMAKE_C_COMPILER_(?:ID|VERSION)) "([^"]+)"\)',
                path.read_text(encoding="utf-8")):
            result[key.lower()] = value
    for target in ("ivp", "ivp_benchmark"):
        flags = cache.parent / "CMakeFiles" / f"{target}.dir" / "flags.make"
        if flags.exists():
            for line in flags.read_text(encoding="utf-8").splitlines():
                if line.startswith(("C_FLAGS = ", "C_DEFINES = ")):
                    key, value = line.split(" = ", 1)
                    result[f"{target}_{key.lower()}"] = value
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
    lifetime_values = defaultdict(list)
    outside_fractions = defaultdict(list)
    timing_contracts = defaultdict(set)
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
        timing_contracts[key].add(record.get("timing_contract", "native-v1"))
        measured = record["operation_ns"] if record["kind"] == "native" else record["simulate_ns"]
        if "total_ns" in record:
            total = float(record["total_ns"])
            if total < measured or measured <= 0:
                raise BenchmarkError(f"invalid timing boundary for {key[2]}/{key[3]}")
            lifetime_values[key].append(value * total / measured)
            outside_fractions[key].append((total - measured) / total)

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
            "p95_scope": "batch-average",
        }
        key = (kind, implementation, name, scale)
        if len(timing_contracts[key]) != 1:
            raise BenchmarkError(f"mixed timing contracts: {name}/{scale}")
        item["timing_contract"] = next(iter(timing_contracts[key]))
        if lifetime_values[key]:
            item["median_total_ns_per_op"] = statistics.median(lifetime_values[key])
            item["outside_operation_percent"] = statistics.median(outside_fractions[key]) * 100
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
            worker_counts = {int(record.get("parallel_workers", 1)) for record in diagnostics}
            if len(worker_counts) != 1:
                raise BenchmarkError(f"native worker count changed across runs: {name}/{scale}")
            item["parallel_workers"] = next(iter(worker_counts))
            if any("verification_contract" in r for r in diagnostics):
                for field in ("verification_contract", "verification_steps", "trajectory_hash", "contact_height"):
                    observed = {r.get(field) for r in diagnostics}
                    if None in observed or len(observed) != 1:
                        raise BenchmarkError(f"native verification differs across runs: {name}/{scale}/{field}")
                    item[field] = next(iter(observed))
            scoped = [r for r in diagnostics if r.get("phase_scope") == "simulation"]
            for record in scoped:
                wall = int(record["diagnostic_operation_ns"])
                if wall <= 0 or sum(record["phases_ns"].values()) > wall:
                    raise BenchmarkError(f"diagnostic phases exceed measured operation: {name}/{scale}")
            if scoped:
                item["diagnostics"] = [{key: r[key] for key in (
                    "diagnostic_operation_ns", "diagnostic_total_ns", "phases_ns",
                    "windows", "psi_latency_ns", "latency_operation_ns", "latency_total_ns",
                    "steps_per_world", "dt", "phase_scope") if key in r}
                    for r in scoped]
                item["phase_percent"] = {
                    phase: statistics.fmean(r["phases_ns"][phase] / r["diagnostic_operation_ns"] * 100
                                            for r in scoped)
                    for phase in scoped[0]["phases_ns"]
                }
        output.append(item)
    return output


def scaling_analysis(summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups = defaultdict(list)
    for item in summary:
        if item["kind"] == "native":
            groups[(item["implementation"], item["name"])].append(item)
    output = []
    for (implementation, name), items in sorted(groups.items()):
        items.sort(key=lambda r: r["scale"])
        for before, after in zip(items, items[1:]):
            size_ratio = after["scale"] / before["scale"]
            time_ratio = after["median_ns"] / before["median_ns"]
            output.append({"implementation": implementation, "case": name,
                           "from_scale": before["scale"], "to_scale": after["scale"],
                           "time_ratio": time_ratio,
                           "exponent": math.log(time_ratio) / math.log(size_ratio),
                           "unit": "ns/object-cycle" if name == "object_churn" else "ns/operation"})
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
        if int(item.get("parallel_workers", 1)) != int(previous.get("parallel_workers", 1)):
            raise BenchmarkError(
                f"baseline worker count differs: {item['name']}/{item['scale']}")
        if item.get("timing_contract", "native-v1") != previous.get("timing_contract", "native-v1"):
            raise BenchmarkError(f"baseline timing contract differs: {item['name']}/{item['scale']}; regenerate baseline")
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


def enforce_baseline(current: dict[str, Any], baseline_path: Path, *, require_targets: bool = True) -> dict[str, Any]:
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    if baseline.get("schema") != SCHEMA:
        raise BenchmarkError(f"baseline has unsupported schema: {baseline.get('schema')!r}")
    if current.get("profile") != baseline.get("profile"):
        raise BenchmarkError("checked baseline profile does not match current profile")
    if int(current.get("runs", 0)) < 3 or int(baseline.get("runs", 0)) < 3:
        raise BenchmarkError("checked baselines require at least three runs")

    current_meta = current["metadata"]
    baseline_meta = baseline.get("metadata", {})
    cpu = current_meta.get("machine", {}).get("cpu")
    if not cpu or cpu in ("unknown", "arm", "arm64", "x86_64"):
        raise BenchmarkError("checked baseline requires a detected CPU model")
    for field in ("machine", "compiler", "build_type", "ivp_real_double"):
        if current_meta.get(field) != baseline_meta.get(field):
            raise BenchmarkError(f"checked baseline metadata differs: {field}")
    if int(current_meta.get("parallel_workers", 1)) != int(
            baseline_meta.get("parallel_workers", 1)):
        raise BenchmarkError("checked baseline metadata differs: parallel_workers")
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
    if not now or set(now) != set(before):
        raise BenchmarkError("checked baseline native case/scale set differs")

    speedups: dict[str, float] = {}
    for key, item in now.items():
        previous = before[key]
        if item.get("timing_contract", "native-v1") != previous.get("timing_contract", "native-v1"):
            raise BenchmarkError(f"baseline timing contract differs: {key[0]}/{key[1]}")
        if item.get("state_hash") != previous.get("state_hash"):
            raise BenchmarkError(f"state hash differs for {key[0]}/{key[1]}")
        if item.get("verification_contract") != "native-trajectory-v1":
            raise BenchmarkError(f"missing trajectory verification for {key[0]}/{key[1]}; regenerate baseline")
        if not isinstance(item.get("trajectory_hash"), str) or not re.fullmatch(r"[0-9a-f]{16}", item["trajectory_hash"]) or \
                type(item.get("verification_steps")) is not int or item["verification_steps"] < 0:
            raise BenchmarkError(f"invalid trajectory verification for {key[0]}/{key[1]}")
        for field in ("verification_contract", "verification_steps", "trajectory_hash", "contact_height"):
            if item.get(field) != previous.get(field):
                raise BenchmarkError(f"verification differs for {key[0]}/{key[1]}/{field}; use a verified baseline")
        if int(item.get("residual_bytes", -1)) != 0 or int(previous.get("residual_bytes", -1)) != 0:
            raise BenchmarkError(f"residual allocation bytes for {key[0]}/{key[1]}")
        previous_peak = int(previous.get("peak_bytes", 0))
        if previous_peak and int(item.get("peak_bytes", 0)) > previous_peak * 1.30:
            raise BenchmarkError(f"peak allocation regression exceeds 30% for {key[0]}/{key[1]}")
        speedup = float(previous["median_ns"]) / float(item["median_ns"])
        speedups[key[0]] = speedup
        if speedup < (1.0 / 1.05):
            raise BenchmarkError(f"performance regression exceeds 5% for {key[0]}/{key[1]}")

    if require_targets and current["profile"] == "large":
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
        "requires_overall_targets": require_targets,
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
              f"workers={item.get('parallel_workers', 1):<2} "
              f"median={item['median_ns'] / 1000:9.3f} us/op "
              f"batch-p95={item['p95_ns'] / 1000:9.3f} us/op "
              f"cv={item['coefficient_of_variation'] * 100:5.1f}%")
        if "outside_operation_percent" in item:
            print(f"    outside operation={item['outside_operation_percent']:.1f}% of lifecycle wall time")
        if "phase_percent" in item:
            phases = sorted(item["phase_percent"].items(), key=lambda pair: -pair[1])[:3]
            print("    simulation phases: " + ", ".join(f"{p}={v:.1f}%" for p, v in phases))
            psi = [d["psi_latency_ns"] for d in item["diagnostics"] if "psi_latency_ns" in d]
            if psi:
                print(f"    diagnostic PSI p95={statistics.median(p['p95'] for p in psi) / 1000:.1f} us "
                      f"max={max(p['max'] for p in psi) / 1000:.1f} us ({psi[0]['scope']})")
    if result.get("scaling"):
        print("\nScale growth (per-operation exponent; churn is normalized per object)")
        for row in result["scaling"]:
            print(f"  {row['case']:<18} {row['from_scale']}->{row['to_scale']}: "
                  f"time {row['time_ratio']:.2f}x, exponent {row['exponent']:.2f}")
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
    if checked and checked.get("passed"):
        print("\nChecked baseline: PASS")
        if checked["large_geometric_mean"] is not None:
            print(f"  large geometric mean: {checked['large_geometric_mean']:.3f}x")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard", "large"), default="quick")
    parser.add_argument("--compare-reference", action="store_true")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--check-baseline", type=Path)
    parser.add_argument("--check-regression", type=Path,
                        help="check a recent checkpoint without reapplying the overall 2x target")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--case", action="append", dest="cases")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--native-only", action="store_true")
    parser.add_argument("--scale", type=int)
    parser.add_argument("--workers", type=int, default=1,
                        help="total execution width for each libivp world")
    args = parser.parse_args(argv)
    if args.runs <= 0:
        parser.error("--runs must be positive")
    if args.workers <= 0:
        parser.error("--workers must be positive")
    if args.profile == "large" and args.compare_reference:
        parser.error("large is a native-only profile; use standard for reference comparison")
    if args.scale is not None and (args.scale <= 0 or not args.cases or len(args.cases) != 1):
        parser.error("--scale must be positive and requires exactly one native --case")
    if args.native_only and args.compare_reference:
        parser.error("--native-only cannot be used with --compare-reference")

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
        "--workers", str(args.workers),
    ]
    if args.check or args.check_baseline:
        native_command.append("--check")
    if args.scale:
        native_command += ["--scale", str(args.scale)]
    native_filters = [case for case in (args.cases or []) if case in {
        "active_bodies", "contact_pile", "dense_contacts", "constraint_chain",
        "fast_impacts", "constraint_islands", "mixed_constraint_islands",
        "heterogeneous_constraint_islands", "object_churn", "sleeping_world",
        "convex_build",
    }]
    if args.scale and not native_filters:
        parser.error("--scale requires a native --case")
    records: list[dict[str, Any]] = []
    for run_index in range(args.runs):
        print(f"Native run {run_index + 1}/{args.runs} ({args.profile})", file=sys.stderr, flush=True)
        native_records: list[dict[str, Any]] = []
        if native_filters:
            for case in native_filters:
                native_records.extend(run_native(native_command + ["--case", case], cwd=root, source=case))
        elif not args.cases:
            native_records.extend(run_native(native_command, cwd=root, source="ivp_benchmark"))
        for record in native_records:
            record["run"] = run_index
        records.extend(native_records)

    all_scenarios = scenarios(root / "tools" / "scenarios.txt")
    chosen_names = set(
        () if args.profile == "large" or args.native_only or args.scale
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
    compiler = read_compiler(lib_build / "CMakeCache.txt")
    metadata: dict[str, Any] = {
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "machine": {
            "system": platform.system(), "release": platform.release(),
            "architecture": platform.machine(), "cpu": cpu_name(),
        },
        "python": platform.python_version(),
        "compiler": compiler,
        "libivp": git_info(root),
        "build_type": compiler.get("cmake_build_type", "unknown"),
        "ivp_real_double": compiler.get("ivp_real_double") == "ON",
        "parallel_workers": args.workers,
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
        "scaling": scaling_analysis(summary),
        "absolute_gate_failures": [{"case": r["case"], "scale": r["scale"], "run": r["run"]}
                                   for r in records if r["kind"] == "native-summary" and
                                   r.get("gate_ok") is False],
    }
    if args.baseline:
        result["baseline"] = baseline_comparison(result, args.baseline)
    if args.check_baseline:
        try:
            result["checked_baseline"] = enforce_baseline(result, args.check_baseline)
        except BenchmarkError as exc:
            result["checked_baseline"] = {"passed": False, "error": str(exc)}
    if args.check_regression:
        try:
            result["checked_regression"] = enforce_baseline(result, args.check_regression, require_targets=False)
        except BenchmarkError as exc:
            result["checked_regression"] = {"passed": False, "error": str(exc)}
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print_report(result)
    print(f"\nRaw result: {output_path}")
    if result["absolute_gate_failures"]:
        print(f"Absolute gates FAILED: {result['absolute_gate_failures']}", file=sys.stderr)
    if result.get("checked_baseline", {}).get("passed") is False:
        print(f"Checked baseline FAILED: {result['checked_baseline']['error']}", file=sys.stderr)
    if "checked_regression" in result:
        checked = result["checked_regression"]
        print("Incremental regression: " + ("PASS" if checked["passed"] else "FAILED: " + checked["error"]))
    return int(bool(result["absolute_gate_failures"]) or
               any(result.get(key, {}).get("passed") is False for key in ("checked_baseline", "checked_regression")))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BenchmarkError as exc:
        print(f"benchmark: {exc}", file=sys.stderr)
        raise SystemExit(1)
