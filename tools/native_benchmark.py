#!/usr/bin/env python3
"""Compare identical native hull/LCS fixtures with the external C++ engine.

Verification is untimed. Measurement alternates engine order between runs;
neither engine collects phase timings in the measured loop.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

sys.dont_write_bytecode = True
import benchmark as bench
import benchmark_trace as trace

CASES = {"fast_impacts": (32, 128, 512), "contact_pile": (64, 256, 1024)}


def measurement_build(compiler):
    if compiler.get("cmake_build_type") != "Release" or compiler.get("ivp_real_double") != "ON":
        raise bench.BenchmarkError("native comparison requires a Release, IVP_REAL_DOUBLE=ON build")
    for option in ("ivp_enable_asserts", "ivp_enable_stats", "ivp_container_debug", "ivp_enable_asan",
                   "ivp_enable_ubsan", "ivp_benchmark_diagnostics"):
        if compiler.get(option, "OFF") != "OFF":
            raise bench.BenchmarkError(f"timing binary must not enable {option}; use a separate diagnostic build")


def validate_samples(records, implementation, name, scale, samples):
    rows = [r for r in records if r.get("kind") == "native"]
    if len(rows) != samples or [r.get("sample") for r in rows] != list(range(samples)):
        raise bench.BenchmarkError("incomplete or reordered timing samples")
    for r in rows:
        if (r.get("implementation"), r.get("case"), r.get("scale"), r.get("timing_contract")) != (
                implementation, name, scale, "native-noop-v1"):
            raise bench.BenchmarkError("timing workload or counter contract differs")
        steps = 180 if name == "fast_impacts" else 360
        worlds = r.get("iterations")
        if type(worlds) is not int or worlds < 1 or r.get("operations") != worlds * steps:
            raise bench.BenchmarkError("invalid timing operation count")
        if not (0 < r.get("operation_ns", 0) <= r.get("total_ns", 0)):
            raise bench.BenchmarkError("invalid timing boundary")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard", "large"), default="large")
    parser.add_argument("--case", choices=tuple(CASES))
    parser.add_argument("--scale", type=int)
    parser.add_argument("--contact-height", type=int)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--min-sample-ms", type=float, default=500)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--work-counts", action="store_true",
                        help="also verify separately instrumented queue/LCS work-count binaries")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.runs < 1 or not 1 <= args.samples <= 64 or not math.isfinite(args.min_sample_ms) or args.min_sample_ms <= 0:
        parser.error("runs, samples (<=64) and finite minimum duration must be positive")
    if args.scale is not None and (not args.case or not 1 <= args.scale <= 16384):
        parser.error("--scale (1..16384) requires --case")
    if args.contact_height is not None and (args.case != "contact_pile" or args.contact_height < 1):
        parser.error("--contact-height requires --case contact_pile and a positive height")
    root = Path(__file__).resolve().parents[1]
    build = root / "build" / "benchmark"
    output = (args.output or build / "native-comparison.json").resolve()
    raw = output.parent / (output.stem + "-raw")
    raw.mkdir(parents=True, exist_ok=True)
    reference = Path(os.environ.get("IVP_REF_SRC", root.parent / "ivp")).resolve()
    if not args.no_build:
        bench.run(["cmake", "-S", str(root), "-B", str(build / "lib"),
                   "-DCMAKE_BUILD_TYPE=Release", "-DIVP_BUILD_BENCHMARKS=ON"], cwd=root)
        bench.run(["cmake", "--build", str(build / "lib"), "--target", "ivp_benchmark", "-j"], cwd=root)
        bench.run(["bash", str(root / "tools" / "build_native_reference.sh")], cwd=root)
        if args.work_counts:
            bench.run([sys.executable, str(root / "tools" / "build_native_probes.py")], cwd=root)
    binaries = {"libivp": build / "lib" / "ivp_benchmark",
                "reference": build / "native-reference" / "ivp_native_reference"}
    for binary in binaries.values():
        if not binary.is_file():
            raise bench.BenchmarkError(f"missing {binary}; omit --no-build")
    compiler = bench.read_compiler(build / "lib" / "CMakeCache.txt")
    measurement_build(compiler)
    ref_lib = Path(os.environ.get("IVP_REF_LIB_DIR", build / "reference/ref_release/libs"))
    reference_options = {}
    for line in (ref_lib / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if line.startswith(("CMAKE_CXX_COMPILER:", "CMAKE_CXX_FLAGS:", "CMAKE_CXX_FLAGS_RELEASE:", "CMAKE_BUILD_TYPE:")):
            key, value = line.split("=", 1)
            reference_options[key.split(":", 1)[0]] = value
    result = {"schema": bench.SCHEMA, "kind": "native-comparison", "profile": args.profile,
              "runs": args.runs, "samples": args.samples, "min_sample_ms": args.min_sample_ms,
              "counter_contract": "native-noop-v1", "workloads": [], "records": [],
              "metadata": {"generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                           "machine": {"system": platform.system(), "release": platform.release(),
                                       "architecture": platform.machine(), "cpu": bench.cpu_name()},
                           "libivp": bench.git_info(root), "reference": bench.git_info(reference),
                           "compiler": compiler,
                           "binary_sha256": {impl: hashlib.sha256(path.read_bytes()).hexdigest()
                                             for impl, path in binaries.items()},
                           "reference_compiler": bench.run(["c++", "--version"], cwd=root, capture=True).strip(),
                           "reference_build_options": reference_options,
                           "reference_runner_flags": "-O3 -DNDEBUG -ffp-contract=off -fno-fast-math -frounding-math"}}

    def save():
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    profile_index = ("quick", "standard", "large").index(args.profile)
    for name in ([args.case] if args.case else CASES):
        scale = args.scale or CASES[name][profile_index]
        if (args.contact_height or 0) > scale:
            parser.error("contact height must not exceed scale")
        common = ["--case", name, "--scale", str(scale)]
        if args.contact_height:
            common += ["--contact-height", str(args.contact_height)]
        commands = {impl: [str(binary), *common] for impl, binary in binaries.items()}
        commands["libivp"] += ["--profile", "large", "--format", "jsonl", "--noop-counter"]
        # All current profiles share these horizons; always specify one to
        # keep selection independent from a runner's future default.
        traces = {}
        print(f"Verify {name}/{scale} (height={args.contact_height or 'original'})", file=sys.stderr, flush=True)
        for impl, command in commands.items():
            data = bench.run(command + ["--verify"], cwd=root, capture=True)
            (raw / f"{name}-{scale}-{impl}-trace.jsonl").write_text(data, encoding="utf-8")
            traces[impl] = bench.parse_jsonl(data, impl)
        checked = trace.compare(traces["libivp"], traces["reference"], cross_engine=True)
        workload = {"case": name, "scale": scale, "contact_height": args.contact_height or 0,
                    "verification": checked,
                    "trajectory_hash": traces["libivp"][-1]["trajectory_hash"]}
        result["workloads"].append(workload)
        save()
        if not checked["passed"]:
            raise bench.BenchmarkError(f"first native mismatch: {name}/{scale}: {checked}; {output}")
        if args.work_counts:
            workload["work_counts"] = {}
            for impl, command in commands.items():
                probe = build / "native-probes" / impl
                observed = subprocess.run([str(probe), *command[1:], "--verify"], cwd=root,
                                          text=True, capture_output=True, check=False)
                (raw / f"{name}-{scale}-{impl}-work.jsonl").write_text(observed.stderr, encoding="utf-8")
                if observed.returncode:
                    raise bench.BenchmarkError(f"work probe failed: {impl}: {observed.stderr[-2000:]}")
                probe_trace = bench.parse_jsonl(observed.stdout, impl + " probe")
                checked_probe = trace.compare(traces[impl], probe_trace)
                if not checked_probe["passed"]:
                    raise bench.BenchmarkError(f"work instrumentation changes state: {checked_probe}")
                work = bench.parse_jsonl(observed.stderr, impl + " work")
                if len(work) != 1 or work[0].get("kind") != "native-work" or \
                        len(work[0].get("cumulative_steps", [])) != len(probe_trace) - 1:
                    raise bench.BenchmarkError("incomplete work-count trace")
                workload["work_counts"][impl] = work[0]
            workload["work_counts_equal"] = workload["work_counts"]["libivp"] == workload["work_counts"]["reference"]
            save()
        for run_index in range(args.runs):
            order = ("libivp", "reference") if run_index % 2 == 0 else ("reference", "libivp")
            for impl in order:
                print(f"Time {name}/{scale} run {run_index + 1}/{args.runs}: {impl}", file=sys.stderr, flush=True)
                data = bench.run(commands[impl] + ["--samples", str(args.samples),
                                 "--min-sample-ms", str(args.min_sample_ms)], cwd=root, capture=True)
                (raw / f"{name}-{scale}-{impl}-run{run_index}.jsonl").write_text(data, encoding="utf-8")
                records = bench.parse_jsonl(data, impl)
                validate_samples(records, impl, name, scale, args.samples)
                if impl == "libivp":
                    summaries = [r for r in records if r.get("kind") == "native-summary"]
                    if len(summaries) != 1 or summaries[0].get("residual_bytes") != 0 or \
                            summaries[0].get("trajectory_hash") != workload["trajectory_hash"]:
                        raise bench.BenchmarkError("timing run verification or allocator diagnostic differs")
                for row in records:
                    row["run"] = run_index
                result["records"].extend(records)
                save()
    result["summary"] = bench.summarize(result["records"])
    print("case/scale             C17 us/PSI     C++ us/PSI    C17/C++")
    for w in result["workloads"]:
        rows = {r["implementation"]: r for r in result["summary"]
                if r["name"] == w["case"] and r["scale"] == w["scale"]}
        a, b = rows["libivp"]["median_ns"], rows["reference"]["median_ns"]
        w["libivp_to_reference_ratio"] = a / b
        print(f"{w['case'] + '/' + str(w['scale']):<23} {a / 1000:11.3f} {b / 1000:13.3f} {a / b:10.3f}")
    save()
    print(f"Verified comparison: {output} (batch-average p95; not a realtime gate)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (bench.BenchmarkError, OSError, ValueError) as exc:
        print(f"native_benchmark: {exc}", file=sys.stderr)
        raise SystemExit(1)
