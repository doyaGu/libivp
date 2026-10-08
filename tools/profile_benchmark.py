#!/usr/bin/env python3
"""Sample native/scenario stacks and separate simulation from world lifecycle."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
import benchmark

PROFILE_SCHEMA = "libivp-hotspots-v1"
DEFAULT_CASES = ("active_bodies", "contact_pile", "constraint_chain", "fast_impacts",
                 "object_churn", "sleeping_world", "convex_build")


def sample_stacks(text: str, *, locations: bool = False) -> list:
    """Convert sample's inclusive call tree to weighted exclusive stacks.

    Indentation includes branch punctuation. Subtract direct children only;
    adding every tree row would double-count callers and recursive frames.
    """
    nodes = []
    active = []
    started = False
    for line in text.splitlines():
        if line == "Call graph:":
            started = True
            continue
        if not started:
            continue
        if line.startswith(("Total number in stack", "Sort by top", "Binary Images:")):
            break
        match = re.match(r"^([ +!:.|`-]*)(\d+) (.+)$", line)
        if not match:
            continue
        prefix, count, detail = match.groups()
        depth = len(prefix)
        is_thread = bool(re.match(r"Thread_\d+", detail))
        if is_thread:
            symbol = "[thread]"
            # Only the engine's main thread; utility threads are not workloads.
            main_thread = "com.apple.main-thread" in detail
        else:
            if "  (in " not in detail:
                continue
            symbol = detail.split("  (in ", 1)[0]
            main_thread = active[-1][1]["main"] if active else False
        while active and active[-1][0] >= depth:
            active.pop()
        node = {"count": int(count), "children": 0, "main": main_thread,
                "stack": (active[-1][1]["stack"] if active else ()) + (symbol,)}
        source = re.search(r"\]\s+(/.+):(\d+)\s*$", detail)
        node["source"] = f"{source[1]}:{source[2]}" if source and source[2] != "0" else None
        if active:
            active[-1][1]["children"] += int(count)
        nodes.append(node)
        active.append((depth, node))
    if not nodes:
        raise benchmark.BenchmarkError("sample report has no symbolized call graph")
    stacks = []
    for node in nodes:
        weight = node["count"] - node["children"]
        if weight < 0:
            raise benchmark.BenchmarkError("sample call tree has inconsistent child counts")
        if weight and node["main"]:
            stacks.append((weight, node["stack"], node["source"]) if locations
                          else (weight, node["stack"]))
    if not stacks:
        raise benchmark.BenchmarkError("sample report has no main-thread stacks")
    return stacks


def perf_stacks(text: str) -> list[tuple[int, tuple[str, ...]]]:
    """Read `perf script` call chains (leaf first, one event per blank block)."""
    stacks = []
    for block in re.split(r"\n\s*\n", text.strip()):
        symbols = []
        for line in block.splitlines():
            match = re.match(r"^\s+[0-9a-f]+\s+(.+?)\s+\(.+\)\s*$", line)
            if match:
                symbols.append(re.sub(r"\+0x[0-9a-f]+$", "", match[1]))
        if symbols:
            stacks.append((1, tuple(reversed(symbols))))
    if not stacks:
        raise benchmark.BenchmarkError("perf script has no symbolized call chains")
    return stacks


def scope_for(stack: tuple[str, ...]) -> str:
    if any(s == "bench_simulate" for s in stack):
        return "simulation"
    if "hash_environment" in stack or "hash_object" in stack:
        return "validation"
    if "bench_sleep_warmup" in stack:
        return "settling"
    if "bench_destroy" in stack:
        return "teardown"
    if any(s.startswith("ivp_build_convex_surface") for s in stack):
        return "convex_build"
    if "run_once" in stack:
        return "setup_or_churn"
    return "harness"


def scenario_scope_for(stack: tuple[str, ...]) -> str:
    # The two runners inline/tail-call different environment entry points.
    # Do not classify run_benchmark_world itself: it also creates/destroys
    # worlds. Native settling/validation classification remains separate.
    if any(s in ("ivp_environment_simulate_until", "ivp_time_manager_event_loop") or
           s.startswith("IVP_Time_Manager::event_loop(") for s in stack):
        return "simulation"
    return "lifecycle_or_harness"


def aggregate_stacks(stacks: list[tuple[int, tuple[str, ...]]], *, scope_classifier=None) -> dict:
    counts = Counter()
    exclusive = defaultdict(Counter)
    inclusive = defaultdict(Counter)
    chains = defaultdict(Counter)
    unresolved = Counter()
    locations = defaultdict(lambda: defaultdict(Counter))
    for entry in stacks:
        weight, stack = entry[:2]
        scope = (scope_classifier or scope_for)(stack)
        counts[scope] += weight
        exclusive[scope][stack[-1]] += weight
        for symbol in set(stack):
            inclusive[scope][symbol] += weight
        chains[scope][stack] += weight
        if len(entry) > 2 and entry[2]:
            locations[scope][stack[-1]][entry[2]] += weight
        if stack[-1] in ("???", "[unknown]") or stack[-1].startswith("0x"):
            unresolved[scope] += weight
    total = sum(counts.values())
    output = {}
    for scope, count in counts.items():
        functions = []
        for name in inclusive[scope]:
            self_count = exclusive[scope][name]
            share = self_count / count
            functions.append({
                "function": name, "self_samples": self_count,
                "inclusive_samples": inclusive[scope][name],
                "self_percent": share * 100,
                "inclusive_percent": inclusive[scope][name] / count * 100,
                "approx_self_percent_95_error": 196 * math.sqrt(share * (1 - share) / count),
                "max_speedup_if_self_eliminated": 1 / (1 - share) if share < 1 else None,
                "self_locations": [{"source": source, "samples": samples}
                                   for source, samples in locations[scope][name].most_common()],
            })
        output[scope] = {
            "samples": count, "process_percent": count / total * 100,
            "unresolved_leaf_samples": unresolved[scope],
            "functions": sorted(functions, key=lambda f: (-f["self_samples"], f["function"])),
            "stacks": [{"samples": n, "stack": list(stack)}
                       for stack, n in chains[scope].most_common(20)],
        }
    return {"samples": total, "scopes": output}


def validate_operation_counts(counts: dict | None, case: str) -> None:
    if not counts or counts["adds"] != counts["tree_adds"] + counts["skip_adds"] + counts["fallback_adds"] or counts["adds"] != sum(counts["population_buckets"]):
        raise benchmark.BenchmarkError(f"{case}: missing/inconsistent operation counts")
    # Local searches are a subset of skip-path insertions, not an additional
    # mutually exclusive insertion category. Older reports omit these fields.
    if not 0 <= counts.get("finger_hits", 0) <= counts.get("finger_attempts", 0) <= counts["skip_adds"]:
        raise benchmark.BenchmarkError(f"{case}: inconsistent finger operation counts")


def validate_broadphase_counts(counts: dict | None, case: str) -> None:
    fields = ("rechecks", "recursive_rechecks", "max_depth", "candidates", "old_collisions",
              "max_candidates", "max_old_collisions", "potential_search_pairs", "searches",
              "found", "misses", "entries_compared", "max_scan", "swaps")
    if not isinstance(counts, dict) or any(type(counts.get(f)) is not int or counts[f] < 0
                                          for f in fields):
        raise benchmark.BenchmarkError(f"{case}: missing/invalid broadphase operation counts")
    index_fields = ("indexed_rechecks", "indexed_searches", "indexed_found", "index_entries",
                    "index_probes", "max_index_probes", "duplicate_fallbacks", "unsafe_fallbacks", "oom_fallbacks")
    if any(f in counts for f in index_fields):
        if (any(type(counts.get(f)) is not int or counts[f] < 0 for f in index_fields)
                or counts["indexed_rechecks"] > counts["rechecks"]
                or counts["indexed_searches"] > counts["searches"]
                or counts["indexed_found"] > min(counts["found"], counts["indexed_searches"])
                or counts["index_probes"] < counts["indexed_searches"]
                or counts["max_index_probes"] > counts["index_probes"]
                or counts["index_entries"] > counts["old_collisions"]):
            raise benchmark.BenchmarkError(f"{case}: missing/inconsistent broadphase index counts")
    if (counts["searches"] != counts["found"] + counts["misses"]
            or counts["searches"] > counts["candidates"]
            or counts["swaps"] > counts["found"]
            or counts["entries_compared"] < counts["found"] - counts.get("indexed_found", 0)
            or counts["entries_compared"] > counts["potential_search_pairs"]
            or counts["max_scan"] > counts["max_old_collisions"]
            or counts["max_candidates"] > counts["candidates"]
            or counts["max_old_collisions"] > counts["old_collisions"]
            or counts["recursive_rechecks"] > counts["rechecks"]
            or counts["max_depth"] > counts["rechecks"]):
        raise benchmark.BenchmarkError(f"{case}: inconsistent broadphase operation counts")


def validate_hull_counts(counts: dict | None, case: str) -> None:
    fields = ("manager_checks", "callbacks", "max_callbacks_per_check",
              "polygon_callbacks", "anchor_callbacks", "oo_watcher_callbacks",
              "oo_connector_callbacks", "synapse_inserts", "lazy_synapse_inserts",
              "synapse_updates", "lazy_synapse_updates", "synapse_removes",
              "halfspace_extensions", "exact_promotions", "phantom_promotions",
              "recursive_events", "oo_events", "callback_bucket_upper_bounds",
              "callback_buckets")
    scalar_fields = fields[:-2]
    if (not isinstance(counts, dict)
            or any(type(counts.get(f)) is not int or counts[f] < 0 for f in scalar_fields)
            or counts.get("callback_bucket_upper_bounds") != [0, 1, 3, 7, 15, 31, None]
            or not isinstance(counts.get("callback_buckets"), list)
            or len(counts["callback_buckets"]) != 7
            or any(type(value) is not int or value < 0 for value in counts["callback_buckets"])):
        raise benchmark.BenchmarkError(f"{case}: missing/invalid hull operation counts")
    typed = (counts["polygon_callbacks"] + counts["anchor_callbacks"]
             + counts["oo_watcher_callbacks"] + counts["oo_connector_callbacks"])
    if (typed != counts["callbacks"]
            or counts["max_callbacks_per_check"] > counts["callbacks"]
            or counts["halfspace_extensions"] + counts["exact_promotions"]
               + counts["phantom_promotions"] + counts["recursive_events"]
               > counts["polygon_callbacks"]
            or counts["oo_events"] > counts["oo_watcher_callbacks"]
            or sum(counts["callback_buckets"]) != counts["manager_checks"]):
        raise benchmark.BenchmarkError(f"{case}: inconsistent hull operation counts")


def validate_contact_recalc_counts(counts: dict | None, case: str) -> None:
    fields = ("calls", "branch_pp", "branch_pk", "branch_pf", "branch_kk",
              "same_time_calls", "same_geometry_fingerprint",
              "same_kinematic_fingerprint", "same_full_input_fingerprint",
              "max_calls_per_contact_at_time", "side_visits",
              "object_pose_first_uses", "object_pose_reuses",
              "core_state_first_uses", "core_state_reuses",
              "point_world_uses", "point_world_first_uses", "point_world_reuses",
              "edge_world_uses", "edge_world_first_uses", "edge_world_reuses",
              "face_normal_uses", "face_normal_first_uses", "face_normal_reuses",
              "face_tangent_uses", "face_tangent_first_uses", "face_tangent_reuses",
              "tracking_overflows")
    if (not isinstance(counts, dict) or
            counts.get("scope") != "per-simulation-time-v1" or
            counts.get("fingerprint_contract") != "contact-input-128-v1" or
            any(type(counts.get(field)) is not int or counts[field] < 0 for field in fields)):
        raise benchmark.BenchmarkError(f"{case}: missing/invalid contact recalc counts")
    if (counts["calls"] != counts["branch_pp"] + counts["branch_pk"] +
            counts["branch_pf"] + counts["branch_kk"] or
            counts["same_time_calls"] > counts["calls"] or
            counts["same_kinematic_fingerprint"] > counts["same_geometry_fingerprint"] or
            counts["same_full_input_fingerprint"] > counts["same_kinematic_fingerprint"] or
            counts["side_visits"] != counts["calls"] * 2 or
            counts["side_visits"] != counts["object_pose_first_uses"] +
            counts["object_pose_reuses"] or
            counts["side_visits"] != counts["core_state_first_uses"] +
            counts["core_state_reuses"] or counts["tracking_overflows"] != 0):
        raise benchmark.BenchmarkError(f"{case}: inconsistent contact recalc counts")
    for prefix in ("point_world", "edge_world", "face_normal", "face_tangent"):
        if counts[f"{prefix}_uses"] != (counts[f"{prefix}_first_uses"] +
                                        counts[f"{prefix}_reuses"]):
            raise benchmark.BenchmarkError(f"{case}: inconsistent contact feature counts")


def capture_command(runner: Path, command: list[str], args, raw: Path) -> tuple[str, list]:
    if args.backend == "sample":
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            profiler = subprocess.run(
                ["sample", str(process.pid), str(args.seconds), "1", "-mayDie",
                 "-fullPaths", "-file", str(raw)], capture_output=True, text=True)
            if profiler.returncode:
                raise benchmark.BenchmarkError(f"sample failed: {profiler.stderr.strip()}")
            try:
                stdout, stderr = process.communicate(timeout=args.seconds * 5 + 60)
            except subprocess.TimeoutExpired as exc:
                raise benchmark.BenchmarkError("profile runner exceeded its lifecycle timeout") from exc
            if process.returncode:
                raise benchmark.BenchmarkError(f"profile runner failed: {stderr.strip()}")
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
        stacks = sample_stacks(raw.read_text(encoding="utf-8", errors="replace"), locations=True)
    else:
        stdout = benchmark.run(
            ["perf", "record", "-F", "997", "-g", "--call-graph", "dwarf",
             "-o", str(raw), "--", *command], cwd=runner.parent, capture=True)
        script = benchmark.run(["perf", "script", "-i", str(raw)], cwd=runner.parent, capture=True)
        raw.with_suffix(".txt").write_text(script, encoding="utf-8")
        stacks = perf_stacks(script)
    return stdout, stacks


def capture(runner: Path, args, case: str, directory: Path, counts_runner: Path | None = None) -> dict:
    raw = directory / f"{case}.{'sample.txt' if args.backend == 'sample' else 'perf.data'}"
    command = [str(runner), "--profile", args.profile, "--case", case,
               "--hotspot-ms", str((args.seconds + 2) * 1000), "--format", "jsonl"]
    if args.scale:
        command += ["--scale", str(args.scale)]
    stdout, stacks = capture_command(runner, command, args, raw)
    record = benchmark.parse_jsonl(stdout, case)
    if len(record) != 1 or record[0].get("kind") != "native-profile":
        raise benchmark.BenchmarkError(f"{case}: expected one native-profile record")
    report = aggregate_stacks(stacks)
    report.update({"case": case, "scale": record[0]["scale"], "workload": record[0], "raw": str(raw)})
    target = ("convex_build" if case == "convex_build" else
              "setup_or_churn" if case == "object_churn" else "simulation")
    scoped = report["scopes"].get(target, {})
    report["adequate_samples"] = (scoped.get("samples", 0) >= args.min_samples
                                  and scoped.get("unresolved_leaf_samples", 0)
                                  <= scoped.get("samples", 0) * 0.10)
    verify = [str(runner), "--profile", args.profile, "--case", case,
              "--samples", "1", "--min-sample-ms", "1", "--format", "jsonl"]
    if args.scale:
        verify += ["--scale", str(args.scale)]
    expected = benchmark.parse_jsonl(benchmark.run(verify, cwd=runner.parent, capture=True), case)[-1]
    if expected.get("state_hash") != record[0]["state_hash"] or expected.get("residual_bytes") != 0:
        raise benchmark.BenchmarkError(f"{case}: profiling hash/residual allocation verification failed")
    report["state_hash_verified"] = True
    if counts_runner:
        verify[0] = str(counts_runner)
        counted = benchmark.parse_jsonl(benchmark.run(verify, cwd=counts_runner.parent, capture=True), case)[-1]
        if counted.get("state_hash") != record[0]["state_hash"] or counted.get("residual_bytes") != 0:
            raise benchmark.BenchmarkError(f"{case}: operation-count run changed state or leaked memory")
        counts = counted.get("minlist_operations")
        validate_operation_counts(counts, case)
        report["minlist_operations"] = counts
        # Older operation runners omit broadphase counters.
        if "broadphase_operations" in counted:
            broadphase = counted["broadphase_operations"]
            validate_broadphase_counts(broadphase, case)
            report["broadphase_operations"] = broadphase
        if "hull_operations" in counted:
            hull = counted["hull_operations"]
            validate_hull_counts(hull, case)
            report["hull_operations"] = hull
        if "contact_recalc_operations" in counted:
            contact = counted["contact_recalc_operations"]
            validate_contact_recalc_counts(contact, case)
            report["contact_recalc_operations"] = contact
    return report


def scenario_command(runner: Path, scene: tuple[str, str, int], implementation: str,
                     *, seconds: int | None = None) -> list[str]:
    name, delta, steps = scene
    command = [str(runner), "--scenario", name, "--dt", delta, "--steps", str(steps)]
    if seconds is not None:
        command += ["--benchmark", "--samples", "1", "--warmup", "1",
                    "--min-sample-ms", str((seconds + 2) * 1000)]
    if implementation == "libivp":
        command += ["--counter", "noop"]
    return command


def validate_scenario_record(record: dict, scene: tuple[str, str, int], implementation: str) -> None:
    name, delta, steps = scene
    if (record.get("kind") != "scenario" or record.get("implementation") != implementation or
            record.get("scenario") != name or record.get("steps_per_world") != steps or
            record.get("dt") != float(delta) or type(record.get("worlds")) is not int or
            record["worlds"] <= 0 or type(record.get("simulate_ns")) is not int or
            type(record.get("total_ns")) is not int or
            not 0 < record["simulate_ns"] <= record["total_ns"]):
        raise benchmark.BenchmarkError(f"{implementation}:{name}: mismatched scenario profile record")
    if implementation == "libivp" and (record.get("counter_mode") != "noop" or
                                      record.get("timing_contract") != "scenario-noop-v1"):
        raise benchmark.BenchmarkError(f"{name}: scenario profile requires the noop counter")


def verify_scenario_snapshots(runners: dict[str, Path], scene: tuple[str, str, int]) -> dict:
    outputs = {}
    for implementation, runner in runners.items():
        try:
            result = subprocess.run(scenario_command(runner, scene, implementation),
                                    capture_output=True, timeout=60)
        except subprocess.TimeoutExpired as exc:
            raise benchmark.BenchmarkError(f"{implementation}:{scene[0]}: snapshot timed out") from exc
        if result.returncode:
            raise benchmark.BenchmarkError(f"{implementation}:{scene[0]}: snapshot runner failed")
        if not result.stdout.strip():
            raise benchmark.BenchmarkError(f"{implementation}:{scene[0]}: empty snapshot output")
        outputs[implementation] = (result.stdout, result.stderr)
    if "reference" in outputs and outputs["reference"] != outputs["libivp"]:
        raise benchmark.BenchmarkError(
            f"{scene[0]}: reference snapshots differ (stdout/stderr); run tools/parity/check.sh")
    stdout, stderr = outputs["libivp"]
    return {"stdout_sha256": hashlib.sha256(stdout).hexdigest(),
            "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
            "reference_byte_identical": True if "reference" in outputs else None}


def scenario_contact_counts(runner: Path, scene: tuple[str, str, int], verification: dict) -> dict:
    snapshot = subprocess.run(scenario_command(runner, scene, "libivp"), capture_output=True,
                              timeout=60)
    if snapshot.returncode:
        raise benchmark.BenchmarkError(f"{scene[0]}: diagnostic snapshot runner failed")
    if (hashlib.sha256(snapshot.stdout).hexdigest() != verification["stdout_sha256"] or
            hashlib.sha256(snapshot.stderr).hexdigest() != verification["stderr_sha256"]):
        raise benchmark.BenchmarkError(f"{scene[0]}: contact diagnostics changed the snapshot")
    command = scenario_command(runner, scene, "libivp")
    command += ["--benchmark", "--samples", "1", "--warmup", "0",
                "--min-sample-ms", "0.001"]
    output = benchmark.run(command, cwd=runner.parent, capture=True)
    records = benchmark.parse_jsonl(output, f"contact diagnostics:{scene[0]}")
    if len(records) != 1:
        raise benchmark.BenchmarkError(f"{scene[0]}: expected one diagnostic scenario record")
    validate_scenario_record(records[0], scene, "libivp")
    counts = records[0].get("contact_recalc_operations")
    validate_contact_recalc_counts(counts, scene[0])
    return counts


def capture_scenario(runner: Path, args, scene: tuple[str, str, int], implementation: str,
                     directory: Path, verification: dict,
                     counts_runner: Path | None = None) -> dict:
    name, delta, steps = scene
    raw = directory / f"{name}-{implementation}.{'sample.txt' if args.backend == 'sample' else 'perf.data'}"
    command = scenario_command(runner, scene, implementation, seconds=args.seconds)
    stdout, stacks = capture_command(runner, command, args, raw)
    records = benchmark.parse_jsonl(stdout, f"{implementation}:{name}")
    if len(records) != 1:
        raise benchmark.BenchmarkError(f"{implementation}:{name}: expected one scenario record")
    validate_scenario_record(records[0], scene, implementation)
    report = aggregate_stacks(stacks, scope_classifier=scenario_scope_for)
    scoped = report["scopes"].get("simulation", {})
    report.update({"case": name, "kind": "scenario-profile", "implementation": implementation,
                   "dt": float(delta), "steps": steps, "workload": records[0],
                   "command": command, "raw": str(raw), "snapshot_verification": verification,
                   "counter_contract": "scenario-noop-v1",
                   "adequate_samples": scoped.get("samples", 0) >= args.min_samples and
                   scoped.get("unresolved_leaf_samples", 0) <= scoped.get("samples", 0) * 0.10})
    if counts_runner:
        report["contact_recalc_operations"] = scenario_contact_counts(
            counts_runner, scene, verification)
    return report


def print_report(result: dict, top: int) -> None:
    for case in result["cases"]:
        fixture = (f"{case['implementation']} steps={case['steps']} dt={case['dt']}"
                   if case.get("kind") == "scenario-profile" else f"n={case['scale']}")
        print(f"\n{case['case']} {fixture} main-thread samples={case['samples']}")
        for scope, item in sorted(case["scopes"].items(), key=lambda p: -p[1]["samples"]):
            print(f"  {scope}: {item['process_percent']:.1f}% of process samples ({item['samples']})")
            if scope not in ("simulation", "convex_build", "setup_or_churn", "teardown"):
                continue
            for function in item["functions"][:top]:
                print(f"    self={function['self_percent']:5.1f}% "
                      f"inclusive={function['inclusive_percent']:5.1f}%  {function['function']}")
                if function["self_locations"]:
                    print(f"      {function['self_locations'][0]['source']}")
        if not case["adequate_samples"]:
            print("  INSUFFICIENT: increase --seconds; too few scoped samples or unresolved symbols")
        if "minlist_operations" in case:
            c = case["minlist_operations"]
            adds = c["adds"]
            print(f"  minlist (separate intrusive run): adds={adds}, tree={c['tree_adds']}, "
                  f"skip={c['skip_adds']}, fallback={c['fallback_adds']}, max population={c['max_population']}")
            print(f"    comparisons/add={c['search_comparisons'] / adds if adds else 0:.1f}, "
                  f"max comparisons={c['max_search_comparisons']}, population buckets={c['population_buckets']}")
            if "finger_attempts" in c:
                attempts = c["finger_attempts"]
                print(f"    cached skip-prefix hits={c['finger_hits']}/{attempts} "
                      f"({100 * c['finger_hits'] / attempts if attempts else 0:.1f}% of attempts)")
        if "broadphase_operations" in case:
            c = case["broadphase_operations"]
            searches = c["searches"]
            print(f"  broadphase (separate intrusive run): rechecks={c['rechecks']}, "
                  f"candidates={c['candidates']}, old collisions={c['old_collisions']}")
            print(f"    searches={searches}, found={c['found']}, misses={c['misses']}, "
                  f"comparisons/search={c['entries_compared'] / searches if searches else 0:.1f}, "
                  f"max scan={c['max_scan']}, swaps={c['swaps']}")
            print(f"    max candidates={c['max_candidates']}, max old collisions={c['max_old_collisions']}, "
                  f"recursive rechecks={c['recursive_rechecks']}, max depth={c['max_depth']}")
            if "indexed_searches" in c:
                searches = c["indexed_searches"]
                print(f"    indexed rechecks={c['indexed_rechecks']}, queries={searches}, "
                      f"entries built={c['index_entries']}, "
                      f"probes/query={c['index_probes'] / searches if searches else 0:.1f}, "
                      f"max probes={c['max_index_probes']}")
                print(f"    fallback rechecks: duplicate={c['duplicate_fallbacks']}, "
                      f"unsafe={c['unsafe_fallbacks']}, OOM={c['oom_fallbacks']}")
        if "hull_operations" in case:
            c = case["hull_operations"]
            print(f"  hull queues (separate intrusive run): checks={c['manager_checks']}, "
                  f"callbacks={c['callbacks']}, max/check={c['max_callbacks_per_check']}")
            print(f"    callbacks/check buckets={c['callback_buckets']} "
                  f"for upper bounds={c['callback_bucket_upper_bounds']}")
            print(f"    callback types: polygon={c['polygon_callbacks']}, anchor={c['anchor_callbacks']}, "
                  f"watcher={c['oo_watcher_callbacks']}, connector={c['oo_connector_callbacks']}")
            print(f"    polygon paths: halfspace={c['halfspace_extensions']}, "
                  f"exact={c['exact_promotions']}, phantom={c['phantom_promotions']}, "
                  f"recursive={c['recursive_events']}")
            print(f"    queue calls: insert={c['synapse_inserts']}, lazy insert={c['lazy_synapse_inserts']}, "
                  f"update={c['synapse_updates']}, lazy update={c['lazy_synapse_updates']}, "
                  f"remove={c['synapse_removes']}")
        if "contact_recalc_operations" in case:
            c = case["contact_recalc_operations"]
            sides = c["side_visits"]
            print(f"  contact recalc (separate intrusive run): calls={c['calls']}, "
                  f"same-time={c['same_time_calls']}, "
                  f"full-input-equal={c['same_full_input_fingerprint']}")
            print(f"    branches pp/pk/pf/kk={c['branch_pp']}/{c['branch_pk']}/"
                  f"{c['branch_pf']}/{c['branch_kk']}")
            print(f"    shared object-pose reuse={c['object_pose_reuses'] / sides if sides else 0:.1%}, "
                  f"core-state reuse={c['core_state_reuses'] / sides if sides else 0:.1%}")
            for label, prefix in (("point transforms", "point_world"),
                                  ("edge transforms", "edge_world"),
                                  ("face normals", "face_normal"),
                                  ("face tangents", "face_tangent")):
                uses = c[f"{prefix}_uses"]
                print(f"    {label}: uses={uses}, reuse={c[f'{prefix}_reuses'] / uses if uses else 0:.1%}")


def main(argv=None) -> int:
    root = Path(__file__).resolve().parents[1]
    all_scenes = benchmark.scenarios(root / "tools" / "scenarios.txt")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard", "large"), default="large")
    parser.add_argument("--case", action="append", choices=DEFAULT_CASES, dest="cases")
    parser.add_argument("--scale", type=int)
    parser.add_argument("--seconds", type=int, default=5)
    parser.add_argument("--backend", choices=("auto", "sample", "perf"), default="auto")
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument("--top", type=int, default=8)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--operation-counts", action="store_true")
    parser.add_argument("--scenario", action="append", choices=[s[0] for s in all_scenes], dest="scenarios",
                        help="profile a fixed parity scene instead of a scalable native case")
    parser.add_argument("--compare-reference", action="store_true",
                        help="also sample the original IVP runner; requires matching snapshots")
    args = parser.parse_args(argv)
    if args.seconds < 1 or args.min_samples < 1 or args.top < 1 or (args.scale is not None and args.scale < 1):
        parser.error("seconds, min-samples, top and scale must be positive")
    if args.scale and (not args.cases or len(args.cases) != 1):
        parser.error("--scale requires exactly one --case")
    if args.scenarios and (args.cases or args.scale):
        parser.error("--scenario cannot be combined with --case or --scale")
    if args.compare_reference and not args.scenarios:
        parser.error("--compare-reference requires --scenario")
    if args.scenarios and sys.platform == "win32":
        parser.error("scenario profiling uses the non-Windows noop reference counter contract")
    if not args.scenarios and args.profile == "standard" and not args.scale:
        parser.error("standard profiling requires --case and --scale to isolate one configuration")
    if args.backend == "auto":
        args.backend = "sample" if sys.platform == "darwin" else "perf"
    if not shutil.which(args.backend):
        raise benchmark.BenchmarkError(f"{args.backend} is unavailable; install the OS profiler")
    base = root / "build" / "benchmark" / "hotspots"
    directory = base / dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    directory.mkdir(parents=True, exist_ok=True)
    build = root / "build" / "benchmark" / "profile-lib"
    target = "test_scenarios" if args.scenarios else "ivp_benchmark"
    if not args.no_build:
        benchmark.run(["cmake", "-S", str(root), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release",
                       "-DIVP_BUILD_BENCHMARKS=ON",
                       f"-DIVP_BUILD_TESTS={'ON' if args.scenarios else 'OFF'}",
                       f"-DIVP_ENABLE_SCENARIO_TESTS={'ON' if args.scenarios else 'OFF'}",
                       "-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG -g -fno-omit-frame-pointer -fno-optimize-sibling-calls"], cwd=root)
        benchmark.run(["cmake", "--build", str(build), "--target", target, "-j", "8"], cwd=root)
    runner = build / target
    if not runner.exists():
        raise benchmark.BenchmarkError(f"missing profiling runner: {runner}")
    counts_runner = None
    if args.operation_counts:
        count_build = root / "build" / "benchmark" / "operation-lib"
        count_target = "test_scenarios" if args.scenarios else "ivp_benchmark"
        if not args.no_build:
            benchmark.run(["cmake", "-S", str(root), "-B", str(count_build),
                           "-DCMAKE_BUILD_TYPE=Release", "-DIVP_BUILD_BENCHMARKS=ON",
                           f"-DIVP_BUILD_TESTS={'ON' if args.scenarios else 'OFF'}",
                           f"-DIVP_ENABLE_SCENARIO_TESTS={'ON' if args.scenarios else 'OFF'}",
                           "-DIVP_BENCHMARK_DIAGNOSTICS=ON"], cwd=root)
            benchmark.run(["cmake", "--build", str(count_build), "--target", count_target,
                           "-j", "8"], cwd=root)
        counts_runner = count_build / count_target
        if not counts_runner.exists():
            raise benchmark.BenchmarkError(f"missing operation-count runner: {counts_runner}")
    result = {
        "schema": PROFILE_SCHEMA, "profile": "scenario" if args.scenarios else args.profile, "backend": args.backend,
        "seconds": args.seconds, "metadata": {"libivp": benchmark.git_info(root),
        "system": platform.system(), "architecture": platform.machine(),
        "cpu": benchmark.cpu_name(),
        "compiler": benchmark.read_compiler(build / "CMakeCache.txt")},
        "cases": [],
    }
    if counts_runner:
        result["metadata"]["operation_runner"] = {
            "path": str(counts_runner),
            "sha256": hashlib.sha256(counts_runner.read_bytes()).hexdigest(),
        }
    if args.scenarios:
        runners = {"libivp": runner}
        if args.compare_reference:
            reference_root = Path(os.environ.get("IVP_REF_SRC", root.parent / "ivp")).resolve()
            reference_build = root / "build" / "benchmark" / "reference"
            if not args.no_build:
                env = dict(os.environ, IVP_REF_SRC=str(reference_root), IVP_PARITY_DIR=str(reference_build))
                benchmark.run([str(root / "tools" / "parity" / "build_reference.sh"), "release"], cwd=root, env=env)
            reference_runner = reference_build / "ref_release" / "runner" / "legacy_runner"
            if not reference_runner.exists():
                raise benchmark.BenchmarkError(f"missing reference runner: {reference_runner}")
            runners["reference"] = reference_runner
            result["metadata"]["reference"] = benchmark.git_info(reference_root)
        result["metadata"]["runners"] = {
            label: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for label, path in runners.items()}
        result["metadata"]["build_skipped"] = args.no_build
        result["metadata"]["counter_contract"] = "scenario-noop-v1"
        for index, scene in enumerate(s for s in all_scenes if s[0] in args.scenarios):
            verification = verify_scenario_snapshots(runners, scene)
            order = list(runners.items())
            if index % 2:
                order.reverse()
            for implementation, binary in order:
                print(f"Sampling {implementation}:{scene[0]} for {args.seconds}s...", file=sys.stderr, flush=True)
                diagnostic_runner = counts_runner if implementation == "libivp" else None
                result["cases"].append(capture_scenario(
                    binary, args, scene, implementation, directory, verification,
                    diagnostic_runner))
    else:
        for case in args.cases or DEFAULT_CASES:
            print(f"Sampling {case} for {args.seconds}s...", file=sys.stderr, flush=True)
            result["cases"].append(capture(runner, args, case, directory, counts_runner))
    path = args.output or base / "results.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print_report(result, args.top)
    print(f"\nHotspot report: {path}")
    return 0 if all(case["adequate_samples"] for case in result["cases"]) else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except benchmark.BenchmarkError as exc:
        print(f"profile: {exc}", file=sys.stderr)
        raise SystemExit(1)
