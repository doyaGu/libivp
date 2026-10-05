#!/usr/bin/env python3
"""Build intrusive work-count binaries from copies; never edit engine sources.

Unix diagnostic companion to native_benchmark.py. Source anchors fail closed
when the reference or port changes. Timings from these binaries are invalid.
"""
from __future__ import annotations
import os
from pathlib import Path
import re
import sys
sys.dont_write_bytecode = True
import benchmark as bench
from native_benchmark import measurement_build


def inject(source: str, signature: str, call: str) -> str:
    matches = list(re.finditer(signature + r"\s*\{", source))
    if len(matches) != 1:
        raise bench.BenchmarkError(f"expected one probe anchor, got {len(matches)}: {signature}")
    end = matches[0].end()
    return source[:end] + "\n    " + call + source[end:]


def build():
    root = Path(__file__).resolve().parents[1]
    reference = Path(os.environ.get("IVP_REF_SRC", root.parent / "ivp")).resolve()
    output = root / "build/benchmark/native-probes"
    output.mkdir(parents=True, exist_ok=True)
    lib = root / "build/benchmark/lib"
    measurement_build(bench.read_compiler(lib / "CMakeCache.txt"))
    ref_lib = Path(os.environ.get("IVP_REF_LIB_DIR", root / "build/benchmark/reference/ref_release/libs"))
    includes = ["-I" + str(reference / d) for d in (
        "ivp_compact_builder", "ivp_surface_manager", "ivp_collision", "ivp_physics",
        "ivp_utility", "ivp_controller", "ivp_intern")]
    includes += ["-I" + str(ref_lib / "include/ivp"), "-I" + str(root / "benchmarks")]
    cflags = ["-std=c17", "-O3", "-DNDEBUG", "-DIVP_REAL_DOUBLE", "-ffp-contract=off",
              "-fno-fast-math", "-frounding-math", "-fno-strict-aliasing", "-I" + str(root / "include"),
              "-I" + str(root / "benchmarks"), "-I" + str(root / "src/containers")]
    rflags = ["-std=c++17", "-O3", "-DNDEBUG", "-DIVP_VERSION_SDK", "-DUNIX",
              "-ffp-contract=off", "-fno-strict-aliasing", "-w", *includes]
    if sys.platform == "darwin": rflags += ["-DAPPLE"]
    replacements = [
        (root / "src/containers/ivp_minlist.c", "c-list.c", [
            (r"ivp_minlist_index_t ivp_minlist_add\([^)]*\)", "if (ml) bench_work_add(ml, ml->counter + 1);"),
            (r"void ivp_minlist_remove\([^)]*\)", "if (ml) bench_work_remove(ml);")]),
        (root / "src/solver/ivp_linear_constraint_solver.c", "c-lcs.c", [
            (r"ivp_result_t ivp_lcs_init_and_solve_in_memory\([^)]*\)", "bench_work_lcs(n, actives_at_begin);")]),
        (reference / "ivp_utility/ivu_min_list.cxx", "ref-list.cpp", [
            (r"IVP_U_MINLIST_INDEX IVP_U_Min_List::add\([^)]*\)", "bench_work_add(this, counter + 1);"),
            (r"void IVP_U_Min_List::remove_minlist_elem\([^)]*\)", "bench_work_remove(this);")]),
        (reference / "ivp_physics/ivp_great_matrix.cxx", "ref-lcs.cpp", [
            (r"IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::init_and_solve_lc\([^)]*\)",
             "bench_work_lcs(var_num, actives_at_begin);")]),
    ]
    for source, name, hooks in replacements:
        text = source.read_text(encoding="utf-8")
        for signature, call in hooks: text = inject(text, signature, call)
        # Generated instrumentation copy, not an engine source edit.
        target = output / name
        target.write_text('#include "bench_work.h"\n' + text, encoding="utf-8")
        compiler, flags = ("cc", cflags) if name.startswith("c-") else ("c++", rflags)
        bench.run([compiler, *flags, "-c", str(target), "-o", str(target.with_suffix(".o"))], cwd=root)
    common = output / "work.o"
    bench.run(["cc", *cflags, "-c", str(root / "benchmarks/bench_work.c"), "-o", str(common)], cwd=root)
    bench.run(["cc", *cflags, "-DBENCH_WORK_DIAGNOSTICS", str(root / "benchmarks/ivp_benchmark.c"),
               str(output / "c-list.o"), str(output / "c-lcs.o"), str(common), str(lib / "libivp.a"),
               str(lib / "libivp_qhull.a"), "-lm", "-o", str(output / "libivp")], cwd=root)
    libraries = [str(ref_lib / ("libivp_" + name + ".a")) for name in (
        "physics", "compact_builder", "collision", "surface_manager", "controller", "utility")]
    bench.run(["c++", *rflags, "-fno-fast-math", "-frounding-math", "-DIVP_ENABLE_QHULL",
               "-DBENCH_WORK_DIAGNOSTICS", str(root / "benchmarks/ivp_native_reference.cpp"),
               str(output / "ref-list.o"), str(output / "ref-lcs.o"), str(common),
               *libraries, *libraries, "-o", str(output / "reference")], cwd=root)


if __name__ == "__main__":
    try:
        build()
    except (bench.BenchmarkError, OSError) as exc:
        print(f"native probes: {exc}", file=sys.stderr)
        raise SystemExit(1)
