#!/usr/bin/env python3
"""Which public headers do the examples use?

For every header below include/ivp (the render module aside) the script
collects the names the header declares: functions, types, macros and enum
constants. A header counts as used by an example when the example's sources
(examples/example_<name>.c and the shared examples/*.h) mention one of them.

    tools/example_coverage.py            print the table
    tools/example_coverage.py --write    rewrite examples/COVERAGE.md
    tools/example_coverage.py --check    exit 1 if examples/COVERAGE.md is stale

The shared framework headers (demo_app.h and friends) use many engine headers
on behalf of every example, so the script reads the example's own file, plus the
helper headers that bring one feature with them (demo_terrain.h, demo_car.h).
"""
import argparse
import glob
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
INCLUDE = os.path.join(ROOT, "include", "ivp")
EXAMPLES = os.path.join(ROOT, "examples")
OUT = os.path.join(EXAMPLES, "COVERAGE.md")

# Modules whose headers are building blocks of the engine, not features an
# example would show.
INTERNAL = {
    "base": "allocator, arena, assert and message plumbing",
    "containers": "arrays, hashes, sets and heaps",
    "solver": "impact, contact and friction solvers (reached through any collision)",
    "math": "vectors, matrices, quaternions",
    "diagnostics": "statistics and debug managers",
}

# Headers an example reaches only through another API: (examples, how).
INDIRECT = {
    "builder/ivp_build_compact_recursive.h": (["cut", "ring", "workshop"], "inside ivp_ledge_soup_compile"),
    "controllers/ivp_constraint_car.h": (["wheelcar"], "the solver behind ivp_car_system"),
    "dynamics/ivp_merge_core.h": (["compound"], "behind ivp_object_attach_object"),
}

# Headers no example calls, and why that is fine.
NOTES = {
    "collision/ivp_collision_core.h": "the core data of the collision detection",
    "collision/ivp_compact_ledge_solver.h": "ledge against ledge queries behind the mindists",
    "collision/ivp_hull_mgr.h": "hull managers, the engine's view of an object",
    "collision/ivp_narrowphase.h": "the narrow phase behind the mindists",
    "controllers/ivp_actuator_extra.h": "the reference's demo actuator (a puck force that is zero, a float camera)",
    "controllers/ivp_controller_util.h": "helpers for writing the built-in controllers",
    "dynamics/ivp_anchor.h": "anchors of actuators, reached through the templates",
    "dynamics/ivp_cache_object.h": "cached object transforms",
    "dynamics/ivp_sim_unit.h": "simulation units, merged by the controllers' discovery",
    "dynamics/ivp_time_manager.h": "the event queue behind the PSI loop",
    "geometry/ivp_compact_ledge_geom.h": "reading ledge geometry: for tools",
    "geometry/ivp_compact_triangle.h": "reading triangles: for tools",
    "geometry/ivp_ledgetree.h": "the bounding volume hierarchy: for tools",
    "utility/ivp_hash.h": "hash helpers",
    "utility/ivp_multidimensional_interp.h": "buoyancy interpolation (compiled out like the reference)",
    "utility/ivp_string.h": "string helpers",
    "utility/ivp_union_find.h": "a helper of the builders",
}


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def names_in(text):
    """Names the header declares (function bodies of static inline helpers are
    not scanned for calls: a declaration starts a line with a return type)."""
    text = strip_comments(text)
    funcs = set()
    for m in re.finditer(r"(?m)^[ \t]*([A-Za-z_][\w \t\*]*?[ \t\*])(ivp_\w+)\s*\(", text):
        if "return" in m.group(1).split() or "=" in m.group(1):
            continue
        funcs.add(m.group(2))
    types = set(re.findall(r"\}\s*(ivp_\w+)\s*;", text))
    types |= set(re.findall(r"typedef\s+[^;{}]*?\b(ivp_\w+)\s*;", text))
    types |= set(re.findall(r"typedef\s+[^;{}]*?\(\s*\*\s*(ivp_\w+)\s*\)", text))
    macros = set(re.findall(r"#\s*define\s+(IVP_\w+)", text))
    consts = set(re.findall(r"(?m)^\s*(IVP_[A-Z0-9_]+)\s*(?:=|,|$)", text))
    names = funcs | types | macros | consts
    return {n for n in names if not n.endswith("_H")}


def tokens(text):
    return set(re.findall(r"\b(?:ivp|IVP)_\w+", text))


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def collect():
    # helper headers that bring one feature with them (the framework headers do
    # not: they use many engine headers on behalf of every example)
    helpers = {"demo_terrain.h": tokens(read(os.path.join(EXAMPLES, "demo_terrain.h"))),
               "demo_car.h": tokens(read(os.path.join(EXAMPLES, "demo_car.h")))}
    examples = {}
    for path in sorted(glob.glob(os.path.join(EXAMPLES, "example_*.c"))):
        name = os.path.basename(path)[len("example_"):-2]
        if name == "gallery":
            continue
        text = read(path)
        toks = tokens(text)
        for inc in re.findall(r'#include\s+"([^"]+)"', text):
            toks |= helpers.get(inc, set())
        examples[name] = toks
    headers = {}
    for path in sorted(glob.glob(os.path.join(INCLUDE, "**", "*.h"), recursive=True)):
        rel = os.path.relpath(path, INCLUDE).replace(os.sep, "/")
        if rel.startswith("render/"):
            continue
        headers[rel] = names_in(read(path))
    owners = {}
    for rel, names in headers.items():
        for n in names:
            owners.setdefault(n, set()).add(rel)
    # a name declared by several headers (a forward declaration, a repeated
    # typedef) belongs to the header whose file name it starts with, if any
    for rel in headers:
        stem = os.path.splitext(os.path.basename(rel))[0]
        keep = set()
        for n in headers[rel]:
            if len(owners[n]) == 1:
                keep.add(n)
                continue
            prefixed = [o for o in owners[n] if n.lower().startswith(os.path.splitext(os.path.basename(o))[0])]
            if prefixed and max(prefixed, key=lambda o: len(os.path.basename(o))) == rel:
                keep.add(n)
        headers[rel] = keep
    return headers, examples


def build():
    headers, examples = collect()
    rows = {}
    for rel, names in headers.items():
        if not names:
            continue
        users = [e for e, toks in examples.items() if names & toks]
        rows[rel] = (len(names & set().union(*examples.values())), len(names), users)
        if not users and rel in INDIRECT:
            rows[rel] = (0, len(names), [])
    return rows, examples


def render(rows, examples):
    out = []
    out.append("# Example coverage\n")
    out.append("<!-- generated by tools/example_coverage.py --write; do not edit -->\n")
    out.append("Which public headers the examples use. A header counts as used by an example "
               "when the example's own source mentions a function, type, macro or enum "
               "constant the header declares. `used/names` is the share of the header's "
               "names that appear in any example.\n")
    by_module = {}
    for rel, row in rows.items():
        by_module.setdefault(rel.split("/")[0], []).append((rel, row))

    feature_mods = [m for m in sorted(by_module) if m not in INTERNAL]
    total = sum(1 for m in feature_mods for _ in by_module[m])
    used = sum(1 for m in feature_mods for rel, r in by_module[m] if r[2] or rel in INDIRECT)
    out.append(f"Feature modules: {used} of {total} headers are used by at least one example; "
               f"the {total - used} others are internals (see the notes).\n")

    for mod in feature_mods:
        out.append(f"## {mod}\n")
        out.append("| header | used/names | examples |")
        out.append("|---|---|---|")
        for rel, (u, n, users) in by_module[mod]:
            who = ", ".join(f"`{e}`" for e in users) if users else "-"
            if not users and rel in INDIRECT:
                names, how = INDIRECT[rel]
                who = ", ".join(f"`{e}`" for e in names) + f" ({how})"
            elif not users and rel in NOTES:
                who = f"- ({NOTES[rel]})"
            out.append(f"| `{rel[len(mod) + 1:]}` | {u}/{n} | {who} |")
        out.append("")

    out.append("## Not shown by an example, on purpose\n")
    for mod, why in INTERNAL.items():
        if mod in by_module:
            n = len(by_module[mod])
            u = sum(1 for _, r in by_module[mod] if r[2])
            out.append(f"- `{mod}` ({n} headers, {u} touched): {why}")
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="rewrite examples/COVERAGE.md")
    ap.add_argument("--check", action="store_true", help="fail if examples/COVERAGE.md is stale")
    args = ap.parse_args()
    rows, examples = build()
    text = render(rows, examples)
    if args.write:
        with open(OUT, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"wrote {os.path.relpath(OUT, ROOT)}")
    elif args.check:
        old = read(OUT) if os.path.exists(OUT) else ""
        if old != text:
            print("examples/COVERAGE.md is stale: run tools/example_coverage.py --write", file=sys.stderr)
            return 1
        print("examples/COVERAGE.md is up to date")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
