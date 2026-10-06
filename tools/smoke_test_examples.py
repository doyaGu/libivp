#!/usr/bin/env python3
"""Run every built example headless and check its self-check result.

Each example_* binary under <build-dir>/examples is started with
`--headless`; it passes when it exits with 0 and prints
"self-check PASSED". No window or GPU is needed.
"""

import argparse
import glob
import os
import subprocess
import sys


def find_examples(build_dir):
    found = {}
    for sub in ("examples", os.path.join("examples", "Debug"),
                os.path.join("examples", "Release")):
        for path in glob.glob(os.path.join(build_dir, sub, "example_*")):
            name = os.path.basename(path)
            if name.endswith((".a", ".lib", ".pdb", ".dSYM")):
                continue
            if os.path.isfile(path) and os.access(path, os.X_OK):
                found.setdefault(name, path)
    return dict(sorted(found.items()))


def run_one(path, frames, timeout):
    env = dict(os.environ)
    env.pop("IVP_EXAMPLE_HEADLESS", None)
    try:
        proc = subprocess.run(
            [path, "--headless"] + ([str(frames)] if frames else []),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=timeout, env=env, cwd=os.path.dirname(path))
    except subprocess.TimeoutExpired as exc:
        return False, "timeout after %ss" % timeout, exc.stdout or ""
    out = proc.stdout or ""
    if proc.returncode != 0:
        return False, "exit code %d" % proc.returncode, out
    if "self-check PASSED" not in out:
        return False, "no 'self-check PASSED' in output", out
    return True, "ok", out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--frames", type=int, default=0,
                    help="frames to simulate (default: the example's own, 600)")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="seconds allowed per example")
    ap.add_argument("--only", nargs="*", default=None,
                    help="example names to run (default: all)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    examples = find_examples(os.path.abspath(args.build_dir))
    if args.only:
        examples = {k: v for k, v in examples.items() if k in args.only}
    if not examples:
        print("no example_* binaries found under %s" % args.build_dir)
        return 1

    failed = []
    for name, path in examples.items():
        ok, why, out = run_one(path, args.frames, args.timeout)
        print("%-4s %s%s" % ("ok" if ok else "FAIL", name,
                             "" if ok else "  (%s)" % why))
        if not ok or args.verbose:
            tail = "\n".join(out.strip().splitlines()[-15:])
            if tail:
                print("\n".join("     | " + l for l in tail.splitlines()))
        if not ok:
            failed.append(name)

    print("%d/%d examples passed" % (len(examples) - len(failed), len(examples)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
