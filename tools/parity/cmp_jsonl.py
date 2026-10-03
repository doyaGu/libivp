#!/usr/bin/env python3
"""Exact comparison of two scenario JSONL files.

usage: cmp_jsonl.py <reference.jsonl> <candidate.jsonl>

Prints "OK" or "DIFF" with the frame counts, the first frame in which pos,
quat, lin_vel or ang_vel of any object differs (bit for bit, as printed) and
the largest position difference.  Exit code 0 = identical, 1 = different.
Non-JSON lines are ignored.
"""
import json
import math
import sys


def load(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.lstrip().startswith('{')]


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    first, max_pos = None, 0.0
    for i in range(min(len(a), len(b))):
        oa, ob = a[i]['objects'], b[i]['objects']
        if len(oa) != len(ob) and first is None:
            first = i
        for x, y in zip(oa, ob):
            if first is None and any(x[k] != y[k] for k in ('pos', 'quat', 'lin_vel', 'ang_vel')):
                first = i
            d = math.sqrt(sum((x['pos'][k] - y['pos'][k]) ** 2 for k in 'xyz'))
            max_pos = max(max_pos, d if d == d else float('inf'))
    ok = first is None and len(a) == len(b) and len(a) > 0
    print(f"{'OK  ' if ok else 'DIFF'} frames={len(a)}/{len(b)} first_diff={first} max_pos={max_pos:.3g}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
