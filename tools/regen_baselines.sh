#!/bin/bash
# Regenerate libivp scenario outputs and the drift baselines
# (tools/baselines/<scenario>.json = compare_jsonl metrics of libivp against
# the legacy reference).  The references themselves
# (tools/testdata/ref_<scenario>.jsonl) come from the legacy C++ engine:
# regenerate them with tools/regen_reference.sh, never from libivp.
#
#   tools/regen_baselines.sh [build dir with test_scenarios]   (default: build)
set -e
cd "$(dirname "$0")/.."
BUILD="${1:-build}"

SCENARIOS=$(grep -v '^#' tools/scenarios.txt | awk 'NF {print $1}')

mkdir -p "$BUILD/scenario_output" tools/baselines
for s in $SCENARIOS; do
  if [ ! -f "tools/testdata/ref_${s}.jsonl" ]; then
    echo "missing tools/testdata/ref_${s}.jsonl: run tools/regen_reference.sh first" >&2
    exit 1
  fi
done

echo "=== Generating scenario outputs ==="
grep -v '^#' tools/scenarios.txt | while read -r s dt steps; do
  [ -n "$s" ] || continue
  "$BUILD/test_scenarios" --scenario "$s" --steps "$steps" --dt "$dt" --output "$BUILD/scenario_output/${s}.jsonl"
done

echo "=== Generating baselines ==="
for s in $SCENARIOS; do
  python3 tools/compare_jsonl.py \
    tools/testdata/ref_${s}.jsonl \
    "$BUILD/scenario_output"/${s}.jsonl \
    --adaptive \
    --save-metrics "$BUILD/scenario_output"/metrics_${s}.json > /dev/null 2>&1 || true
  cp "$BUILD/scenario_output"/metrics_${s}.json tools/baselines/${s}.json
done

echo "=== Done ==="
