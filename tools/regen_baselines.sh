#!/bin/bash
# Regenerate all golden references, scenario outputs, and baselines.
set -e
cd "$(dirname "$0")/.."

DT60="0.016666666666666666"
DT180="0.005555555555555556"

SCENARIOS_60="freefall two_balls slope_friction"
SCENARIOS_180="springs rope motor buoyancy cubes force_actuator forcefield stiff_spring check_distance collision_filter motion_controller phantom vehicle"

mkdir -p build/scenario_output

echo "=== Generating golden references ==="
for s in $SCENARIOS_60; do
  ./build/test_scenarios --scenario $s --steps 240 --dt $DT60 --output tools/testdata/ref_${s}.jsonl
done
for s in $SCENARIOS_180; do
  ./build/test_scenarios --scenario $s --steps 480 --dt $DT180 --output tools/testdata/ref_${s}.jsonl
done

echo "=== Generating scenario outputs ==="
for s in $SCENARIOS_60; do
  ./build/test_scenarios --scenario $s --steps 240 --dt $DT60 --output build/scenario_output/${s}.jsonl
done
for s in $SCENARIOS_180; do
  ./build/test_scenarios --scenario $s --steps 480 --dt $DT180 --output build/scenario_output/${s}.jsonl
done

echo "=== Generating baselines ==="
for s in $SCENARIOS_60 $SCENARIOS_180; do
  python3 tools/compare_jsonl.py \
    tools/testdata/ref_${s}.jsonl \
    build/scenario_output/${s}.jsonl \
    --adaptive \
    --save-metrics build/scenario_output/metrics_${s}.json 2>/dev/null
  cp build/scenario_output/metrics_${s}.json tools/baselines/${s}.json
done

echo "=== Done ==="
