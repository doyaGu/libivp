#!/bin/bash
# Build the legacy C++ IVP reference runner and regenerate the golden
# reference JSONL files (tools/testdata/ref_<scenario>.jsonl) from it.
#
#   tools/regen_reference.sh [path/to/ivp] [legacy build dir]
#
# The reference engine (default: ../ivp next to this repository) is built in
# Release with CMake and -ffp-contract=off (the same strict FP rules as
# libivp; libivp reproduces this build bit for bit), with the reentrant
# Qhull 2020.2 backend and without the reference's software x87 emulation.  Set IVP_REF_FMA=1 to
# build it with the compiler's default contraction instead, like the shipped
# arm64 libraries (FMA: differs by ~1e-6 and chaotically in some scenes).
# tools/ivp_reference_runner/*.cxx of the reference tree is compiled against
# its static libraries (libraries listed twice, see DEVELOPMENT.md of the
# runner).  The runner prints the same JSONL format as libivp's
# test_scenarios, so tools/compare_jsonl.py can compare them.  The scenario
# list (name, dt, steps) is tools/scenarios.txt.
set -euo pipefail
cd "$(dirname "$0")/.."
REF_SRC="${1:-$(cd .. && pwd)/ivp}"
LEGACY_BUILD="${2:-build/legacy_ivp}"
RUNNER="$LEGACY_BUILD/legacy_runner"

if [ ! -d "$REF_SRC/tools/ivp_reference_runner" ]; then
  echo "reference source not found: $REF_SRC" >&2
  exit 1
fi

echo "=== Building legacy IVP libraries ($LEGACY_BUILD) ==="
REF_CXX_FLAGS="-ffp-contract=off"
if [ "${IVP_REF_FMA:-0}" = "1" ]; then REF_CXX_FLAGS=""; fi
cmake -B "$LEGACY_BUILD" -S "$REF_SRC" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="$REF_CXX_FLAGS" \
  -DIVP_REENTRANT_QHULL=ON -DIVP_ENABLE_X87_EMULATION=OFF \
  -DIVP_X87_ARITHMETIC=OFF -DIVP_X87_ARITHMETIC_TRANSCENDENTAL=OFF \
  -DIVP_BUILD_EXAMPLES=OFF -DIVP_BUILD_TESTS=OFF > /dev/null
cmake --build "$LEGACY_BUILD" -j8 > /dev/null

echo "=== Building reference runner ==="
LIBS=("$LEGACY_BUILD/libivp_physics.a" "$LEGACY_BUILD/libivp_compact_builder.a"
      "$LEGACY_BUILD/libivp_collision.a" "$LEGACY_BUILD/libivp_surface_manager.a"
      "$LEGACY_BUILD/libivp_controller.a" "$LEGACY_BUILD/libivp_utility.a")
INC=(-I"$LEGACY_BUILD/include/ivp")
for d in ivp_compact_builder ivp_surface_manager ivp_collision ivp_physics ivp_utility ivp_controller ivp_intern; do
  INC+=(-I"$REF_SRC/$d")
done
c++ -std=c++17 -O2 -ffp-contract=off -fno-fast-math -w -DIVP_VERSION_SDK -DIVP_ENABLE_QHULL \
  -DIVP_ENABLE_X87_EMULATION=0 -DIVP_X87_ARITHMETIC=0 -DIVP_X87_ARITHMETIC_TRANSCENDENTAL=0 \
  "${INC[@]}" "$REF_SRC"/tools/ivp_reference_runner/*.cxx "${LIBS[@]}" "${LIBS[@]}" -o "$RUNNER"

echo "=== Generating reference JSONL ==="
mkdir -p tools/testdata
# The runner delegates to a libivp test_scenarios binary placed next to it
# (main.cxx run_c17_test_scenarios); make sure there is none.
if [ -e "$(dirname "$RUNNER")/test_scenarios" ] || [ -e "$(dirname "$RUNNER")/ivp-c17/test_scenarios" ]; then
  echo "refusing: a test_scenarios binary next to the runner would replace the reference output" >&2
  exit 1
fi
grep -v '^#' tools/scenarios.txt | while read -r s dt steps; do
  [ -n "$s" ] || continue
  # keep only the JSONL frames (the engine may print diagnostics to stdout)
  "$RUNNER" --scenario "$s" --steps "$steps" --dt "$dt" | grep '^{' > "tools/testdata/ref_${s}.jsonl"
done
echo "=== Done (run tools/regen_baselines.sh to refresh the drift baselines) ==="
