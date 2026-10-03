#!/bin/bash
# Full parity check of libivp against the reference engine.
#
#   tools/parity/check.sh [--release] [--debug] [--sweep "dt:steps ..."]
#
# Without options: --release --debug.
#   --release  reference built with -ffp-contract=off (NDEBUG) vs a libivp
#              Release build: JSONL, non-JSON stdout and stderr must match
#   --debug    reference built with -DDEBUG and without NDEBUG vs a libivp
#              Debug build: JSONL, debug stdout text and the sequence of
#              assert reports must match
#   --sweep    extra release runs of every scenario at the given step sizes,
#              e.g. --sweep "0.016666666666666666:900 0.0125:1200"
#              (JSONL only)
#
# Builds go to $IVP_PARITY_DIR (default build/parity); the reference sources
# are taken from $IVP_REF_SRC (default ../ivp).  Exit code 1 if any
# comparison differs.  The scenario list is tools/scenarios.txt
# ($SCENARIOS restricts it).
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/common.sh"
DO_REL=0; DO_DBG=0; SWEEP=""
while [ $# -gt 0 ]; do
  case "$1" in
    --release) DO_REL=1 ;; --debug) DO_DBG=1 ;;
    --sweep) shift; SWEEP="${1:-}"; DO_REL=1 ;;
    *) parity_die "unknown option $1" ;;
  esac
  shift
done
[ $DO_REL = 0 ] && [ $DO_DBG = 0 ] && { DO_REL=1; DO_DBG=1; }
parity_check_ref_src
FAIL=0

build_lib() { # <dir> <build type>
  mkdir -p "$1"
  cmake -B "$1" -S "$PARITY_REPO" -DCMAKE_BUILD_TYPE="$2" -DIVP_ENABLE_SCENARIO_TESTS=ON > "$1.cmake.log" &&
    cmake --build "$1" --target test_scenarios -j"$IVP_JOBS" > "$1.build.log" ||
    parity_die "libivp $2 build failed (see $1.build.log)"
}

if [ $DO_REL = 1 ]; then
  "$HERE/build_reference.sh" release > /dev/null || parity_die "reference release build failed"
  build_lib "$IVP_PARITY_DIR/lib_release" Release
  REF_RUNNER="$(parity_ref_dir release)/runner/legacy_runner"
  LIB_BIN="$IVP_PARITY_DIR/lib_release/test_scenarios"
  echo "=== release: defaults ==="
  rm -rf "$IVP_PARITY_DIR/out/ref_release" "$IVP_PARITY_DIR/out/lib_release"
  "$HERE/run_scenarios.sh" ref "$REF_RUNNER" "$IVP_PARITY_DIR/out/ref_release"
  "$HERE/run_scenarios.sh" lib "$LIB_BIN" "$IVP_PARITY_DIR/out/lib_release"
  "$HERE/compare.sh" --text --stderr "$IVP_PARITY_DIR/out/ref_release" "$IVP_PARITY_DIR/out/lib_release" > "$IVP_PARITY_DIR/out/release.txt"; r=$?
  tail -1 "$IVP_PARITY_DIR/out/release.txt"; [ $r = 0 ] || { FAIL=1; grep -v "jsonl:OK.*text:OK.*stderr:OK" "$IVP_PARITY_DIR/out/release.txt" | head -5; }
  for spec in $SWEEP; do
    dt="${spec%%:*}"; steps="${spec##*:}"
    echo "=== release: dt=$dt steps=$steps ==="
    o="$IVP_PARITY_DIR/out/sweep_${dt}_${steps}"; rm -rf "$o"
    "$HERE/run_scenarios.sh" ref "$REF_RUNNER" "$o/ref" "$dt" "$steps"
    "$HERE/run_scenarios.sh" lib "$LIB_BIN" "$o/lib" "$dt" "$steps"
    "$HERE/compare.sh" "$o/ref" "$o/lib" > "$o.txt"; r=$?
    tail -1 "$o.txt"; [ $r = 0 ] || { FAIL=1; grep -v "jsonl:OK" "$o.txt" | head -5; }
  done
fi

if [ $DO_DBG = 1 ]; then
  "$HERE/build_reference.sh" debug > /dev/null || parity_die "reference debug build failed"
  build_lib "$IVP_PARITY_DIR/lib_debug" Debug
  echo "=== debug: defaults ==="
  rm -rf "$IVP_PARITY_DIR/out/ref_debug" "$IVP_PARITY_DIR/out/lib_debug"
  "$HERE/run_scenarios.sh" ref "$(parity_ref_dir debug)/runner/legacy_runner" "$IVP_PARITY_DIR/out/ref_debug"
  "$HERE/run_scenarios.sh" lib "$IVP_PARITY_DIR/lib_debug/test_scenarios" "$IVP_PARITY_DIR/out/lib_debug"
  "$HERE/compare.sh" --text --asserts "$IVP_PARITY_DIR/out/ref_debug" "$IVP_PARITY_DIR/out/lib_debug" > "$IVP_PARITY_DIR/out/debug.txt"; r=$?
  tail -1 "$IVP_PARITY_DIR/out/debug.txt"; [ $r = 0 ] || { FAIL=1; grep -v "text:OK.*asserts:OK" "$IVP_PARITY_DIR/out/debug.txt" | head -5; }
fi

[ $FAIL = 0 ] && echo "PARITY OK" || echo "PARITY FAILED (details in $IVP_PARITY_DIR/out/*.txt)"
exit $FAIL
