#!/bin/bash
# Full parity check of both libivp reproducibility domains against the
# reference engine.
#
#   tools/parity/check.sh [--release] [--debug] [--sweep "dt:steps ..."]
#
# Without options: --release --debug.
#   --release  reference built with -ffp-contract=off (NDEBUG) vs Release
#              builds of both libivp policies
#   --debug    reference built with -DDEBUG and without NDEBUG vs Debug
#              builds of both libivp policies
#   --sweep    extra release runs of every scenario at the given step sizes,
#              e.g. --sweep "0.016666666666666666:900 0.0125:1200"
#              (JSONL only)
#
# Builds go to $IVP_PARITY_DIR (default build/parity); the reference sources
# are taken from $IVP_REF_SRC (default ../ivp).  Exit code 1 if any
# comparison violates its policy.  The reference-development build must be
# 48/48 identical.  The mainline build must differ in exactly the scenarios
# listed by mainline_expected_differences.txt.  The scenario list is
# tools/scenarios.txt ($SCENARIOS restricts it).
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
EXPECTED_MAINLINE="$HERE/mainline_expected_differences.txt"

build_lib() { # <dir> <build type> <canonical state> <qhull mode>
  mkdir -p "$1"
  cmake -B "$1" -S "$PARITY_REPO" -DCMAKE_BUILD_TYPE="$2" \
    -DIVP_ENABLE_SCENARIO_TESTS=ON \
    -DIVP_CANONICAL_INTERNAL_STATE="$3" \
    -DIVP_QHULL_MODE="$4" > "$1.cmake.log" &&
    cmake --build "$1" --target test_scenarios -j"$IVP_JOBS" > "$1.build.log" ||
    parity_die "libivp $2/$4 build failed (see $1.build.log)"
}

report_comparison() { # <result file> <compare exit code>
  tail -1 "$1"
  if [ "$2" != 0 ]; then
    FAIL=1
    grep -E 'status:(UNEXPECTED|MISSING)' "$1" | head -5
  fi
}

if [ $DO_REL = 1 ]; then
  "$HERE/build_reference.sh" release > /dev/null || parity_die "reference release build failed"
  build_lib "$IVP_PARITY_DIR/lib_reference_release" Release OFF REFERENCE
  build_lib "$IVP_PARITY_DIR/lib_mainline_release" Release ON REENTRANT
  REF_RUNNER="$(parity_ref_dir release)/runner/legacy_runner"
  echo "=== release: reference-development policy ==="
  rm -rf "$IVP_PARITY_DIR/out/ref_release" \
         "$IVP_PARITY_DIR/out/lib_reference_release" \
         "$IVP_PARITY_DIR/out/lib_mainline_release"
  "$HERE/run_scenarios.sh" ref "$REF_RUNNER" "$IVP_PARITY_DIR/out/ref_release"
  "$HERE/run_scenarios.sh" lib \
    "$IVP_PARITY_DIR/lib_reference_release/test_scenarios" \
    "$IVP_PARITY_DIR/out/lib_reference_release"
  "$HERE/compare.sh" --text --stderr \
    "$IVP_PARITY_DIR/out/ref_release" \
    "$IVP_PARITY_DIR/out/lib_reference_release" \
    > "$IVP_PARITY_DIR/out/release_reference.txt"; r=$?
  report_comparison "$IVP_PARITY_DIR/out/release_reference.txt" "$r"

  echo "=== release: mainline policy ==="
  "$HERE/run_scenarios.sh" lib \
    "$IVP_PARITY_DIR/lib_mainline_release/test_scenarios" \
    "$IVP_PARITY_DIR/out/lib_mainline_release"
  "$HERE/compare.sh" --text --stderr \
    --expect-differences "$EXPECTED_MAINLINE" \
    "$IVP_PARITY_DIR/out/ref_release" \
    "$IVP_PARITY_DIR/out/lib_mainline_release" \
    > "$IVP_PARITY_DIR/out/release_mainline.txt"; r=$?
  report_comparison "$IVP_PARITY_DIR/out/release_mainline.txt" "$r"

  for spec in $SWEEP; do
    dt="${spec%%:*}"; steps="${spec##*:}"
    echo "=== release: dt=$dt steps=$steps ==="
    o="$IVP_PARITY_DIR/out/sweep_${dt}_${steps}"; rm -rf "$o"
    "$HERE/run_scenarios.sh" ref "$REF_RUNNER" "$o/ref" "$dt" "$steps"
    "$HERE/run_scenarios.sh" lib \
      "$IVP_PARITY_DIR/lib_reference_release/test_scenarios" \
      "$o/reference" "$dt" "$steps"
    "$HERE/run_scenarios.sh" lib \
      "$IVP_PARITY_DIR/lib_mainline_release/test_scenarios" \
      "$o/mainline" "$dt" "$steps"
    "$HERE/compare.sh" "$o/ref" "$o/reference" > "$o/reference.txt"; r=$?
    report_comparison "$o/reference.txt" "$r"
    "$HERE/compare.sh" --allow-differences "$EXPECTED_MAINLINE" \
      "$o/ref" "$o/mainline" > "$o/mainline.txt"; r=$?
    report_comparison "$o/mainline.txt" "$r"
  done
fi

if [ $DO_DBG = 1 ]; then
  "$HERE/build_reference.sh" debug > /dev/null || parity_die "reference debug build failed"
  build_lib "$IVP_PARITY_DIR/lib_reference_debug" Debug OFF REFERENCE
  build_lib "$IVP_PARITY_DIR/lib_mainline_debug" Debug ON REENTRANT
  echo "=== debug: reference-development policy ==="
  rm -rf "$IVP_PARITY_DIR/out/ref_debug" \
         "$IVP_PARITY_DIR/out/lib_reference_debug" \
         "$IVP_PARITY_DIR/out/lib_mainline_debug"
  "$HERE/run_scenarios.sh" ref "$(parity_ref_dir debug)/runner/legacy_runner" "$IVP_PARITY_DIR/out/ref_debug"
  "$HERE/run_scenarios.sh" lib \
    "$IVP_PARITY_DIR/lib_reference_debug/test_scenarios" \
    "$IVP_PARITY_DIR/out/lib_reference_debug"
  "$HERE/compare.sh" --text --asserts \
    "$IVP_PARITY_DIR/out/ref_debug" \
    "$IVP_PARITY_DIR/out/lib_reference_debug" \
    > "$IVP_PARITY_DIR/out/debug_reference.txt"; r=$?
  report_comparison "$IVP_PARITY_DIR/out/debug_reference.txt" "$r"

  echo "=== debug: mainline policy ==="
  "$HERE/run_scenarios.sh" lib \
    "$IVP_PARITY_DIR/lib_mainline_debug/test_scenarios" \
    "$IVP_PARITY_DIR/out/lib_mainline_debug"
  "$HERE/compare.sh" --text --asserts \
    --expect-differences "$EXPECTED_MAINLINE" \
    "$IVP_PARITY_DIR/out/ref_debug" \
    "$IVP_PARITY_DIR/out/lib_mainline_debug" \
    > "$IVP_PARITY_DIR/out/debug_mainline.txt"; r=$?
  report_comparison "$IVP_PARITY_DIR/out/debug_mainline.txt" "$r"
fi

[ $FAIL = 0 ] && echo "PARITY OK" || echo "PARITY FAILED (details in $IVP_PARITY_DIR/out/*.txt)"
exit $FAIL
