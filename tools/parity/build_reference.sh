#!/bin/bash
# Build the reference engine libraries and the reference scenario runner.
#
#   tools/parity/build_reference.sh [release|debug|fma]   (default: release)
#
#   release  -ffp-contract=off, NDEBUG: the bit-exact target of libivp Release
#   debug    -ffp-contract=off -DDEBUG -O3 without NDEBUG: the target of a
#            libivp Debug build (IVP_IF code, asserts, debug output)
#   fma      compiler default contraction, like the shipped arm64 libraries
#
# Output: $IVP_PARITY_DIR/ref_<flavor>/libs (CMake build) and
#         $IVP_PARITY_DIR/ref_<flavor>/runner/legacy_runner
# The runner must never sit next to a libivp test_scenarios binary: it then
# delegates to it (main.cxx run_c17_test_scenarios).
set -euo pipefail
source "$(dirname "$0")/common.sh"
FLAVOR="${1:-release}"
parity_check_ref_src

case "$FLAVOR" in
  release) CXXFLAGS="-ffp-contract=off"; CXXFLAGS_RELEASE="-O3 -DNDEBUG"; RUNNER_DEFS="" ;;
  debug)   CXXFLAGS="-ffp-contract=off -DDEBUG"; CXXFLAGS_RELEASE="-O3"; RUNNER_DEFS="-DDEBUG" ;;
  fma)     CXXFLAGS=""; CXXFLAGS_RELEASE="-O3 -DNDEBUG"; RUNNER_DEFS="" ;;
  *) parity_die "unknown flavor '$FLAVOR' (release|debug|fma)" ;;
esac

OUT="$(parity_ref_dir "$FLAVOR")"
LIBDIR="$OUT/libs"
mkdir -p "$OUT"
echo "=== reference $FLAVOR libraries ($LIBDIR) ==="
cmake -B "$LIBDIR" -S "$IVP_REF_SRC" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="$CXXFLAGS" -DCMAKE_CXX_FLAGS_RELEASE="$CXXFLAGS_RELEASE" \
  -DIVP_BUILD_EXAMPLES=OFF -DIVP_BUILD_TESTS=OFF > "$OUT.cmake.log" 2>&1
cmake --build "$LIBDIR" -j"$IVP_JOBS" > "$OUT.build.log"

echo "=== reference $FLAVOR runner ==="
mkdir -p "$OUT/runner"
# shellcheck disable=SC2046
c++ -std=c++17 -O2 -ffp-contract=off -fno-fast-math -w $RUNNER_DEFS -DIVP_VERSION_SDK -DIVP_ENABLE_QHULL \
  $(parity_ref_includes "$LIBDIR") "$IVP_REF_SRC"/tools/ivp_reference_runner/*.cxx \
  $(parity_ref_libs "$LIBDIR") -o "$OUT/runner/legacy_runner"
echo "$OUT/runner/legacy_runner"
