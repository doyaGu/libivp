#!/bin/bash
# Optional C++ counterpart for native hull/LCS workloads; reference is read-only.
set -euo pipefail
source "$(dirname "$0")/parity/common.sh"
parity_check_ref_src
OUT="${1:-$PARITY_REPO/build/benchmark/native-reference}"
LIBDIR="${IVP_REF_LIB_DIR:-$PARITY_REPO/build/benchmark/reference/ref_release/libs}"
mkdir -p "$OUT"
# Always configure/build: an existing archive may be from another reference SHA.
cmake -B "$LIBDIR" -S "$IVP_REF_SRC" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-ffp-contract=off" -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
  $PARITY_REF_CMAKE_OPTIONS -DIVP_BUILD_EXAMPLES=OFF -DIVP_BUILD_TESTS=OFF > "$OUT/cmake.log" 2>&1
cmake --build "$LIBDIR" -j"$IVP_JOBS" > "$OUT/build.log" 2>&1
# shellcheck disable=SC2046
c++ -std=c++17 -O3 -DNDEBUG -ffp-contract=off -fno-fast-math -frounding-math \
  -DIVP_VERSION_SDK -DIVP_ENABLE_QHULL $PARITY_REF_DEFS -w $(parity_ref_includes "$LIBDIR") \
  "$PARITY_REPO/benchmarks/ivp_native_reference.cpp" $(parity_ref_libs "$LIBDIR") \
  -o "$OUT/ivp_native_reference"
echo "$OUT/ivp_native_reference"
