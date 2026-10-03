#!/bin/bash
# Link a reference runner in which some reference objects are replaced by
# instrumented copies (e.g. a copy of ivp_mindist.cxx with fprintf(stderr,
# "%a") probes).  The overrides are compiled with the flags of the reference
# library objects, so everything else stays bit-identical.
#
#   tools/parity/build_override_runner.sh <release|debug> <out_dir> <override.cxx>...
#
# Needs tools/parity/build_reference.sh <flavor> first.  Result:
# <out_dir>/legacy_runner (never put it next to a test_scenarios binary).
set -euo pipefail
source "$(dirname "$0")/common.sh"
[ $# -ge 3 ] || parity_die "usage: $0 <release|debug> <out_dir> <override.cxx>..."
FLAVOR="$1"; OUT="$2"; shift 2
parity_check_ref_src
LIBDIR="$(parity_ref_dir "$FLAVOR")/libs"
[ -f "$LIBDIR/libivp_physics.a" ] || parity_die "build the reference first: tools/parity/build_reference.sh $FLAVOR"
case "$FLAVOR" in
  release) FLAGS="$PARITY_OVERRIDE_FLAGS_RELEASE"; RUNNER_DEFS="" ;;
  debug)   FLAGS="$PARITY_OVERRIDE_FLAGS_DEBUG"; RUNNER_DEFS="-DDEBUG" ;;
  *) parity_die "unknown flavor '$FLAVOR' (release|debug)" ;;
esac
INC="$(parity_ref_includes "$LIBDIR")"
mkdir -p "$OUT/obj"
OBJS=""
for f in "$@"; do
  o="$OUT/obj/$(basename "$f" .cxx).o"
  # shellcheck disable=SC2086
  c++ -c $FLAGS $INC "$f" -o "$o"
  OBJS="$OBJS $o"
done
# shellcheck disable=SC2086
c++ -std=c++17 -O2 -ffp-contract=off -fno-fast-math -w $RUNNER_DEFS -DIVP_VERSION_SDK -DIVP_ENABLE_QHULL $INC \
  "$IVP_REF_SRC"/tools/ivp_reference_runner/*.cxx $OBJS $(parity_ref_libs "$LIBDIR") -o "$OUT/legacy_runner"
echo "$OUT/legacy_runner"
