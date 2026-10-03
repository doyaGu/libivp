#!/bin/bash
# Run every scenario of tools/scenarios.txt (or $SCENARIOS) with one binary.
#
#   tools/parity/run_scenarios.sh <ref|lib> <binary> <out_dir> [dt steps]
#
#   ref  a reference runner (legacy_runner); SIGINT is ignored so that failing
#        reference asserts print and continue, like a debugger-less run
#   lib  libivp's test_scenarios; run with --assert-continue, which prints
#        failed asserts in the reference format and continues
#   dt steps  override the step size and count of every scenario
#
# Writes <out_dir>/<scenario>.{jsonl,out,err,rc}: jsonl = the JSON frames,
# out = full stdout, err = stderr, rc = exit code.
set -euo pipefail
source "$(dirname "$0")/common.sh"
[ $# -ge 3 ] || parity_die "usage: $0 <ref|lib> <binary> <out_dir> [dt steps]"
KIND="$1"; BIN="$2"; OUT="$3"; DT="${4:-}"; STEPS="${5:-}"
[ -x "$BIN" ] || parity_die "not executable: $BIN"
[ "$KIND" = ref ] || [ "$KIND" = lib ] || parity_die "kind must be ref or lib"
mkdir -p "$OUT"
export PARITY_KIND="$KIND" PARITY_BIN="$BIN" PARITY_OUT="$OUT"
parity_scenarios | while read -r s dt n; do
  echo "$s ${DT:-$dt} ${STEPS:-$n}"
done | xargs -P "$IVP_JOBS" -L 1 bash -c '
  trap "" INT
  s=$0; dt=$1; n=$2; o="$PARITY_OUT/$s"
  if [ "$PARITY_KIND" = lib ]; then
    "$PARITY_BIN" --scenario "$s" --steps "$n" --dt "$dt" --output "$o.jsonl" --assert-continue > "$o.out" 2> "$o.err"
    echo $? > "$o.rc"
  else
    "$PARITY_BIN" --scenario "$s" --steps "$n" --dt "$dt" > "$o.out" 2> "$o.err"
    echo $? > "$o.rc"
    grep "^{" "$o.out" > "$o.jsonl" || true
  fi'
