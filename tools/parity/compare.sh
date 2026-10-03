#!/bin/bash
# Compare the outputs of tools/parity/run_scenarios.sh, scenario by scenario.
#
#   tools/parity/compare.sh [--text] [--stderr] [--asserts] <ref_dir> <lib_dir>
#
#   (always)   JSONL frames bit for bit (cmp_jsonl.py)
#   --text     the non-JSON stdout lines (engine messages, debug output)
#   --stderr   the whole stderr (release runs: engine error output)
#   --asserts  the sequence of "ASSERTION FAILURE: <expr>" reports in stderr
#              (debug runs; file and line differ by nature, the expression
#              and the order must match)
#   and the exit codes of both runs.
#
# Prints one line per scenario and a summary; exit code 1 if anything differs.
set -uo pipefail
source "$(dirname "$0")/common.sh"
TEXT=0; STDERR=0; ASSERTS=0
while [ $# -gt 0 ]; do
  case "$1" in
    --text) TEXT=1 ;; --stderr) STDERR=1 ;; --asserts) ASSERTS=1 ;;
    -*) parity_die "unknown option $1" ;;
    *) break ;;
  esac
  shift
done
[ $# -eq 2 ] || parity_die "usage: $0 [--text] [--stderr] [--asserts] <ref_dir> <lib_dir>"
REF="$1"; LIB="$2"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
total=0; bad=0
while read -r s _dt _n; do
  total=$((total + 1)); line="$s"; ok=1
  r=$(python3 "$(dirname "$0")/cmp_jsonl.py" "$REF/$s.jsonl" "$LIB/$s.jsonl" 2>&1) || ok=0
  line="$line jsonl:${r}"
  if [ "$(cat "$REF/$s.rc" 2>/dev/null)" != "$(cat "$LIB/$s.rc" 2>/dev/null)" ]; then
    ok=0; line="$line rc:$(cat "$REF/$s.rc" 2>/dev/null)/$(cat "$LIB/$s.rc" 2>/dev/null)"
  fi
  if [ $TEXT = 1 ]; then
    grep -v '^{' "$REF/$s.out" > "$TMP/r" 2>/dev/null; grep -v '^{' "$LIB/$s.out" > "$TMP/l" 2>/dev/null
    if cmp -s "$TMP/r" "$TMP/l"; then line="$line text:OK($(wc -l < "$TMP/r" | tr -d ' '))"
    else ok=0; line="$line text:DIFF@$(diff "$TMP/r" "$TMP/l" | head -1)"; fi
  fi
  if [ $STDERR = 1 ]; then
    if cmp -s "$REF/$s.err" "$LIB/$s.err"; then line="$line stderr:OK"
    else ok=0; line="$line stderr:DIFF@$(diff "$REF/$s.err" "$LIB/$s.err" | head -1)"; fi
  fi
  if [ $ASSERTS = 1 ]; then
    grep '^ASSERTION FAILURE: ' "$REF/$s.err" > "$TMP/r" 2>/dev/null; grep '^ASSERTION FAILURE: ' "$LIB/$s.err" > "$TMP/l" 2>/dev/null
    if cmp -s "$TMP/r" "$TMP/l"; then line="$line asserts:OK($(wc -l < "$TMP/r" | tr -d ' '))"
    else ok=0; line="$line asserts:DIFF@$(diff "$TMP/r" "$TMP/l" | head -1)"; fi
  fi
  [ $ok = 1 ] || bad=$((bad + 1))
  echo "$line"
done < <(parity_scenarios)
echo "=== $((total - bad))/$total identical ==="
[ $bad = 0 ]
