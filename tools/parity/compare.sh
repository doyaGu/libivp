#!/bin/bash
# Compare the outputs of tools/parity/run_scenarios.sh, scenario by scenario.
#
#   tools/parity/compare.sh [options] <ref_dir> <lib_dir>
#
#   (always)   JSONL frames bit for bit (cmp_jsonl.py)
#   --text     the non-JSON stdout lines (engine messages, debug output)
#   --stderr   the whole stderr (release runs: engine error output)
#   --asserts  the sequence of "ASSERTION FAILURE: <expr>" reports in stderr
#              (debug runs; file and line differ by nature, the expression
#              and the order must match)
#   --expect-differences <file>
#              require exactly the listed scenarios to have different JSONL;
#              missing artifacts and exit-code differences are never accepted
#   --allow-differences <file>
#              require every JSONL difference to be listed, but allow a listed
#              scenario to remain identical (useful for alternate step counts)
#   --expect-text-differences <file>
#              require exactly the listed scenarios to have different --text
#              output (debug messages whose order follows mainline internal
#              state); their JSONL is still compared on its own
#   and the exit codes of both runs.
#
# Prints one line per scenario and a summary.  Without an expectation file,
# exit code 1 means anything differs.  With one, it means the observed set of
# differing scenarios does not match the expectation file.
set -uo pipefail
source "$(dirname "$0")/common.sh"
TEXT=0; STDERR=0; ASSERTS=0; EXPECT_MODE=0; EXPECT_FILE=""; TEXT_EXPECT_FILE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --text) TEXT=1 ;; --stderr) STDERR=1 ;; --asserts) ASSERTS=1 ;;
    --expect-differences)
      shift
      [ $# -gt 0 ] || parity_die "--expect-differences needs a file"
      EXPECT_MODE=1; EXPECT_FILE="$1"
      ;;
    --allow-differences)
      shift
      [ $# -gt 0 ] || parity_die "--allow-differences needs a file"
      EXPECT_MODE=2; EXPECT_FILE="$1"
      ;;
    --expect-text-differences)
      shift
      [ $# -gt 0 ] || parity_die "--expect-text-differences needs a file"
      TEXT_EXPECT_FILE="$1"
      ;;
    -*) parity_die "unknown option $1" ;;
    *) break ;;
  esac
  shift
done
[ $# -eq 2 ] || parity_die \
  "usage: $0 [--text] [--stderr] [--asserts] [--expect-differences file|--allow-differences file] [--expect-text-differences file] <ref_dir> <lib_dir>"
REF="$1"; LIB="$2"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
EXPECTED="$TMP/expected"
: > "$EXPECTED"
TEXT_EXPECTED="$TMP/text_expected"
: > "$TEXT_EXPECTED"
if [ -n "$TEXT_EXPECT_FILE" ]; then
  [ $TEXT = 1 ] || parity_die "--expect-text-differences needs --text"
  [ -f "$TEXT_EXPECT_FILE" ] || parity_die "expectation file not found: $TEXT_EXPECT_FILE"
  awk '
    /^[[:space:]]*(#|$)/ { next }
    NF != 1 { exit 2 }
    { print $1 }
  ' "$TEXT_EXPECT_FILE" > "$TEXT_EXPECTED" ||
    parity_die "expectation file must contain one scenario name per line: $TEXT_EXPECT_FILE"
  [ -z "$(sort "$TEXT_EXPECTED" | uniq -d)" ] ||
    parity_die "duplicate expected text scenario: $(sort "$TEXT_EXPECTED" | uniq -d | head -1)"
  while read -r expected; do
    awk -v name="$expected" '$1 == name && NF == 3 { found = 1 }
      END { exit found ? 0 : 1 }' "$PARITY_SCENARIO_LIST" ||
      parity_die "unknown expected text scenario: $expected"
  done < "$TEXT_EXPECTED"
fi
if [ $EXPECT_MODE != 0 ]; then
  [ -f "$EXPECT_FILE" ] || parity_die "expectation file not found: $EXPECT_FILE"
  awk '
    /^[[:space:]]*(#|$)/ { next }
    NF != 1 { exit 2 }
    { print $1 }
  ' "$EXPECT_FILE" > "$EXPECTED" ||
    parity_die "expectation file must contain one scenario name per line: $EXPECT_FILE"
  duplicates="$(sort "$EXPECTED" | uniq -d)"
  [ -z "$duplicates" ] || parity_die \
    "duplicate expected scenario: $(echo "$duplicates" | head -1)"
  while read -r expected; do
    awk -v name="$expected" '$1 == name && NF == 3 { found = 1 }
      END { exit found ? 0 : 1 }' "$PARITY_SCENARIO_LIST" ||
      parity_die "unknown expected scenario: $expected"
  done < "$EXPECTED"
fi

total=0; identical=0; expected_total=0; expected_seen=0; bad=0
while read -r s _dt _n; do
  total=$((total + 1)); line="$s"; artifacts_ok=1
  json_ok=1; content_ok=1; rc_ok=1
  required="jsonl rc"
  [ $TEXT = 1 ] && required="$required out"
  if [ $STDERR = 1 ] || [ $ASSERTS = 1 ]; then required="$required err"; fi
  for suffix in $required; do
    [ -f "$REF/$s.$suffix" ] || {
      artifacts_ok=0; line="$line missing:ref.$suffix";
    }
    [ -f "$LIB/$s.$suffix" ] || {
      artifacts_ok=0; line="$line missing:lib.$suffix";
    }
  done
  r=$(python3 "$(dirname "$0")/cmp_jsonl.py" "$REF/$s.jsonl" "$LIB/$s.jsonl" 2>&1) || {
    json_ok=0; content_ok=0;
  }
  line="$line jsonl:${r}"
  if [ "$(cat "$REF/$s.rc" 2>/dev/null)" != "$(cat "$LIB/$s.rc" 2>/dev/null)" ]; then
    rc_ok=0; line="$line rc:$(cat "$REF/$s.rc" 2>/dev/null)/$(cat "$LIB/$s.rc" 2>/dev/null)"
  fi
  text_expected=0; text_same=1
  grep -Fqx "$s" "$TEXT_EXPECTED" && text_expected=1
  if [ $TEXT = 1 ]; then
    grep -v '^{' "$REF/$s.out" > "$TMP/r" 2>/dev/null; grep -v '^{' "$LIB/$s.out" > "$TMP/l" 2>/dev/null
    if cmp -s "$TMP/r" "$TMP/l"; then line="$line text:OK($(wc -l < "$TMP/r" | tr -d ' '))"
    elif [ $text_expected = 1 ]; then
      text_same=0; line="$line text:EXPECTED_DIFF@$(diff "$TMP/r" "$TMP/l" | head -1)"
    else content_ok=0; line="$line text:DIFF@$(diff "$TMP/r" "$TMP/l" | head -1)"; fi
  fi
  if [ $STDERR = 1 ]; then
    if cmp -s "$REF/$s.err" "$LIB/$s.err"; then line="$line stderr:OK"
    else content_ok=0; line="$line stderr:DIFF@$(diff "$REF/$s.err" "$LIB/$s.err" | head -1)"; fi
  fi
  if [ $ASSERTS = 1 ]; then
    grep '^ASSERTION FAILURE: ' "$REF/$s.err" > "$TMP/r" 2>/dev/null; grep '^ASSERTION FAILURE: ' "$LIB/$s.err" > "$TMP/l" 2>/dev/null
    if cmp -s "$TMP/r" "$TMP/l"; then line="$line asserts:OK($(wc -l < "$TMP/r" | tr -d ' '))"
    else content_ok=0; line="$line asserts:DIFF@$(diff "$TMP/r" "$TMP/l" | head -1)"; fi
  fi

  expected=0
  grep -Fqx "$s" "$EXPECTED" && expected=1
  if [ $artifacts_ok = 1 ] && [ $content_ok = 1 ] && [ $rc_ok = 1 ] && [ $text_same = 1 ]; then
    identical=$((identical + 1))
  fi
  if [ $text_expected = 1 ] && [ $text_same = 1 ]; then
    bad=$((bad + 1)); line="$line status:MISSING_EXPECTED_TEXT_DIFF"
  elif [ $expected = 1 ]; then
    expected_total=$((expected_total + 1))
    if [ $artifacts_ok = 0 ]; then
      bad=$((bad + 1)); line="$line status:UNEXPECTED_MISSING"
    elif [ $json_ok = 0 ] && [ $rc_ok = 1 ]; then
      expected_seen=$((expected_seen + 1))
      if [ $EXPECT_MODE = 1 ]; then
        line="$line status:EXPECTED_DIFF"
      else
        line="$line status:ALLOWED_DIFF"
      fi
    elif [ $rc_ok = 0 ]; then
      bad=$((bad + 1)); line="$line status:UNEXPECTED_RC"
    elif [ $EXPECT_MODE = 2 ] && [ $content_ok = 1 ]; then
      line="$line status:OK"
    else
      bad=$((bad + 1)); line="$line status:MISSING_EXPECTED_DIFF"
    fi
  elif [ $artifacts_ok = 1 ] && [ $content_ok = 1 ] && [ $rc_ok = 1 ]; then
    line="$line status:OK"
  else
    bad=$((bad + 1)); line="$line status:UNEXPECTED_DIFF"
  fi
  echo "$line"
done < <(parity_scenarios)
if [ $EXPECT_MODE = 1 ]; then
  echo "=== $identical/$total identical; $expected_seen/$expected_total expected differences ==="
elif [ $EXPECT_MODE = 2 ]; then
  echo "=== $identical/$total identical; $expected_seen/$expected_total allowed differences ==="
else
  echo "=== $identical/$total identical ==="
fi
[ $bad = 0 ]
