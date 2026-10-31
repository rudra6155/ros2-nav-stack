#!/usr/bin/env bash
# Launch smoke: `sim.launch.py headless:=true` must bring up ALL expected nodes in < 30 s.
# Report: results/launch_smoke.txt.   usage: scripts/check_launch_smoke.sh [world]  (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
CHECK_NAME=check_launch_smoke
WORLD="${1:-simple}"
LIMIT="${SMOKE_LIMIT_S:-30}"
mkdir -p "$RESULTS_DIR"
REPORT="$RESULTS_DIR/launch_smoke.txt"
trap stack_stop EXIT

stack_start "$WORLD"
out="$(python3 "$HERE/wait_for_nodes.py" --nodes "$HERE/expected_nodes.txt" --timeout 90 --t0 "$STACK_T0" 2>&1)"
echo "$out" | tee "$REPORT"
elapsed="$(echo "$out" | grep -o '[0-9.]* s' | head -1 | cut -d' ' -f1)"
if echo "$out" | grep -q '^ALL' && python3 -c "import sys; sys.exit(0 if float('${elapsed:-999}') < float('$LIMIT') else 1)"; then
  echo "PASS: all nodes up in ${elapsed} s (< ${LIMIT} s)" | tee -a "$REPORT"
  record_evidence check_launch_smoke pass "scripts/check_launch_smoke.sh $WORLD" "$REPORT"; exit 0
fi
echo "FAIL: not all nodes up within ${LIMIT} s (elapsed: ${elapsed:-n/a} s)" | tee -a "$REPORT"
record_evidence check_launch_smoke fail "scripts/check_launch_smoke.sh $WORLD" "$REPORT"; exit 1
