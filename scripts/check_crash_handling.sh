#!/usr/bin/env bash
# Crash protocol: kill a node (astar_planner) in the middle of episode 2 and verify that
#   - the episode is recorded as timed_out=true (crashed=true in the diagnostics),
#   - the node name and its stderr go to crash_log.txt,
#   - the stack is restarted and the remaining episode still runs (nothing is dropped).
# Uses its own results dir (results/tmp/crash_test); the official results are untouched.
# usage: scripts/check_crash_handling.sh      (inside the container, ~2 min)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
OUT="$RESULTS_DIR/tmp/crash_test"
rm -rf "$OUT"; mkdir -p "$OUT"
LOG="$OUT/run.log"; REPORT="$OUT/report.txt"

ros2 run evaluation run_episodes --world cluttered --episodes 3 --results-dir "$OUT" --tag crashtest --no-bag > "$LOG" 2>&1 &
RP=$!
for _ in $(seq 1 300); do grep -q "ep  1/3" "$LOG" 2>/dev/null && break; sleep 1; done
sleep 8                                     # episode 2 is now running
echo "killing astar_planner (pid: $(pgrep -f lib/navigation/astar_planner | tr '\n' ' ')) during episode 2" | tee "$REPORT"
pkill -9 -f lib/navigation/astar_planner
wait $RP; echo "run_episodes exit code: $?" | tee -a "$REPORT"

python3 - "$OUT" <<'PY' | tee -a "$REPORT"
import csv, sys, os
out = sys.argv[1]
eps = list(csv.DictReader(open(os.path.join(out, 'episodes.csv'))))
diag = list(csv.DictReader(open(os.path.join(out, 'episode_diagnostics.csv'))))
crash = open(os.path.join(out, 'crash_log.txt')).read()
checks = [
    ('3 episodes recorded (none dropped)', len(eps) == 3),
    ('episode 2: success=false', eps[1]['success'] == 'false'),
    ('episode 2: timed_out=true', eps[1]['timed_out'] == 'true'),
    ('episode 2: crashed=true, stop_reason=node_crash', diag[1]['crashed'] == 'true' and diag[1]['stop_reason'] == 'node_crash'),
    ('crash_log.txt names the node (astar_planner) and the episode', 'astar_planner' in crash and 'episode=2' in crash),
    ('crash_log.txt carries the node stderr tail', '[astar_planner-' in crash and 'astar_planner up' in crash),
    ('stack restarted: episode 3 ran after the crash', diag[2]['stop_reason'] in ('goal_reached', 'hard_timeout', 'exceeded_3x_mean')),
]
bad = 0
for text, ok in checks:
    print(('PASS  ' if ok else 'FAIL  ') + text)
    bad += (not ok)
sys.exit(1 if bad else 0)
PY
rc=${PIPESTATUS[0]}
if [ $rc -eq 0 ]; then record_evidence check_crash_handling pass "scripts/check_crash_handling.sh" "$REPORT"; else record_evidence check_crash_handling fail "scripts/check_crash_handling.sh" "$REPORT"; fi
exit $rc
