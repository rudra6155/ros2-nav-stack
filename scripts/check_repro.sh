#!/usr/bin/env bash
# Episode reproducibility: re-run one episode of the official evaluation with its recorded seed
# and compare the sampled start pose / goal with the row the original run recorded.
# usage: scripts/check_repro.sh [world=simple] [episode=7]       (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
WORLD="${1:-simple}"; EP="${2:-7}"
OUT="$RESULTS_DIR/tmp/repro"
rm -rf "$OUT"; mkdir -p "$OUT"
REPORT="$OUT/repro_report.txt"

ros2 run evaluation run_episodes --world "$WORLD" --episode "$EP" --results-dir "$OUT" --tag repro --no-bag 2>&1 | tail -3 | tee "$REPORT"

python3 - "$RESULTS_DIR/episode_diagnostics.csv" "$OUT/episode_diagnostics.csv" "$WORLD" "$EP" <<'PY' | tee -a "$REPORT"
import csv, sys
orig_p, new_p, world, ep = sys.argv[1:5]
def row(path):
    for r in csv.DictReader(open(path)):
        if r['world'] == world and r['episode'] == ep:
            return r
    sys.exit(f'FAIL: no row for {world} episode {ep} in {path}')
a, b = row(orig_p), row(new_p)
keys = ['seed', 'start_x', 'start_y', 'start_yaw', 'goal_x', 'goal_y']
bad = [k for k in keys if a[k] != b[k]]
for k in keys:
    print(f'{k:9s} original={a[k]:>10s}  rerun={b[k]:>10s}  {"same" if a[k]==b[k] else "DIFFERENT"}')
if bad:
    sys.exit(f'FAIL: poses differ for {bad}')
print(f'PASS: episode {ep} of {world} re-run with seed {a["seed"]} spawned at the same pose (and same goal)')
PY
rc=${PIPESTATUS[0]}
if [ $rc -eq 0 ]; then record_evidence check_repro pass "scripts/check_repro.sh $WORLD $EP" "$REPORT"; else record_evidence check_repro fail "scripts/check_repro.sh $WORLD $EP" "$REPORT"; fi
exit $rc
