#!/usr/bin/env bash
# Paired micro-experiment behind FINDINGS 3.3: cluttered episode 4 (the episode that failed at lookahead 1.2),
# repeated REPEATS times at lookahead 0.6 (nominal) and 1.2. Runs are not bit-identical, so repetition
# is the only honest way to see whether the effect is real.   Output: results/ablation_ep4.json
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
REPEATS="${REPEATS:-3}"
OUT="$RESULTS_DIR/tmp/ablation"; rm -rf "$OUT"; mkdir -p "$OUT"
for lk in 0.6 1.2; do
  for r in $(seq 1 "$REPEATS"); do
    ros2 run evaluation run_episodes --world cluttered --episode 4 --lookahead "$lk" --timeout 30 \
      --results-dir "$OUT/lk${lk}_r$r" --tag abl --no-bag 2>&1 | grep -E "ep +[0-9]+/"
  done
done
python3 - "$OUT" "$RESULTS_DIR" "$REPEATS" <<'PY'
import csv, json, os, sys
out, res, reps = sys.argv[1], sys.argv[2], int(sys.argv[3])
data = {}
for lk in ('0.6', '1.2'):
    runs = []
    for r in range(1, reps + 1):
        d = os.path.join(out, f'lk{lk}_r{r}')
        e = list(csv.DictReader(open(os.path.join(d, 'episodes.csv'))))[0]
        g = list(csv.DictReader(open(os.path.join(d, 'episode_diagnostics.csv'))))[0]
        runs.append({'success': e['success'] == 'true', 'collisions': int(e['collision_count']),
                     'min_distance_m': float(e['min_obstacle_distance_m']), 'stop_reason': g['stop_reason']})
    data[f'lookahead_{lk}'] = {
        'runs': reps, 'runs_with_contact': sum(1 for x in runs if x['collisions'] > 0),
        'runs_success': sum(1 for x in runs if x['success']),
        'mean_min_distance_m': round(sum(x['min_distance_m'] for x in runs) / reps, 3),
        'min_min_distance_m': round(min(x['min_distance_m'] for x in runs), 3), 'detail': runs}
json.dump(data, open(os.path.join(res, 'ablation_ep4.json'), 'w'), indent=2)
print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'detail'} for k, v in data.items()}, indent=2))
PY
