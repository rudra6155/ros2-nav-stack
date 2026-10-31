#!/usr/bin/env bash
# Fault injection: three deliberate faults, each run in the real simulator, so that the "Known
# fragility" section of ARCHITECTURE.md reports OBSERVED behaviour instead of speculation.
#
#   A. lidar frame mismatch  - the lidar plugin stamps /scan with `laser_link` while the URDF link is `laser`
#   B. unreachable goals     - a goal inside a wall and a goal outside the map
#   C. low real-time factor  - an episode run while the CPUs are saturated by busy loops
#
# Needs the official results (results/episode_diagnostics.csv) to pick the episodes / baselines.
# Output: results/fault_injection.json  (inside the container, ~10 min)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
OUT="$RESULTS_DIR/tmp/fault"; mkdir -p "$OUT"
STEPS="${FAULT_STEPS:-A B C}"      # run a subset with e.g. FAULT_STEPS="A C"
CHECK_NAME=fault_injection
LOAD_PROCS="${LOAD_PROCS:-12}"
export STACK_NICE="${STACK_NICE:-8}"       # the stack runs at lower priority than the busy loops (the sim alone needs < 1 core)

# ---------------------------------------------------------------- A. frame mismatch
has() { case " $STEPS " in *" $1 "*) return 0;; esac; return 1; }
EP_A=$(python3 - "$RESULTS_DIR/episode_diagnostics.csv" "$NAV_ROOT/worlds/simple.obstacles" <<'PY'
import csv, math, sys
diag, obs_path = sys.argv[1], sys.argv[2]
obs = []
for line in open(obs_path):
    p = line.split()
    if not p or p[0].startswith('#') or p[0] == 'arena':
        continue
    obs.append(('box', *map(float, p[2:7])) if p[0] == 'box' else ('cyl', *map(float, p[2:5])))
def sdf(o, x, y):
    if o[0] == 'cyl':
        return math.hypot(x - o[1], y - o[2]) - o[3]
    _, cx, cy, yaw, sx, sy = o
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
    qx, qy = abs(lx) - sx / 2, abs(ly) - sy / 2
    return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
for r in csv.DictReader(open(diag)):
    if r['world'] != 'simple':
        continue
    sx, sy, gx, gy = (float(r[k]) for k in ('start_x', 'start_y', 'goal_x', 'goal_y'))
    # interior obstacles only (skip the 4 walls): does the straight line pass within 0.3 m of one?
    inner = [o for o in obs[4:]]
    worst = min(sdf(o, sx + t / 100 * (gx - sx), sy + t / 100 * (gy - sy)) for o in inner for t in range(101))
    if worst < 0.3:
        print(r['episode']); break
PY
)
EP_A="${EP_A:-2}"
if has A; then
rm -rf "$OUT/A" "$OUT"/A_*
echo "=== A. lidar frame mismatch (simple world, episode $EP_A: its straight start->goal line crosses an obstacle)"
python3 "$HERE/fault_probe.py" --duration 75 > "$OUT/A_probe.json" 2> "$OUT/A_probe.err" &
PROBE=$!
ros2 run evaluation run_episodes --world simple --episode "$EP_A" --launch-arg lidar_frame=laser_link --timeout 60 \
  --results-dir "$OUT/A" --tag faultA --no-bag > "$OUT/A_run.log" 2>&1
wait $PROBE
tail -3 "$OUT/A_run.log"
cat "$OUT/A_probe.json"
fi

# ---------------------------------------------------------------- B. unreachable goals
if has B; then
rm -f "$OUT"/B*
echo "=== B. unreachable goals"
CHECK_NAME=fault_B
stack_start simple
python3 "$HERE/wait_for_nodes.py" --nodes "$HERE/expected_nodes.txt" --timeout 90 > /dev/null
sleep 3
python3 "$HERE/fault_unreachable.py" --observe 10 > "$OUT/B.json" 2> "$OUT/B.err"
cat "$OUT/B.json"
cp "$STACK_LOG" "$OUT/B_launch.log"
stack_stop
fi

# ---------------------------------------------------------------- C. low RTF
if has C; then
rm -rf "$OUT"/C2 "$OUT"/C8
echo "=== C. low real-time factor ($LOAD_PROCS busy loops on $(nproc) cores, stack at nice +$STACK_NICE)"
for i in $(seq 1 "$LOAD_PROCS"); do (yes > /dev/null &) ; done
for ep in 2 8; do
  nice -n "$STACK_NICE" ros2 run evaluation run_episodes --world simple --episode "$ep" --results-dir "$OUT/C$ep" --tag faultC --no-bag 2>&1 | grep -E "ep +[0-9]+/|DONE"
done
pkill -x yes
fi

# ---------------------------------------------------------------- aggregate
python3 - "$OUT" "$RESULTS_DIR" "$EP_A" "$LOAD_PROCS" <<'PY'
import csv, json, os, re, sys
out, res, ep_a, load = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
def row(path, world, ep, name):
    for r in csv.DictReader(open(path)):
        if r['world'] == world and int(r['episode']) == ep:
            return r
    raise SystemExit(f'no row for {world} {ep} in {path}')
def result(d, ep):
    e = row(os.path.join(d, 'episodes.csv'), 'simple', ep, 'ep')
    g = row(os.path.join(d, 'episode_diagnostics.csv'), 'simple', ep, 'diag')
    return {'success': e['success'] == 'true', 'timed_out': e['timed_out'] == 'true', 'time_to_goal_s': e['time_to_goal_s'],
            'path_length_m': float(e['path_length_m']), 'collision_count': int(e['collision_count']),
            'min_obstacle_distance_m': float(e['min_obstacle_distance_m']), 'stop_reason': g['stop_reason'],
            'final_goal_dist_m': float(g['final_goal_dist_m']), 'rtf': float(g['rtf']), 'seed': int(g['seed'])}
fa = {'episode': ep_a, 'fault': "lidar plugin frame_name = 'laser_link' (URDF link is 'laser')"}
fa['probe'] = json.load(open(os.path.join(out, 'A_probe.json')))
fa['result'] = result(os.path.join(out, 'A'), ep_a)
fa['baseline'] = result(res, ep_a)
launch = open(os.path.join(out, 'A', 'logs', 'faultA_simple_launch_1.log'), errors='replace').read()
fa['dropped_scan_warnings'] = len(re.findall(r"Message Filter dropping message: frame 'laser_link'", launch))
fb = json.load(open(os.path.join(out, 'B.json')))
bl = open(os.path.join(out, 'B_launch.log'), errors='replace').read()
fb['planner_warnings'] = len(re.findall(r'no plan to .* publishing empty path', bl))
fb['planner_warning_reasons'] = sorted(set(re.findall(r'no plan to \([^)]*\): (.*?) -> publishing', bl)))
fc = {'busy_loops': load, 'cores': os.cpu_count(), 'stack_nice': int(os.environ.get('STACK_NICE', '8')), 'episodes': []}
for ep in (2, 8):
    fc['episodes'].append({'episode': ep, 'loaded': result(os.path.join(out, f'C{ep}'), ep), 'baseline': result(res, ep)})
json.dump({'frame_mismatch': fa, 'unreachable_goal': fb, 'low_rtf': fc}, open(os.path.join(res, 'fault_injection.json'), 'w'), indent=2)
print('wrote results/fault_injection.json')
PY
rc=$?
if [ $rc -eq 0 ]; then record_evidence fault_injection pass "scripts/fault_injection.sh" "$RESULTS_DIR/fault_injection.json"; else record_evidence fault_injection fail "scripts/fault_injection.sh" /dev/null; fi
exit $rc
