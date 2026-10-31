#!/usr/bin/env bash
# Re-run the failing stress episode WITH a rosbag and attribute the failure by evidence.
# (The sweep runs themselves use --no-bag; runs are not bit-identical, so up to 3 attempts are made
#  and every outcome is recorded.)   usage: scripts/analyze_stress.sh   (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
OUT="$RESULTS_DIR/tmp/stress"; rm -rf "$OUT"; mkdir -p "$OUT" "$RESULTS_DIR/analysis"
LOG="$RESULTS_DIR/analysis/stress_attempts.txt"; : > "$LOG"
for k in 1 2 3; do
  d="$OUT/try$k"
  ros2 run evaluation run_episodes --world cluttered --episode 4 --lookahead 1.2 --timeout 45 --results-dir "$d" --tag "stress$k" 2>&1 | grep -E "ep +[0-9]+/" | tee -a "$LOG"
  if python3 - "$d/episodes.csv" <<'PY'
import csv, sys
r = list(csv.DictReader(open(sys.argv[1])))[0]
sys.exit(0 if int(r['collision_count']) > 0 else 1)
PY
  then
    echo "attempt $k: collision reproduced -> analysing" | tee -a "$LOG"
    export NAV_BAG="$d/bags/stress${k}_cluttered"
    NAV_DIAG="$d/episode_diagnostics.csv" NAV_ANALYSIS_KEY="cluttered_4_lk12" python3 "$HERE/analyze_plan_vs_robot.py" cluttered 4 2>&1 | grep -v rosbag2_storage | tee "$RESULTS_DIR/analysis/plan_vs_robot_lookahead1.2_ep4.txt" | tail -3
    python3 "$HERE/analyze_stuck.py" cluttered 4 lk12_ep4 2>&1 | grep -v rosbag2_storage
    break
  fi
  echo "attempt $k: no collision" | tee -a "$LOG"
done
