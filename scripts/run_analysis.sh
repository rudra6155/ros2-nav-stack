#!/usr/bin/env bash
# Close-call / failure analyses that back FINDINGS.md (inside the container; needs the official bags).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
cd "$NAV_ROOT"
rm -rf "$RESULTS_DIR/analysis"; mkdir -p "$RESULTS_DIR/analysis"
python3 "$HERE/analyze_close_calls.py" 2>&1 | grep -v rosbag2_storage | head -4
python3 "$HERE/analyze_first_seen.py" cluttered 2 13 15 2>&1 | grep -v rosbag2_storage
python3 "$HERE/analyze_plan_vs_robot.py" cluttered 2 0 6   2>&1 | grep -v rosbag2_storage | tee "$RESULTS_DIR/analysis/plan_vs_robot_cluttered_2.txt"  | tail -1
python3 "$HERE/analyze_plan_vs_robot.py" cluttered 15 9 15  2>&1 | grep -v rosbag2_storage | tee "$RESULTS_DIR/analysis/plan_vs_robot_cluttered_15.txt" | tail -1
python3 "$HERE/analyze_plan_vs_robot.py" cluttered 13 13 18 2>&1 | grep -v rosbag2_storage | tee "$RESULTS_DIR/analysis/plan_vs_robot_cluttered_13.txt" | tail -1
"$HERE/analyze_stress.sh"
