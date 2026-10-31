#!/usr/bin/env bash
# Controller contract, Gazebo-in-the-loop: starting 0.5 m off a straight path the controller must
# bring the cross-track error below 0.1 m within 5 s (and respect the velocity limits).
# Launches the `empty` world + the waypoint_follower only (no planner), runs controller_check.
# Report: results/controller_check.json     usage: scripts/check_controller_gazebo.sh   (in the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
CHECK_NAME=check_controller_gazebo
mkdir -p "$RESULTS_DIR"
trap 'kill ${FOLLOWER_PID:-0} 2>/dev/null; stack_stop' EXIT

stack_start empty with_stack:=false
CFG="$(ros2 pkg prefix bringup)/share/bringup/config/stack_params.yaml"
LIM="$(ros2 pkg prefix robot_description)/share/robot_description/config/robot_limits.yaml"
ros2 run control waypoint_follower --ros-args -p use_sim_time:=true --params-file "$CFG" --params-file "$LIM" \
  > "$RESULTS_DIR/logs/check_controller_gazebo_follower.log" 2>&1 &
FOLLOWER_PID=$!
sleep 8
ros2 run evaluation controller_check --results-dir "$RESULTS_DIR" 2>&1 | tee "$RESULTS_DIR/logs/controller_check.txt"
rc=${PIPESTATUS[0]}
if [ $rc -eq 0 ]; then record_evidence check_controller_gazebo pass "scripts/check_controller_gazebo.sh" "$RESULTS_DIR/controller_check.json"
else record_evidence check_controller_gazebo fail "scripts/check_controller_gazebo.sh" "$RESULTS_DIR/controller_check.json"; fi
exit $rc
