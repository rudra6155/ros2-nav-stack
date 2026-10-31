#!/usr/bin/env bash
# TF / topic / node audit - the stack's smoke test and CI gate.
#
# Launches the stack headless, waits 20 s, then asserts:
#   * tf2_echo resolves  map->odom  and  odom->base_link  (plus base_link->laser/camera, map->base_link)
#   * `ros2 topic list` contains /scan /odom /cmd_vel /plan /obstacles (+ /costmap /goal_pose /tf ...)
#   * `ros2 node list` contains EVERY expected node (scripts/expected_nodes.txt)
#   * every stack node runs on simulation time (use_sim_time == True)
#   * the key topics are actually publishing
# Exit status is non-zero if anything is missing. Report: results/tf_audit.txt
#
# usage: scripts/check_tf.sh [world]        (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
CHECK_NAME=check_tf
WORLD="${1:-simple}"
WAIT="${CHECK_TF_WAIT:-20}"
mkdir -p "$RESULTS_DIR"
REPORT="$RESULTS_DIR/tf_audit.txt"
: > "$REPORT"
FAILS=0
say()  { echo "$*" | tee -a "$REPORT"; }
pass() { say "PASS  $*"; }
fail() { say "FAIL  $*"; FAILS=$((FAILS + 1)); }
trap stack_stop EXIT

stack_start "$WORLD"
say "== launched: ros2 launch bringup sim.launch.py world:=$WORLD headless:=true  (log: $STACK_LOG)"
say "== waiting ${WAIT} s"
sleep "$WAIT"

say ""; say "== TF (tf2_echo must resolve each pair)"
for pair in "map odom" "odom base_link" "map base_link" "base_link laser" "base_link camera"; do
  ok=0
  for attempt in 1 2 3; do
    out="$(timeout 12 ros2 run tf2_ros tf2_echo $pair 2>&1)"
    if echo "$out" | grep -q "Translation"; then ok=1; break; fi
  done
  if [ "$ok" = 1 ]; then pass "tf2_echo $pair resolves"; else fail "tf2_echo $pair did not resolve: $(echo "$out" | tail -1)"; fi
done

say ""; say "== topics (polled for up to 30 s: DDS discovery is not instantaneous)"
deadline=$(( $(date +%s) + 30 ))
missing=""
while :; do
  topics="$(ros2 topic list --no-daemon 2>/dev/null)"
  missing=""
  while read -r t; do
    t="${t%%#*}"; t="$(echo "$t" | xargs)"; [ -z "$t" ] && continue
    echo "$topics" | grep -qx "$t" || missing="$missing $t"
  done < "$HERE/expected_topics.txt"
  [ -z "$missing" ] && break
  [ "$(date +%s)" -ge "$deadline" ] && break
  sleep 2
done
while read -r t; do
  t="${t%%#*}"; t="$(echo "$t" | xargs)"; [ -z "$t" ] && continue
  if echo "$topics" | grep -qx "$t"; then pass "topic $t exists"; else fail "topic $t is missing"; fi
done < "$HERE/expected_topics.txt"

say ""; say "== nodes (ros2 node list must include every node in scripts/expected_nodes.txt; polled for up to 40 s)"
deadline=$(( $(date +%s) + 40 ))
while :; do
  nodelist="$(ros2 node list --no-daemon 2>/dev/null)"
  missing=""
  while read -r n; do
    n="${n%%#*}"; n="$(echo "$n" | xargs)"; [ -z "$n" ] && continue
    echo "$nodelist" | grep -qx "$n" || missing="$missing $n"
  done < "$HERE/expected_nodes.txt"
  [ -z "$missing" ] && break
  [ "$(date +%s)" -ge "$deadline" ] && break
  sleep 2
done
say "ros2 node list:"; echo "$nodelist" | sed 's/^/    /' | tee -a "$REPORT" > /dev/null
while read -r n; do
  n="${n%%#*}"; n="$(echo "$n" | xargs)"; [ -z "$n" ] && continue
  if echo "$nodelist" | grep -qx "$n"; then pass "node $n is in ros2 node list"; else fail "node $n is missing from ros2 node list"; fi
done < "$HERE/expected_nodes.txt"
nodes_out="$(python3 "$HERE/wait_for_nodes.py" --nodes "$HERE/expected_nodes.txt" --timeout 30 2>&1)"
echo "$nodes_out" | grep -q "^ALL" && pass "rclpy graph cross-check: $nodes_out" || fail "rclpy graph cross-check: $nodes_out"

say ""; say "== simulation time (use_sim_time must be True on every stack node)"
for n in /obstacle_detector /astar_planner /waypoint_follower /robot_state_publisher /map_odom_static_tf; do
  v="$(timeout 20 ros2 param get $n use_sim_time 2>&1 | tail -1)"
  echo "$v" | grep -q "True" && pass "$n use_sim_time = True" || fail "$n use_sim_time: $v"
done
if grep -q "use_sim_time asserted" "$STACK_LOG"; then pass "sim.launch.py asserted use_sim_time"; else fail "launch log lacks the use_sim_time assertion"; fi

say ""; say "== liveness (one message each, 25 s budget per topic)"
for t in /clock /scan /odom /costmap /obstacles /tf; do
  if timeout 25 ros2 topic echo --once "$t" > /dev/null 2>&1; then pass "$t is publishing"; else fail "$t published nothing"; fi
done

say ""
if [ "$FAILS" -eq 0 ]; then
  say "RESULT: PASS (all checks)"; record_evidence check_tf pass "scripts/check_tf.sh $WORLD" "$REPORT"; exit 0
else
  say "RESULT: FAIL ($FAILS check(s) failed)"; record_evidence check_tf fail "scripts/check_tf.sh $WORLD" "$REPORT"; exit 1
fi
