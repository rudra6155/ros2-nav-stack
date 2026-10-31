#!/usr/bin/env bash
# Re-run every gate at the final commit (clean build, all tests, every simulator check). Inside the container.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
step() { echo; echo "################ $(date +%H:%M:%S)  $*"; }
cd "$NAV_ROOT/ros2_ws" && rm -rf build install log
source /opt/ros/humble/setup.bash
step "colcon build";  "$HERE/run_gate.sh" colcon_build -- colcon build --symlink-install --event-handlers console_cohesion+ 2>&1 | grep -E "error|warning:|Finished|Failed|Summary"
source "$NAV_ROOT/ros2_ws/install/setup.bash"
step "colcon test";   "$HERE/run_gate.sh" colcon_test -- bash -c "colcon test --event-handlers console_direct+ && colcon test-result --verbose" 2>&1 | grep -E "PASSED|FAILED|tests passed|Summary|cross-track"
step "launch smoke";  "$HERE/check_launch_smoke.sh" | tail -2
step "tf audit";      "$HERE/check_tf.sh" | grep -E "FAIL|RESULT"
step "frame audit";   "$HERE/check_frames.sh" | grep -E "FAIL|PASS  /scan header"
step "repro";         "$HERE/check_repro.sh" simple 7 | tail -1
step "controller";    "$HERE/check_controller_gazebo.sh" | tail -1
step "crash";         "$HERE/check_crash_handling.sh" | grep -E "PASS|FAIL"
step "FINAL GATES DONE"
