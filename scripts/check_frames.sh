#!/usr/bin/env bash
# Frame-id consistency: every frame_id that is published must match the URDF
# (e.g. the lidar frame is exactly `laser`, never `laser_link`).
# Report: results/frame_audit.txt.   usage: scripts/check_frames.sh [world]   (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
CHECK_NAME=check_frames
WORLD="${1:-simple}"
mkdir -p "$RESULTS_DIR"
REPORT="$RESULTS_DIR/frame_audit.txt"
trap stack_stop EXIT

URDF="$(ros2 pkg prefix robot_description)/share/robot_description/urdf/diffbot.urdf.xacro"
LINKS="$(xacro "$URDF" 2>/dev/null | python3 -c "import sys,re; print(' '.join(re.findall(r'<link name=\"([^\"]+)\"', sys.stdin.read())))")"
if [ -z "$LINKS" ]; then echo "FAIL: could not read links from $URDF"; record_evidence check_frames fail "scripts/check_frames.sh $WORLD" /dev/null; exit 1; fi
echo "URDF links: $LINKS"

stack_start "$WORLD"
python3 "$HERE/wait_for_nodes.py" --nodes "$HERE/expected_nodes.txt" --timeout 90 || { echo "FAIL: stack did not come up"; record_evidence check_frames fail "scripts/check_frames.sh $WORLD" "$STACK_LOG"; exit 1; }
sleep 5
python3 "$HERE/frame_audit.py" --urdf-links "$LINKS" --report "$REPORT" --timeout 60
rc=$?
if [ $rc -eq 0 ]; then record_evidence check_frames pass "scripts/check_frames.sh $WORLD" "$REPORT"; else record_evidence check_frames fail "scripts/check_frames.sh $WORLD" "$REPORT"; fi
exit $rc
