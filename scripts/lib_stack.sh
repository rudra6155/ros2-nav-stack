#!/usr/bin/env bash
# Shared helpers for the contract checks. SOURCE this file; do not execute it.
# Needs a sourced ROS + workspace environment (login shells inside the container have it).

NAV_ROOT="${NAV_ROOT:-/workspace}"
RESULTS_DIR="${NAV_RESULTS_DIR:-$NAV_ROOT/results}"
SCRIPTS_DIR="$NAV_ROOT/scripts"
STACK_PID=""
STACK_LOG=""
STACK_T0=""

# stack_start WORLD [extra launch args...]  - sim + stack, headless, in its own process group
stack_start() {
  local world="${1:-simple}"; shift || true
  mkdir -p "$RESULTS_DIR/logs"
  STACK_LOG="$RESULTS_DIR/logs/${CHECK_NAME:-check}_launch.log"
  STACK_T0="$(date +%s.%N)"
  setsid ros2 launch bringup sim.launch.py world:="$world" headless:=true "$@" > "$STACK_LOG" 2>&1 &
  STACK_PID=$!
}

# stack_stop - clean shutdown of everything stack_start launched (never leaves a gzserver behind)
stack_stop() {
  if [ -n "${STACK_PID:-}" ]; then
    kill -INT -- "-$STACK_PID" 2>/dev/null
    for _ in $(seq 1 100); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 0.2; done
    kill -KILL -- "-$STACK_PID" 2>/dev/null
    STACK_PID=""
  fi
  pkill -9 -x gzserver 2>/dev/null
  pkill -9 -x gzclient 2>/dev/null
  return 0
}

# record_evidence NAME STATUS CMD DETAILS_FILE
record_evidence() {
  python3 "$SCRIPTS_DIR/write_evidence.py" "$1" "$2" --cmd "$3" --details-file "$4" --results-dir "$RESULTS_DIR"
}
