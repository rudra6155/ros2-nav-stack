#!/usr/bin/env bash
# Runs, one after the other (they all need the CPU to themselves): the parameter sweeps, the fault
# injection and every simulator-based gate. Progress goes to stdout. Inside the container (~45 min).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
step() { echo; echo "################ $(date +%H:%M:%S)  $*"; }

step "sensitivity: lookahead"
"$HERE/run_sensitivity.sh" lookahead "0.3 0.6 1.2"
step "sensitivity: inflation"
"$HERE/run_sensitivity.sh" inflation "0.2 0.3 0.4"
step "summarize sensitivity"
python3 "$HERE/summarize_sensitivity.py"
step "fault injection";            "$HERE/fault_injection.sh"
step "check_repro";                "$HERE/check_repro.sh" simple 7
step "check_crash_handling";       "$HERE/check_crash_handling.sh"
step "check_controller_gazebo";    "$HERE/check_controller_gazebo.sh"
step "check_launch_smoke";         "$HERE/check_launch_smoke.sh"
step "check_tf";                   "$HERE/check_tf.sh"
step "check_frames";               "$HERE/check_frames.sh"
step "ALL DONE"
