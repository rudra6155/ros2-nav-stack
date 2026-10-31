#!/usr/bin/env bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
step() { echo; echo "################ $(date +%H:%M:%S)  $*"; }
step "fault injection A and C"; FAULT_STEPS="A C" "$HERE/fault_injection.sh"
step "sweep: inflation 0.5";   "$HERE/run_sensitivity.sh" inflation "0.5"; python3 "$HERE/summarize_sensitivity.py"
step "analysis";               "$HERE/run_analysis.sh"
step "FOLLOWUPS DONE"
