#!/usr/bin/env bash
# run_gate.sh NAME -- COMMAND...   : run COMMAND, keep its output and record pass/fail evidence
# (results/evidence/NAME.json). Used for build/test verification steps.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
NAME="$1"; shift; [ "$1" = "--" ] && shift
mkdir -p "$RESULTS_DIR/logs"
LOG="$RESULTS_DIR/logs/gate_$NAME.log"
"$@" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
{ echo "exit code: $rc"; echo "---"; tail -40 "$LOG"; } > "$LOG.tail"
if [ $rc -eq 0 ]; then record_evidence "$NAME" pass "$*" "$LOG.tail"; else record_evidence "$NAME" fail "$*" "$LOG.tail"; fi
exit $rc
