#!/usr/bin/env bash
# Parameter sensitivity: the SAME 5 episodes (seeds 1001..1005 of the cluttered world) for three
# values of one parameter. Episodes are defined with the nominal inflation, so start/goal pairs are
# identical across values (paired comparison).
#
#   scripts/run_sensitivity.sh lookahead "0.3 0.6 1.0"
#   scripts/run_sensitivity.sh inflation "0.2 0.3 0.4"
#
# Output: results/sensitivity/<param>_<value>/{episodes.csv,episode_diagnostics.csv,rtf_log.csv}
# then `python3 scripts/summarize_sensitivity.py` -> results/sensitivity.csv. (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
PARAM="${1:?parameter: lookahead|inflation}"; VALUES="${2:?values}"; WORLD="${3:-cluttered}"; N="${4:-5}"
for v in $VALUES; do
  d="$RESULTS_DIR/sensitivity/${PARAM}_${v}"
  rm -rf "$d"; mkdir -p "$d"
  echo "=== $PARAM = $v  ($N episodes, world $WORLD) -> $d"
  ros2 run evaluation run_episodes --world "$WORLD" --episodes "$N" --results-dir "$d" --tag "sens_${PARAM}_${v}" --no-bag --"$PARAM" "$v" 2>&1 | grep -E "ep +[0-9]+/|DONE|FATAL|CRASH"
done
