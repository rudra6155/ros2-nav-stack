#!/usr/bin/env bash
# Headless bringup acceptance check, run from the HOST:
#   docker compose run --rm ros2 bash -lc "ros2 launch bringup sim.launch.py headless:=true & sleep 20 && ros2 node list && kill %1"
# and a check that every node of scripts/expected_nodes.txt is in its output ("all nodes alive").
# usage (host, repo root): scripts/check_acceptance.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
cd "$ROOT"
mkdir -p results/logs results/evidence
OUT=results/logs/check_acceptance.txt
docker compose run --rm ros2 bash -lc "ros2 launch bringup sim.launch.py headless:=true & sleep 20 && ros2 node list && kill %1" 2>&1 | tee "$OUT"
missing=""
while read -r n; do
  n="${n%%#*}"; n="$(echo "$n" | xargs)"; [ -z "$n" ] && continue
  grep -qx "$n" "$OUT" || missing="$missing $n"
done < "$HERE/expected_nodes.txt"
if [ -z "$missing" ]; then
  echo "PASS: every expected node is in the ros2 node list after 20 s"
  python3 "$HERE/write_evidence.py" check_acceptance pass --cmd 'docker compose run --rm ros2 bash -lc "ros2 launch bringup sim.launch.py headless:=true & sleep 20 && ros2 node list && kill %1"' --details-file "$OUT" --results-dir "$ROOT/results"
  exit 0
fi
echo "FAIL: missing from ros2 node list:$missing"
python3 "$HERE/write_evidence.py" check_acceptance fail --cmd 'acceptance command' --details-file "$OUT" --results-dir "$ROOT/results"
exit 1
