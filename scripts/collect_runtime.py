#!/usr/bin/env python3
"""Write results/wallclock.json from the per-run wall-clock records of run_episodes
(results/logs/wallclock.txt: one line per run). Only runs tagged `eval` count as the evaluation budget;
the sensitivity runs are reported separately."""
import json
import os
import re
import sys

R = os.environ.get('NAV_RESULTS_DIR', os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results'))


def parse(path):
    runs = []
    if os.path.isfile(path):
        for line in open(path):
            m = re.search(r'tag=(\S+) world=(\S+) episodes=(\d+) wall_minutes=([\d.]+)', line)
            if m:
                runs.append({'tag': m[1], 'world': m[2], 'episodes': int(m[3]), 'wall_minutes': float(m[4])})
    return runs


def main():
    runs = parse(os.path.join(R, 'logs', 'wallclock.txt'))
    for sub in sorted(os.listdir(os.path.join(R, 'sensitivity'))) if os.path.isdir(os.path.join(R, 'sensitivity')) else []:
        runs += parse(os.path.join(R, 'sensitivity', sub, 'logs', 'wallclock.txt'))
    ev = [r for r in runs if r['tag'] == 'eval']
    sens = [r for r in runs if r['tag'] != 'eval']
    if not ev:
        sys.exit('no evaluation runs (tag=eval) found in results/logs/wallclock.txt')
    out = {
        'evaluation_runs': ev,
        'total_minutes': round(sum(r['wall_minutes'] for r in ev), 1),
        'total_episodes': sum(r['episodes'] for r in ev),
        'sensitivity_runs': sens,
        'sensitivity_total_minutes': round(sum(r['wall_minutes'] for r in sens), 1),
        'note': 'wall-clock minutes including the ~20 s stack start-up of each run; real-time factor is capped at 1.0',
    }
    with open(os.path.join(R, 'wallclock.json'), 'w') as fh:
        json.dump(out, fh, indent=2)
        fh.write('\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
