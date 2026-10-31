#!/usr/bin/env python3
"""Aggregate the parameter-sensitivity runs into results/sensitivity.csv.

Each run is a directory results/sensitivity/<param>_<value>/ written by
    run_episodes --world W --episodes 5 --results-dir <that dir> --<param> <value>
(same seeds in every run, so episode k has the same start/goal for every value).
Hard-fails if a run does not have exactly --expected episodes.
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_data as nd  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.join(nd.RESULTS, 'sensitivity'))
    ap.add_argument('--expected', type=int, default=5)
    ap.add_argument('--out', default=os.path.join(nd.RESULTS, 'sensitivity.csv'))
    a = ap.parse_args()

    rows = []
    for d in sorted(glob.glob(os.path.join(a.root, '*'))):
        m = re.match(r'(?P<param>[a-z_]+)_(?P<value>[0-9.]+)$', os.path.basename(d))
        if not m or not os.path.isfile(os.path.join(d, 'episodes.csv')):
            continue
        eps = nd.episodes(os.path.join(d, 'episodes.csv'))
        if len(eps) != a.expected:
            sys.exit(f'{d}: {len(eps)} episodes, expected {a.expected}')
        worlds = {e['world'] for e in eps}
        if len(worlds) != 1:
            sys.exit(f'{d}: mixed worlds {worlds}')
        times = [e['time_to_goal_s'] for e in eps if e['success']]
        rows.append({
            'parameter': m['param'], 'value': float(m['value']), 'world': worlds.pop(), 'n_episodes': len(eps),
            'success_rate': sum(e['success'] for e in eps) / len(eps),
            'mean_time_s': nd.mean(times) if times else float('nan'),
            'mean_path_m': nd.mean([e['path_length_m'] for e in eps]),
            'collision_count_total': sum(e['collision_count'] for e in eps),
            'timeout_count': sum(e['timed_out'] for e in eps),
            'mean_min_obstacle_distance_m': nd.mean([e['min_obstacle_distance_m'] for e in eps]),
        })
    if not rows:
        sys.exit(f'no sensitivity runs found under {a.root}')
    rows.sort(key=lambda r: (r['parameter'], r['value']))
    cols = list(rows[0].keys())
    with open(a.out, 'w', newline='') as fh:
        w = csv.writer(fh, lineterminator='\n')
        w.writerow(cols)
        for r in rows:
            w.writerow([('nan' if isinstance(r[c], float) and math.isnan(r[c]) else
                         (f'{r[c]:.4f}' if isinstance(r[c], float) else r[c])) for c in cols])
    print(f'wrote {a.out}')
    for r in rows:
        print(r)


if __name__ == '__main__':
    main()
