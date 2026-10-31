#!/usr/bin/env python3
"""Categorize every episode as success / timeout / collision / stuck / crash.

Rules (applied in this order; thresholds are constants so FINDINGS can quote them):
  success    success == true  (collisions during a successful episode are still counted in the
             collision totals but do not change the category)
  crash      the episode was cut short by a dying node or a dead simulator (diagnostics.crashed)
  collision  failed with collision_count > 0
  stuck      failed, no collision, and the robot sat still (< 0.03 m/s) for >= STUCK_MIN_S in a row
  timeout    every other failure: the time budget ran out while the robot was still moving

Writes results/failure_modes.csv (one row per episode, evidence included) and prints a summary.
"""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_data as nd  # noqa: E402

STUCK_MIN_S = 15.0
CATEGORIES = ['success', 'timeout', 'collision', 'stuck', 'crash']


def classify(ep, diag):
    if ep['success']:
        return 'success'
    if diag['crashed']:
        return 'crash'
    if ep['collision_count'] > 0:
        return 'collision'
    if diag['max_stationary_s'] >= STUCK_MIN_S:
        return 'stuck'
    return 'timeout'


def evidence(ep, diag):
    path_ratio = ep['path_length_m'] / diag['straight_dist_m'] if diag['straight_dist_m'] else float('nan')
    return (f"stop={diag['stop_reason']}; final_goal_dist={diag['final_goal_dist_m']:.2f} m; "
            f"max_stationary={diag['max_stationary_s']:.1f} s; empty_plan_frac={diag['empty_plan_frac']:.2f}; "
            f"path/straight={path_ratio:.2f}; collisions={ep['collision_count']}; "
            f"min_obstacle_dist={ep['min_obstacle_distance_m']:.2f} m")


def compute():
    eps = nd.episodes()
    diags = nd.diagnostics()
    rows = []
    for e in sorted(eps, key=lambda e: (nd.WORLDS.index(e['world']) if e['world'] in nd.WORLDS else 9, e['episode'])):
        d = diags[(e['world'], e['episode'])]
        if d['seed'] != e['seed']:
            raise SystemExit(f"seed mismatch between episodes.csv and diagnostics for {e['world']} ep {e['episode']}")
        rows.append({'world': e['world'], 'episode': e['episode'], 'seed': e['seed'],
                     'category': classify(e, d), 'evidence': evidence(e, d)})
    return rows


def main():
    rows = compute()
    out = os.path.join(nd.RESULTS, 'failure_modes.csv')
    with open(out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=['world', 'episode', 'seed', 'category', 'evidence'], lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    print(f'wrote {out} ({len(rows)} episodes)')
    for world in nd.WORLDS:
        counts = {c: sum(1 for r in rows if r['world'] == world and r['category'] == c) for c in CATEGORIES}
        print(world, counts)


if __name__ == '__main__':
    main()
