"""Loaders for the result files, shared by the doc/claim tooling. Pure stdlib (+ the project's own
strict episodes loader), so it runs on the host as well as in the container."""
import csv
import glob
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.environ.get('NAV_RESULTS_DIR', os.path.join(ROOT, 'results'))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'evaluation', 'scripts'))
import summarize  # noqa: E402  (the project's strict loader / schema)

WORLDS = ['simple', 'cluttered']


def read_csv_dicts(path):
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def episodes(path=None):
    return summarize.load_episodes(path or os.path.join(RESULTS, 'episodes.csv'))


def diagnostics(path=None):
    rows = read_csv_dicts(path or os.path.join(RESULTS, 'episode_diagnostics.csv'))
    out = {}
    for r in rows:
        key = (r['world'], int(r['episode']))
        out[key] = {
            'seed': int(r['seed']),
            'start': (float(r['start_x']), float(r['start_y']), float(r['start_yaw'])),
            'goal': (float(r['goal_x']), float(r['goal_y'])),
            'straight_dist_m': float(r['straight_dist_m']),
            'duration_s': float(r['duration_s']),
            'final_goal_dist_m': float(r['final_goal_dist_m']),
            'max_stationary_s': float(r['max_stationary_s']),
            'empty_plan_frac': float(r['empty_plan_frac']),
            'crashed': r['crashed'].strip().lower() == 'true',
            'stop_reason': r['stop_reason'],
            'rtf': float(r['rtf']) if r['rtf'] != 'nan' else float('nan'),
            'timeout_cap_s': float(r['timeout_cap_s']),
        }
    return out


def summary(path=None):
    rows = read_csv_dicts(path or os.path.join(RESULTS, 'summary.csv'))
    return {r['world']: r for r in rows}


def world_stats():
    with open(os.path.join(ROOT, 'worlds', 'world_stats.json')) as fh:
        return json.load(fh)


def hardware():
    with open(os.path.join(RESULTS, 'hardware.json')) as fh:
        return json.load(fh)


def evidence(name):
    p = os.path.join(RESULTS, 'evidence', name + '.json')
    if not os.path.isfile(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def wallclock():
    p = os.path.join(RESULTS, 'wallclock.json')
    if not os.path.isfile(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def sensitivity():
    p = os.path.join(RESULTS, 'sensitivity.csv')
    return read_csv_dicts(p) if os.path.isfile(p) else None


def mean(xs):
    return sum(xs) / len(xs)


def sample_std(xs):
    if len(xs) < 2:
        return float('nan')
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))
