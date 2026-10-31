#!/usr/bin/env python3
"""Summarize results/episodes.csv into results/summary.csv (+ results/summary.png).

This script REFUSES to produce a partial or silently-wrong summary. It exits non-zero (and
writes nothing) when
  * the episodes header is not EXACTLY the required columns, in order,
  * any row has a missing/extra field, an empty cell or an unparsable value,
  * a world has n_episodes != --expected, or its episode numbers are not exactly 1..expected,
  * an expected world is absent (or an unexpected one is present),
  * the per-episode RTF log lacks a row for any episode.

    summarize.py [--episodes results/episodes.csv] [--rtf results/rtf_log.csv]
                 [--out results/summary.csv] [--png results/summary.png]
                 [--expected 20] [--worlds simple cluttered] [--no-png]

summary.csv columns: the eight required ones, then mean_rtf (see SPEC.md):
  world,n_episodes,success_rate,mean_time_s,std_time_s,mean_path_m,collision_count_total,
  timeout_count,mean_rtf
"""
import argparse
import csv
import math
import os
import sys

EPISODE_COLUMNS = [
    'world', 'episode', 'seed', 'success', 'time_to_goal_s', 'path_length_m',
    'collision_count', 'min_obstacle_distance_m', 'timed_out',
]
SUMMARY_COLUMNS = [
    'world', 'n_episodes', 'success_rate', 'mean_time_s', 'std_time_s', 'mean_path_m',
    'collision_count_total', 'timeout_count', 'mean_rtf',
]
RTF_COLUMNS = ['world', 'episode', 'rtf']


class SummaryError(Exception):
    """Raised for any inconsistency; main() turns it into a non-zero exit."""


def parse_bool(text, where):
    t = text.strip().lower()
    if t in ('true', '1'):
        return True
    if t in ('false', '0'):
        return False
    raise SummaryError(f'{where}: not a boolean: {text!r}')


def parse_float(text, where, allow_nan=False):
    try:
        v = float(text)
    except ValueError:
        raise SummaryError(f'{where}: not a number: {text!r}')
    if math.isnan(v):
        if allow_nan:
            return v
        raise SummaryError(f'{where}: NaN not allowed here')
    if math.isinf(v):
        raise SummaryError(f'{where}: infinite value')
    return v


def parse_int(text, where):
    try:
        return int(text)
    except ValueError:
        raise SummaryError(f'{where}: not an integer: {text!r}')


def read_rows(path, columns):
    if not os.path.isfile(path):
        raise SummaryError(f'missing file: {path}')
    with open(path, newline='') as fh:
        reader = csv.reader(fh)
        try:
            header = next(reader)
        except StopIteration:
            raise SummaryError(f'{path}: empty file')
        if header != columns:
            raise SummaryError(
                f'{path}: header mismatch.\n  expected: {",".join(columns)}\n  found:    {",".join(header)}')
        rows = []
        for lineno, raw in enumerate(reader, start=2):
            if not raw:
                continue  # blank line
            if len(raw) != len(columns):
                raise SummaryError(
                    f'{path}:{lineno}: expected {len(columns)} fields, found {len(raw)}: {raw}')
            for name, cell in zip(columns, raw):
                if cell.strip() == '':
                    raise SummaryError(f'{path}:{lineno}: empty value in column {name!r}')
            rows.append((lineno, dict(zip(columns, raw))))
    return rows


def load_episodes(path):
    episodes = []
    for lineno, r in read_rows(path, EPISODE_COLUMNS):
        w = f'{path}:{lineno}'
        e = {
            'world': r['world'].strip(),
            'episode': parse_int(r['episode'], f'{w} episode'),
            'seed': parse_int(r['seed'], f'{w} seed'),
            'success': parse_bool(r['success'], f'{w} success'),
            'timed_out': parse_bool(r['timed_out'], f'{w} timed_out'),
            'path_length_m': parse_float(r['path_length_m'], f'{w} path_length_m'),
            'collision_count': parse_int(r['collision_count'], f'{w} collision_count'),
            'min_obstacle_distance_m': parse_float(
                r['min_obstacle_distance_m'], f'{w} min_obstacle_distance_m'),
        }
        # time_to_goal_s is NaN for episodes that did not reach the goal
        e['time_to_goal_s'] = parse_float(
            r['time_to_goal_s'], f'{w} time_to_goal_s', allow_nan=True)
        if e['success'] and math.isnan(e['time_to_goal_s']):
            raise SummaryError(f'{w}: success=true but time_to_goal_s is NaN')
        if (not e['success']) and not math.isnan(e['time_to_goal_s']):
            raise SummaryError(f'{w}: success=false but time_to_goal_s is {r["time_to_goal_s"]}')
        if e['success'] and e['timed_out']:
            raise SummaryError(f'{w}: an episode cannot be both success and timed_out')
        if e['collision_count'] < 0 or e['path_length_m'] < 0:
            raise SummaryError(f'{w}: negative count/length')
        episodes.append(e)
    return episodes


def load_rtf(path):
    rtf = {}
    for lineno, r in read_rows(path, RTF_COLUMNS):
        w = f'{path}:{lineno}'
        key = (r['world'].strip(), parse_int(r['episode'], f'{w} episode'))
        value = parse_float(r['rtf'], f'{w} rtf')
        if value <= 0:
            raise SummaryError(f'{w}: non-positive rtf')
        if key in rtf:
            raise SummaryError(f'{w}: duplicate rtf row for {key}')
        rtf[key] = value
    return rtf


def mean(xs):
    return sum(xs) / len(xs)


def sample_std(xs):
    if len(xs) < 2:
        return float('nan')
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def summarize(episodes, rtf, expected, worlds):
    by_world = {}
    for e in episodes:
        by_world.setdefault(e['world'], []).append(e)

    missing = [w for w in worlds if w not in by_world]
    if missing:
        raise SummaryError(f'no episodes for expected world(s): {missing}')
    extra = [w for w in by_world if w not in worlds]
    if extra:
        raise SummaryError(f'unexpected world(s) in episodes file: {extra}')

    out = []
    for world in worlds:
        eps = sorted(by_world[world], key=lambda e: e['episode'])
        if len(eps) != expected:
            raise SummaryError(
                f'world {world!r}: n_episodes = {len(eps)} but {expected} expected '
                f'(refusing to write a partial summary)')
        numbers = [e['episode'] for e in eps]
        if numbers != list(range(1, expected + 1)):
            raise SummaryError(f'world {world!r}: episode numbers are {numbers}, expected 1..{expected}')
        if len({e['seed'] for e in eps}) != len(eps):
            raise SummaryError(f'world {world!r}: duplicate seeds')
        rtfs = []
        for e in eps:
            key = (world, e['episode'])
            if key not in rtf:
                raise SummaryError(f'rtf log has no row for {key}')
            rtfs.append(rtf[key])
        times = [e['time_to_goal_s'] for e in eps if e['success']]
        out.append({
            'world': world,
            'n_episodes': len(eps),
            'success_rate': sum(e['success'] for e in eps) / len(eps),
            'mean_time_s': mean(times) if times else float('nan'),
            'std_time_s': sample_std(times),
            'mean_path_m': mean([e['path_length_m'] for e in eps]),
            'collision_count_total': sum(e['collision_count'] for e in eps),
            'timeout_count': sum(e['timed_out'] for e in eps),
            'mean_rtf': mean(rtfs),
        })
    return out


def fmt(v):
    if isinstance(v, float):
        return 'nan' if math.isnan(v) else f'{v:.4f}'
    return str(v)


def write_summary(rows, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', newline='') as fh:
        w = csv.writer(fh, lineterminator='\n')
        w.writerow(SUMMARY_COLUMNS)
        for r in rows:
            w.writerow([fmt(r[c]) for c in SUMMARY_COLUMNS])
    os.replace(tmp, path)


def write_png(rows, episodes, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    worlds = [r['world'] for r in rows]
    colors = {'simple': '#2a7ab0', 'cluttered': '#c8553d'}
    fig, axes = plt.subplots(1, 4, figsize=(15, 3.8))

    ax = axes[0]
    ax.bar(worlds, [100 * r['success_rate'] for r in rows], color=[colors.get(w, '#777') for w in worlds])
    ax.set_ylim(0, 118)
    ax.set_ylabel('success rate [%]')
    ax.set_title('Success rate')
    for i, r in enumerate(rows):
        ax.text(i, 100 * r['success_rate'] + 2, f"{100 * r['success_rate']:.0f}%  (n={r['n_episodes']})", ha='center')

    ax = axes[1]
    means = [0 if math.isnan(r['mean_time_s']) else r['mean_time_s'] for r in rows]
    stds = [0 if math.isnan(r['std_time_s']) else r['std_time_s'] for r in rows]
    ax.bar(worlds, means, yerr=stds, capsize=5, color=[colors.get(w, '#777') for w in worlds])
    ax.set_ylabel('time to goal [s]')
    ax.set_title('Time to goal (successes), mean ± std')

    ax = axes[2]
    xs = range(len(rows))
    ax.bar([x - 0.2 for x in xs], [r['collision_count_total'] for r in rows], 0.4, label='collisions', color='#555')
    ax.bar([x + 0.2 for x in xs], [r['timeout_count'] for r in rows], 0.4, label='timeouts', color='#d9a441')
    ax.set_xticks(list(xs))
    ax.set_xticklabels(worlds)
    top = max([1] + [r['collision_count_total'] for r in rows] + [r['timeout_count'] for r in rows])
    ax.set_ylim(0, top * 1.25 + 0.5)
    for i, r in enumerate(rows):
        ax.text(i - 0.2, r['collision_count_total'] + 0.03 * top, str(r['collision_count_total']), ha='center')
        ax.text(i + 0.2, r['timeout_count'] + 0.03 * top, str(r['timeout_count']), ha='center')
    ax.set_title('Collisions / timeouts (total)')
    ax.legend(loc='upper left')

    ax = axes[3]
    for w in worlds:
        pts = sorted((e['episode'], e['time_to_goal_s']) for e in episodes if e['world'] == w and e['success'])
        ax.plot([p[0] for p in pts], [p[1] for p in pts], 'o-', ms=4, label=w, color=colors.get(w, '#777'))
        fails = [e['episode'] for e in episodes if e['world'] == w and not e['success']]
        ax.plot(fails, [0] * len(fails), 'x', color='red')
    ax.set_xlabel('episode')
    ax.set_ylabel('time to goal [s]  (x = failed)')
    ax.set_title('Per episode')
    ax.legend()

    fig.suptitle('ros2-nav-stack - simulated in Gazebo Classic (not real-robot results)', fontsize=10)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--episodes', default='results/episodes.csv')
    ap.add_argument('--rtf', default='results/rtf_log.csv')
    ap.add_argument('--out', default='results/summary.csv')
    ap.add_argument('--png', default='results/summary.png')
    ap.add_argument('--expected', type=int, default=20, help='episodes required per world')
    ap.add_argument('--worlds', nargs='+', default=['simple', 'cluttered'])
    ap.add_argument('--no-png', action='store_true')
    args = ap.parse_args(argv)

    try:
        episodes = load_episodes(args.episodes)
        rtf = load_rtf(args.rtf)
        rows = summarize(episodes, rtf, args.expected, args.worlds)
    except SummaryError as exc:
        print(f'summarize.py: FAILED: {exc}', file=sys.stderr)
        return 2

    write_summary(rows, args.out)
    print(f'wrote {args.out}')
    for r in rows:
        print('  ' + ', '.join(f'{c}={fmt(r[c])}' for c in SUMMARY_COLUMNS))
    if not args.no_png:
        try:
            write_png(rows, episodes, args.png)
            print(f'wrote {args.png}')
        except ImportError as exc:
            print(f'summarize.py: summary.csv written but the PNG could not be produced: {exc}', file=sys.stderr)
            return 3
    return 0


if __name__ == '__main__':
    sys.exit(main())
