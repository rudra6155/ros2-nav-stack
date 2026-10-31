"""Summarizer tests: correct numbers on synthetic data; hard failure on any inconsistency."""
import csv
import math
import os
import subprocess
import sys

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'summarize.py')
EP_HEADER = ['world', 'episode', 'seed', 'success', 'time_to_goal_s', 'path_length_m',
             'collision_count', 'min_obstacle_distance_m', 'timed_out']
SUMMARY_HEADER = ['world', 'n_episodes', 'success_rate', 'mean_time_s', 'std_time_s', 'mean_path_m',
                  'collision_count_total', 'timeout_count', 'mean_rtf']


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh, lineterminator='\n')
        w.writerow(header)
        w.writerows(rows)


def run(tmp_path, episodes, rtf=None, expected=4, worlds=('simple',), header=None, extra=()):
    ep = tmp_path / 'episodes.csv'
    rt = tmp_path / 'rtf_log.csv'
    out = tmp_path / 'summary.csv'
    write_csv(ep, header or EP_HEADER, episodes)
    if rtf is None:
        rtf = [[r[0], r[1], '1.0'] for r in episodes]
    write_csv(rt, ['world', 'episode', 'rtf'], rtf)
    proc = subprocess.run(
        [sys.executable, SCRIPT, '--episodes', str(ep), '--rtf', str(rt), '--out', str(out),
         '--expected', str(expected), '--worlds', *worlds, '--no-png', *extra],
        capture_output=True, text=True)
    return proc, out


def good_rows():
    # times of the three successes: 10, 20, 30  -> mean 20, sample std 10
    return [
        ['simple', 1, 1001, 'true', 10.0, 5.0, 0, 0.60, 'false'],
        ['simple', 2, 1002, 'true', 20.0, 7.0, 1, 0.40, 'false'],
        ['simple', 3, 1003, 'false', 'nan', 9.0, 2, 0.14, 'true'],
        ['simple', 4, 1004, 'true', 30.0, 9.0, 0, 0.70, 'false'],
    ]


def test_correct_statistics(tmp_path):
    rtf = [['simple', 1, 0.9], ['simple', 2, 1.1], ['simple', 3, 1.0], ['simple', 4, 1.0]]
    proc, out = run(tmp_path, good_rows(), rtf=rtf)
    assert proc.returncode == 0, proc.stderr
    with open(out) as fh:
        rows = list(csv.reader(fh))
    assert rows[0] == SUMMARY_HEADER  # exact column names and order
    r = dict(zip(rows[0], rows[1]))
    assert r['world'] == 'simple'
    assert int(r['n_episodes']) == 4
    assert float(r['success_rate']) == pytest.approx(0.75)
    assert float(r['mean_time_s']) == pytest.approx(20.0)        # over successes only
    assert float(r['std_time_s']) == pytest.approx(10.0)         # sample std (n-1)
    assert float(r['mean_path_m']) == pytest.approx(7.5)         # over all episodes
    assert int(r['collision_count_total']) == 3
    assert int(r['timeout_count']) == 1
    assert float(r['mean_rtf']) == pytest.approx(1.0)


def test_failed_episodes_stay_in_the_denominator(tmp_path):
    rows = good_rows()
    for r in rows:
        r[3], r[4], r[8] = 'false', 'nan', 'true'
    proc, out = run(tmp_path, rows)
    assert proc.returncode == 0, proc.stderr
    with open(out) as fh:
        r = dict(zip(*list(csv.reader(fh))))
    assert float(r['success_rate']) == 0.0
    assert int(r['n_episodes']) == 4
    assert int(r['timeout_count']) == 4
    assert r['mean_time_s'] == 'nan'


def test_missing_column_in_header_is_a_hard_fail(tmp_path):
    header = [c for c in EP_HEADER if c != 'min_obstacle_distance_m']
    rows = [[c for i, c in enumerate(r) if i != 7] for r in good_rows()]
    proc, out = run(tmp_path, rows, header=header)
    assert proc.returncode != 0
    assert 'header mismatch' in proc.stderr
    assert not out.exists()  # nothing written


def test_missing_column_in_a_row_is_a_hard_fail(tmp_path):
    rows = good_rows()
    rows[2] = rows[2][:-1]  # drop timed_out from one row
    proc, out = run(tmp_path, rows)
    assert proc.returncode != 0
    assert 'expected 9 fields' in proc.stderr
    assert not out.exists()


def test_empty_cell_is_a_hard_fail(tmp_path):
    rows = good_rows()
    rows[1][5] = ''
    proc, out = run(tmp_path, rows)
    assert proc.returncode != 0
    assert 'empty value' in proc.stderr
    assert not out.exists()


def test_wrong_episode_count_is_a_hard_fail(tmp_path):
    proc, out = run(tmp_path, good_rows()[:3], expected=4)
    assert proc.returncode != 0
    assert 'n_episodes = 3 but 4 expected' in proc.stderr
    assert not out.exists()


def test_too_many_episodes_is_a_hard_fail(tmp_path):
    rows = good_rows() + [['simple', 5, 1005, 'true', 12.0, 5.0, 0, 0.5, 'false']]
    proc, _ = run(tmp_path, rows, expected=4)
    assert proc.returncode != 0


def test_missing_world_is_a_hard_fail(tmp_path):
    proc, out = run(tmp_path, good_rows(), worlds=('simple', 'cluttered'))
    assert proc.returncode != 0
    assert 'cluttered' in proc.stderr
    assert not out.exists()


def test_missing_rtf_row_is_a_hard_fail(tmp_path):
    rtf = [['simple', 1, 1.0], ['simple', 2, 1.0], ['simple', 3, 1.0]]
    proc, out = run(tmp_path, good_rows(), rtf=rtf)
    assert proc.returncode != 0
    assert 'rtf' in proc.stderr
    assert not out.exists()


def test_inconsistent_success_and_time_is_a_hard_fail(tmp_path):
    rows = good_rows()
    rows[0][4] = 'nan'  # success=true but no time
    proc, _ = run(tmp_path, rows)
    assert proc.returncode != 0
    rows = good_rows()
    rows[2][4] = '55.0'  # success=false but has a time
    proc, _ = run(tmp_path, rows)
    assert proc.returncode != 0


def test_non_numeric_value_is_a_hard_fail(tmp_path):
    rows = good_rows()
    rows[0][5] = 'lots'
    proc, _ = run(tmp_path, rows)
    assert proc.returncode != 0
    assert 'not a number' in proc.stderr


def test_duplicate_or_gappy_episode_numbers_are_a_hard_fail(tmp_path):
    rows = good_rows()
    rows[3][1] = 3  # episodes 1,2,3,3
    rows[3][2] = 1003 + 1
    proc, _ = run(tmp_path, rows)
    assert proc.returncode != 0
