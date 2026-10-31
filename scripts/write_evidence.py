#!/usr/bin/env python3
"""Write results/evidence/<name>.json - machine-readable records of test gate execution.

    write_evidence.py NAME pass|fail --cmd "<command>" [--details-file FILE] [--results-dir DIR]
"""
import argparse
import datetime
import json
import os
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('status', choices=['pass', 'fail'])
    ap.add_argument('--cmd', required=True)
    ap.add_argument('--details-file', default=None)
    ap.add_argument('--results-dir', default=os.environ.get('NAV_RESULTS_DIR', 'results'))
    a = ap.parse_args()
    details = ''
    if a.details_file and os.path.isfile(a.details_file):
        with open(a.details_file, errors='replace') as fh:
            details = fh.read()[-6000:]
    try:
        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                                cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()
    except Exception:
        commit = ''
    out = os.path.join(a.results_dir, 'evidence')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, a.name + '.json'), 'w') as fh:
        json.dump({
            'name': a.name,
            'status': a.status,
            'utc': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'command': a.cmd,
            'git_commit': commit,
            'details': details,
        }, fh, indent=2)
        fh.write('\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
