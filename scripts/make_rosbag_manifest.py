#!/usr/bin/env python3
"""Write results/rosbag_manifest.json from the rosbag2 directories in results/bags/.

The bags themselves are large and are NOT committed (results/bags/ is git-ignored); the manifest
records what was recorded, with sizes and checksums so a re-recorded or re-downloaded bag can be
verified against it. Reads each bag's metadata.yaml and hashes its data file(s).
"""
import datetime
import hashlib
import json
import os
import sys

import yaml


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('NAV_RESULTS_DIR', 'results')
    bags_dir = os.path.join(results, 'bags')
    bags = []
    if os.path.isdir(bags_dir):
        for name in sorted(os.listdir(bags_dir)):
            d = os.path.join(bags_dir, name)
            meta_path = os.path.join(d, 'metadata.yaml')
            if not os.path.isfile(meta_path):
                continue
            with open(meta_path) as fh:
                meta = yaml.safe_load(fh)['rosbag2_bagfile_information']
            files = []
            for f in meta.get('relative_file_paths', []):
                p = os.path.join(d, f)
                files.append({'file': f, 'bytes': os.path.getsize(p), 'sha256': sha256(p)})
            dur_ns = meta.get('duration', {}).get('nanoseconds', 0)
            bags.append({
                'name': name,
                'path': f'results/bags/{name}',
                'storage_identifier': meta.get('storage_identifier'),
                'duration_s': round(dur_ns / 1e9, 2),
                'message_count': meta.get('message_count'),
                'topics': [
                    {'name': t['topic_metadata']['name'], 'type': t['topic_metadata']['type'],
                     'messages': t['message_count']}
                    for t in meta.get('topics_with_message_count', [])],
                'files': files,
                'committed_to_git': False,
            })
    manifest = {
        'generated_utc': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'description': 'rosbag2 recordings of the evaluation runs (sim time). Bags are not committed '
                       '(size); regenerate with `ros2 run evaluation run_episodes --world <w> --episodes 20`.',
        'bags': bags,
    }
    with open(os.path.join(results, 'rosbag_manifest.json'), 'w') as fh:
        json.dump(manifest, fh, indent=2)
        fh.write('\n')
    print(f'{len(bags)} bag(s) in manifest')
    return 0 if bags else 1


if __name__ == '__main__':
    sys.exit(main())
