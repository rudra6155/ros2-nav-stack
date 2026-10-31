#!/usr/bin/env python3
"""Find WHERE and WHEN each episode comes closest to an obstacle, from the recorded rosbags.

For every episode: min ground-truth distance (centre -> nearest obstacle surface), the time since the
goal was published at which it occurs, the robot's speed/yaw-rate/heading at that moment and its
distance from the start. Inputs: results/bags/eval_<world>/, worlds/<world>.obstacles,
results/episode_diagnostics.csv.  Output: results/close_calls.csv (all episodes, sorted by distance).
Run inside the container (needs rosbag2_py).
"""
import csv
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = os.environ.get('NAV_RESULTS_DIR', os.path.join(ROOT, 'results'))


def load_world(name):
    obs = []
    for line in open(os.path.join(ROOT, 'worlds', name + '.obstacles')):
        p = line.split()
        if not p or p[0].startswith('#') or p[0] == 'arena':
            continue
        if p[0] == 'box':
            obs.append(('box',) + tuple(float(x) for x in p[2:7]))
        else:
            obs.append(('cyl',) + tuple(float(x) for x in p[2:5]))
    return obs


def sdf(obs, x, y):
    best = 1e9
    for o in obs:
        if o[0] == 'cyl':
            d = math.hypot(x - o[1], y - o[2]) - o[3]
        else:
            _, cx, cy, yaw, sx, sy = o
            c, s = math.cos(yaw), math.sin(yaw)
            lx, ly = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
            qx, qy = abs(lx) - sx / 2, abs(ly) - sy / 2
            d = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
        best = min(best, d)
    return best


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {'/odom': [], '/goal_pose': [], '/cmd_vel': []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in msgs:
            msgs[topic].append(deserialize_message(data, get_message(types[topic])))
    return msgs


def main():
    diag = {}
    for r in csv.DictReader(open(os.path.join(R, 'episode_diagnostics.csv'))):
        diag[(r['world'], int(r['episode']))] = r
    out = []
    for world in ('simple', 'cluttered'):
        bag = os.path.join(R, 'bags', f'eval_{world}')
        if not os.path.isdir(bag):
            print(f'skip {world}: no bag at {bag}')
            continue
        obs = load_world(world)
        m = read_bag(bag)
        goals = [g.header.stamp.sec + g.header.stamp.nanosec * 1e-9 for g in m['/goal_pose']]
        odom = [(o.header.stamp.sec + o.header.stamp.nanosec * 1e-9, o.pose.pose.position.x, o.pose.pose.position.y,
                 2 * math.atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w),
                 o.twist.twist.linear.x, o.twist.twist.angular.z) for o in m['/odom']]
        for k, g0 in enumerate(goals, start=1):
            d = diag.get((world, k))
            if d is None:
                continue
            g1 = g0 + float(d['duration_s'])
            seg = [o for o in odom if g0 <= o[0] <= g1]
            if not seg:
                continue
            best = min(seg, key=lambda o: sdf(obs, o[1], o[2]))
            sx, sy = seg[0][1], seg[0][2]
            out.append({
                'world': world, 'episode': k, 'seed': d['seed'],
                'min_dist_m': round(sdf(obs, best[1], best[2]), 3),
                't_since_goal_s': round(best[0] - g0, 2),
                'dist_from_start_m': round(math.hypot(best[1] - sx, best[2] - sy), 2),
                'speed_mps': round(best[4], 3), 'yaw_rate_radps': round(best[5], 3),
                'x': round(best[1], 2), 'y': round(best[2], 2),
            })
    out.sort(key=lambda r: r['min_dist_m'])
    with open(os.path.join(R, 'close_calls.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(out[0].keys()), lineterminator='\n')
        w.writeheader()
        w.writerows(out)
    print(f'wrote {R}/close_calls.csv ({len(out)} episodes)')
    for r in out[:10]:
        print(r)


if __name__ == '__main__':
    main()
