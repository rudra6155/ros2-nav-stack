#!/usr/bin/env python3
"""Root-cause helper for close calls: WHEN did the lidar first see the obstacle the robot later passed
closest to, and how far away was the robot then?

usage: analyze_first_seen.py WORLD EPISODE [EPISODE...]        (inside the container; needs the rosbag)

For each episode: (1) the obstacle nearest to the robot at its closest approach (ground-truth
geometry), (2) the first scan in the episode with a return landing on that obstacle's surface,
(3) the robot-obstacle distance at that moment, and (4) the clearance at closest approach.
Appends to results/analysis/first_seen.txt.
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_close_calls as acc  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402


def named_world(name):
    obs = []
    for line in open(os.path.join(acc.ROOT, 'worlds', name + '.obstacles')):
        p = line.split()
        if not p or p[0].startswith('#') or p[0] == 'arena':
            continue
        if p[0] == 'box':
            obs.append((p[1], ('box',) + tuple(float(x) for x in p[2:7])))
        else:
            obs.append((p[1], ('cyl',) + tuple(float(x) for x in p[2:5])))
    return obs


def read_bag_all(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {'/odom': [], '/goal_pose': [], '/scan': []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in msgs:
            msgs[topic].append(deserialize_message(data, get_message(types[topic])))
    return msgs


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    world = sys.argv[1]
    episodes = [int(x) for x in sys.argv[2:]]
    named = named_world(world)
    obs_only = [o for _, o in named]
    diag = {int(r['episode']): r for r in csv.DictReader(open(os.path.join(acc.R, 'episode_diagnostics.csv'))) if r['world'] == world}
    m = read_bag_all(os.path.join(acc.R, 'bags', f'eval_{world}'))
    goals = [stamp(g) for g in m['/goal_pose']]
    os.makedirs(os.path.join(acc.R, 'analysis'), exist_ok=True)
    out = open(os.path.join(acc.R, 'analysis', 'first_seen.txt'), 'a')
    for ep in episodes:
        g0 = goals[ep - 1]
        g1 = g0 + float(diag[ep]['duration_s'])
        odom = [(stamp(o), o.pose.pose.position.x, o.pose.pose.position.y,
                 2 * math.atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w)) for o in m['/odom'] if g0 <= stamp(o) <= g1]
        # closest approach and the obstacle responsible
        best = min(odom, key=lambda o: acc.sdf(obs_only, o[1], o[2]))
        dists = [(acc.sdf([o], best[1], best[2]), n, o) for n, o in named]
        _, name, shape = min(dists, key=lambda d: d[0])
        clearance = acc.sdf([shape], best[1], best[2])
        # first scan return on that obstacle
        first = None
        for s in m['/scan']:
            ts = stamp(s)
            if ts < g0 - 2.0 or ts > g1:
                continue
            pose = min(odom, key=lambda o: abs(o[0] - ts)) if odom else None
            if pose is None or abs(pose[0] - ts) > 0.2:
                continue
            for i, r in enumerate(s.ranges):
                if not (s.range_min <= r < s.range_max):
                    continue
                a = pose[3] + s.angle_min + i * s.angle_increment
                px, py = pose[1] + r * math.cos(a), pose[2] + r * math.sin(a)
                if acc.sdf([shape], px, py) < 0.12:
                    first = (ts - g0, math.hypot(pose[1] - (shape[1]), pose[2] - (shape[2])), r)
                    break
            if first:
                break
        line = (f'{world} ep {ep} (seed {diag[ep]["seed"]}): closest approach {clearance:.3f} m to {name} at t={best[0]-g0:.2f} s '
                f'(robot at {best[1]:.2f},{best[2]:.2f}); ')
        if first:
            line += (f'first lidar return on {name} at t={first[0]:.2f} s from range {first[2]:.2f} m '
                     f'(robot-centre to obstacle-centre {first[1]:.2f} m); {best[0]-g0-first[0]:.2f} s between first sight and closest approach')
        else:
            line += f'{name} was never hit by a lidar return within 0.12 m of its surface during the episode'
        print(line)
        out.write(line + '\n')
        import json
        jp = os.path.join(acc.R, 'analysis', 'first_seen.json')
        data = json.load(open(jp)) if os.path.isfile(jp) else {}
        data[f'{world}_{ep}'] = {
            'obstacle': name, 'closest_approach_m': round(clearance, 3), 'closest_t_s': round(best[0] - g0, 2),
            'first_return_t_s': None if not first else round(first[0], 2),
            'first_return_range_m': None if not first else round(first[2], 2),
            'seconds_from_first_sight_to_closest_approach': None if not first else round(best[0] - g0 - first[0], 2)}
        json.dump(data, open(jp, 'w'), indent=2)
    out.close()


if __name__ == '__main__':
    main()
