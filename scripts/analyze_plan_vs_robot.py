#!/usr/bin/env python3
"""Planner-vs-controller attribution for a close call.

usage: analyze_plan_vs_robot.py WORLD EPISODE [t_from t_to]       (inside the container; needs the rosbag)

For samples of the episode (default: whole episode, every 0.25 s) it prints
  * clearance of the robot centre to the nearest obstacle surface,
  * cross-track error of the robot w.r.t. the latest /plan,
  * the minimum clearance of the plan itself over the next 1.5 m of path from the robot,
so that a clearance dip can be attributed to the planner (the plan itself runs too close) or to the
controller (the plan is fine but the robot deviates from it).
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


BUBBLE = 0.35   # m of path ahead of the robot: the A* start-relaxation radius (0.30 m) plus a little


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    world, ep = sys.argv[1], int(sys.argv[2])
    obs = acc.load_world(world)
    diag = {int(r['episode']): r for r in csv.DictReader(open(os.environ.get('NAV_DIAG', os.path.join(acc.R, 'episode_diagnostics.csv')))) if r['world'] == world}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.environ.get('NAV_BAG', os.path.join(acc.R, 'bags', f'eval_{world}')), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom, plans, goals, scans = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/odom':
            m = deserialize_message(data, get_message(types[topic]))
            odom.append((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                         2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)))
        elif topic == '/plan':
            m = deserialize_message(data, get_message(types[topic]))
            plans.append((stamp(m), [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif topic == '/goal_pose':
            goals.append(stamp(deserialize_message(data, get_message(types[topic]))))
        elif topic == '/scan':
            m = deserialize_message(data, get_message(types[topic]))
            scans.append((stamp(m), m.angle_min, m.angle_increment, m.range_min, m.range_max, list(m.ranges)))
    g0 = goals[ep - 1] if len(goals) > 1 else goals[0]   # single-episode bags (stress re-runs) hold one goal
    g1 = g0 + float(diag[ep]['duration_s'])
    t_from = g0 + (float(sys.argv[3]) if len(sys.argv) > 3 else 0.0)
    t_to = g0 + (float(sys.argv[4]) if len(sys.argv) > 4 else (g1 - g0))
    print(f'{world} episode {ep} seed {diag[ep]["seed"]}:   t[s]   clearance  cross-track  plan-min-clearance(next 1.5 m)  speed-ish')
    t = t_from
    worst = None
    ep_max_xt, ep_min_plan, ep_min_far = 0.0, 1e9, 1e9
    while t <= t_to:
        o = min(odom, key=lambda s: abs(s[0] - t))
        pl = [p for p in plans if p[0] <= t and p[1]]
        if pl:
            pts = pl[-1][1]
            # closest plan point, then clearance of the plan over the next 1.5 m
            idx = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - o[1], pts[i][1] - o[2]))
            xt = min(math.hypot(pts[i][0] - o[1], pts[i][1] - o[2]) for i in range(len(pts)))
            ahead, arcs, acc_len, j = [], [], 0.0, idx
            while j + 1 < len(pts) and acc_len < 1.5:
                acc_len += math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
                ahead.append(pts[j + 1])
                arcs.append(acc_len)
                j += 1
            pmin = min([acc.sdf(obs, p[0], p[1]) for p in ahead] + [acc.sdf(obs, pts[idx][0], pts[idx][1])])
            far = [acc.sdf(obs, p[0], p[1]) for p, a_ in zip(ahead, arcs) if a_ >= BUBBLE]
            if far:
                ep_min_far = min(ep_min_far, min(far))
        else:
            xt, pmin = float('nan'), float('nan')
        c = acc.sdf(obs, o[1], o[2])
        if not math.isnan(xt):
            ep_max_xt = max(ep_max_xt, xt) if t - g0 > 0.5 else ep_max_xt
            ep_min_plan = min(ep_min_plan, pmin)
        flag = '  <-- closest' if worst is None or c < worst[0] else ''
        if flag:
            worst = (c, t - g0, xt, pmin)
        print(f'                       {t - g0:6.2f}   {c:7.3f}   {xt:9.3f}    {pmin:9.3f}{flag}')
        t += 0.25
    print(f'closest approach: clearance {worst[0]:.3f} m at t={worst[1]:.2f} s, cross-track to the plan then {worst[2]:.3f} m, '
          f'plan clearance ahead {worst[3]:.3f} m')
    # ---- was the obstacle surface nearest to the plan actually SEEN by the lidar when that plan was made?
    t_c = g0 + worst[1]
    pl = [p for p in plans if p[0] <= t_c and p[1]]
    unobserved_gap = float('nan')
    dist_robot_to_point = float('nan')
    if pl:
        pts = pl[-1][1]
        o = min(odom, key=lambda s: abs(s[0] - t_c))
        idx = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - o[1], pts[i][1] - o[2]))
        ahead = pts[idx:idx + 16]
        pmin = min(ahead, key=lambda q: acc.sdf(obs, q[0], q[1]))
        # nearest point Q on the true surface of the obstacle closest to that plan point
        best_o, best_d = None, 1e9
        for ob in obs:
            d = acc.sdf([ob], pmin[0], pmin[1])
            if d < best_d:
                best_o, best_d = ob, d
        if best_o[0] == 'cyl':
            _, cx, cy, r = best_o
            n = math.hypot(pmin[0] - cx, pmin[1] - cy) or 1e-9
            Q = (cx + r * (pmin[0] - cx) / n, cy + r * (pmin[1] - cy) / n)
        else:
            _, cx, cy, yaw, sx_, sy_ = best_o
            c_, s_ = math.cos(yaw), math.sin(yaw)
            lx, ly = c_ * (pmin[0] - cx) + s_ * (pmin[1] - cy), -s_ * (pmin[0] - cx) + c_ * (pmin[1] - cy)
            qx, qy = max(-sx_ / 2, min(sx_ / 2, lx)), max(-sy_ / 2, min(sy_ / 2, ly))
            Q = (cx + c_ * qx - s_ * qy, cy + s_ * qx + c_ * qy)
        near = 1e9
        for (ts, a0, da, rmin, rmax, ranges) in scans:
            if ts < g0 - 2.0 or ts > t_c:
                continue
            po = min(odom, key=lambda s: abs(s[0] - ts))
            if abs(po[0] - ts) > 0.2:
                continue
            yaw = po[3]
            for i, r in enumerate(ranges):
                if rmin <= r < rmax:
                    a = yaw + a0 + i * da
                    near = min(near, math.hypot(po[1] + r * math.cos(a) - Q[0], po[2] + r * math.sin(a) - Q[1]))
        unobserved_gap = near
        dist_robot_to_point = math.hypot(pmin[0] - o[1], pmin[1] - o[2])
        print(f'plan point nearest to an obstacle: ({pmin[0]:.2f}, {pmin[1]:.2f}), clearance to the true surface {best_d:.3f} m; '
              f'distance from the nearest true-surface point to the closest lidar return seen so far: {near:.3f} m; '
              f'that plan point is {dist_robot_to_point:.2f} m from the robot')
    import json
    os.makedirs(os.path.join(acc.R, 'analysis'), exist_ok=True)
    jp = os.path.join(acc.R, 'analysis', 'plan_vs_robot.json')
    data = json.load(open(jp)) if os.path.isfile(jp) else {}
    data[os.environ.get('NAV_ANALYSIS_KEY', f'{world}_{ep}')] = {
        'seed': int(diag[ep]['seed']), 'window_s': [round(t_from - g0, 2), round(t_to - g0, 2)],
        'closest_clearance_m': round(worst[0], 3), 'closest_t_s': round(worst[1], 2),
        'cross_track_at_closest_m': round(worst[2], 3), 'plan_clearance_ahead_at_closest_m': round(worst[3], 3),
        'max_cross_track_in_window_m': round(ep_max_xt, 3), 'min_plan_clearance_in_window_m': round(ep_min_plan, 3),
        'nearest_lidar_return_to_that_surface_point_m': None if math.isnan(unobserved_gap) else round(unobserved_gap, 3),
        'robot_to_min_clearance_plan_point_m': None if math.isnan(dist_robot_to_point) else round(dist_robot_to_point, 3),
        'min_plan_clearance_beyond_bubble_in_window_m': round(ep_min_far, 3) if ep_min_far < 1e8 else None}
    json.dump(data, open(jp, 'w'), indent=2)


if __name__ == '__main__':
    main()
