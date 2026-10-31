#!/usr/bin/env python3
"""What does the stack do after a collision? (root-cause helper for the `stuck` failure mode)

usage: NAV_BAG=<bag dir> analyze_stuck.py WORLD EPISODE KEY      (inside the container)

From a rosbag of one episode: when the first chassis contact happens and where; what the controller
commands afterwards (/cmd_vel), how far the robot actually moves, and whether the planner still
publishes non-empty plans. Writes results/analysis/stuck.json[KEY].
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_close_calls as acc  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    world, ep, key = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    obs = acc.load_world(world)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.environ['NAV_BAG'], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom, cmd, plans, contacts, goals = [], [], [], [], []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in ('/odom', '/cmd_vel', '/plan', '/bumper_states', '/goal_pose'):
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/odom':
            odom.append((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)))
        elif topic == '/cmd_vel':
            cmd.append((ts * 1e-9, m.linear.x, m.angular.z))
        elif topic == '/plan':
            plans.append((stamp(m), len(m.poses)))
        elif topic == '/bumper_states':
            if m.states:
                contacts.append(stamp(m))
        elif topic == '/goal_pose':
            goals.append(stamp(m))
    g0 = goals[-1]
    odom = [o for o in odom if o[0] >= g0]
    if not contacts:
        res = {'collided': False}
    else:
        t_c = min(c for c in contacts if c >= g0)
        at = min(odom, key=lambda o: abs(o[0] - t_c))
        after = [o for o in odom if o[0] >= t_c]
        last = after[-1]
        far = [o for o in after if o[0] - t_c > 5.0]
        disp_after = math.hypot(last[1] - at[1], last[2] - at[2])
        # wall-clock stamps of /cmd_vel (bag time) vs sim stamps of /odom: compare counts of "pushing" commands
        pushing = [c for c in cmd if c[1] > 0.05]
        res = {
            'collided': True, 'first_contact_t_s': round(t_c - g0, 2),
            'contact_pose': [round(at[1], 2), round(at[2], 2), round(at[3], 2)],
            'clearance_at_contact_m': round(acc.sdf(obs, at[1], at[2]), 3),
            'contact_messages': len(contacts),
            'odom_samples_after_contact': len(after), 'observed_s_after_contact': round(last[0] - t_c, 1),
            'displacement_after_contact_m': round(disp_after, 3),
            'max_speed_after_5s_mps': round(max([math.hypot(far[i + 1][1] - far[i][1], far[i + 1][2] - far[i][2]) / max(far[i + 1][0] - far[i][0], 1e-3) for i in range(len(far) - 1)] or [0.0]), 3),
            'cmd_vel_msgs': len(cmd), 'cmd_vel_forward_fraction': round(len(pushing) / max(len(cmd), 1), 3),
            'plan_msgs': len(plans), 'plan_nonempty_fraction': round(sum(1 for p in plans if p[1] > 0) / max(len(plans), 1), 3),
        }
    jp = os.path.join(acc.R, 'analysis', 'stuck.json')
    os.makedirs(os.path.dirname(jp), exist_ok=True)
    data = json.load(open(jp)) if os.path.isfile(jp) else {}
    data[key] = res
    json.dump(data, open(jp, 'w'), indent=2)
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()
