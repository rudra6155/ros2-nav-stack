#!/usr/bin/env python3
"""Fault injection B: goals the planner must refuse (inside a wall / outside the map).

With the stack running (sim.launch.py, world simple): teleport the robot next to the north wall so that
the wall is mapped, publish a goal INSIDE the wall and then one OUTSIDE the map, and observe for
--observe seconds each: the /plan messages (must all be empty), the largest |linear| commanded on /cmd_vel
(must be 0), how far the robot moved, and whether /astar_planner is still alive afterwards.
Prints one JSON object.  usage: fault_unreachable.py [--observe 10]
"""
import argparse
import json
import math
import time

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--observe', type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('fault_unreachable')
    st = {'odom': None, 'plans': [0, 0], 'max_v': 0.0}

    def odom(m):
        st['odom'] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def plan(m):
        st['plans'][0 if m.poses else 1] += 1  # [non-empty, empty]

    def cmd(m):
        st['max_v'] = max(st['max_v'], abs(m.linear.x))
    n.create_subscription(Odometry, '/odom', odom, 10)
    n.create_subscription(Path, '/plan', plan, 10)
    n.create_subscription(Twist, '/cmd_vel', cmd, 10)
    goal_pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
    cancel_pub = n.create_publisher(Empty, '/goal_cancel', 10)
    cli = n.create_client(SetEntityState, '/set_entity_state')
    while not cli.wait_for_service(timeout_sec=1.0):
        pass
    t0 = time.time()
    while st['odom'] is None and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)

    def spin(sec):
        t = time.time() + sec
        while time.time() < t:
            rclpy.spin_once(n, timeout_sec=0.05)

    req = SetEntityState.Request()
    req.state.name = 'diffbot'
    req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = 0.0, 4.6, 0.06
    req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)  # facing +y
    req.state.reference_frame = 'world'
    f = cli.call_async(req)
    while not f.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    spin(4.0)  # let the lidar map the wall (hits <= 6 m)

    out = {}
    for name, (gx, gy) in {'goal_inside_wall': (0.0, 6.1), 'goal_outside_map': (20.0, 0.0)}.items():
        cancel_pub.publish(Empty())
        spin(1.0)
        st['plans'], st['max_v'] = [0, 0], 0.0
        start = st['odom']
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y, g.pose.orientation.w = gx, gy, 1.0
        goal_pub.publish(g)
        spin(a.observe)
        end = st['odom']
        out[name] = {'goal': [gx, gy], 'plan_msgs_nonempty': st['plans'][0], 'plan_msgs_empty': st['plans'][1],
                     'max_cmd_linear_mps': round(st['max_v'], 4),
                     'robot_moved_m': round(math.hypot(end[0] - start[0], end[1] - start[1]), 4)}
    cancel_pub.publish(Empty())
    spin(1.0)
    names = {ns.rstrip('/') + '/' + nm for nm, ns in n.get_node_names_and_namespaces()}
    out['planner_alive_after'] = '/astar_planner' in names
    print(json.dumps(out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
