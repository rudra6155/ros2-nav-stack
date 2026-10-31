#!/usr/bin/env python3
"""Audit every published frame_id against the URDF.

Subscribes to /scan, /odom, /costmap, /plan, /obstacles, /tf, /tf_static (and the camera image if
it is being published), publishes one goal so that /plan is non-empty, then checks that

  * each message's frame_id is the exact, expected frame ("laser", not "laser_link"),
  * every robot frame that appears in TF or in message headers is a link of the URDF
    (or one of the world frames map/odom),
  * every URDF link that must be in the TF tree is there.

    frame_audit.py --urdf-links "base_link laser camera ..." [--report results/frame_audit.txt]

Exit code 0 only if every check passes.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

WORLD_FRAMES = {'map', 'odom'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf-links', required=True, help='space separated link names from the URDF')
    ap.add_argument('--report', default=None)
    ap.add_argument('--timeout', type=float, default=40.0)
    args = ap.parse_args()
    urdf = set(args.urdf_links.split())

    rclpy.init()
    node = rclpy.create_node('frame_audit')
    got = {}
    tf_frames_dynamic, tf_frames_static = set(), set()

    def keep(name):
        def cb(msg):
            got.setdefault(name, msg)
        return cb

    node.create_subscription(LaserScan, '/scan', keep('scan'), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', keep('odom'), 10)
    node.create_subscription(OccupancyGrid, '/costmap', keep('costmap'), 1)
    node.create_subscription(PoseArray, '/obstacles', keep('obstacles'), 10)
    node.create_subscription(Image, '/camera/image_raw', keep('camera'), qos_profile_sensor_data)

    def plan_cb(msg):
        if msg.poses:  # only a non-empty plan carries frames worth checking
            got.setdefault('plan', msg)
    node.create_subscription(Path, '/plan', plan_cb, 10)

    def tf_cb(msg):
        for t in msg.transforms:
            tf_frames_dynamic.update([t.header.frame_id, t.child_frame_id])
    node.create_subscription(TFMessage, '/tf', tf_cb, 100)

    def tf_static_cb(msg):
        for t in msg.transforms:
            tf_frames_static.update([t.header.frame_id, t.child_frame_id])
    node.create_subscription(
        TFMessage, '/tf_static', tf_static_cb,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))

    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    deadline = time.time() + args.timeout
    last_goal = 0.0
    while time.time() < deadline and not {'scan', 'odom', 'costmap', 'obstacles', 'plan'} <= set(got):
        rclpy.spin_once(node, timeout_sec=0.2)
        if 'costmap' in got and 'odom' in got and time.time() - last_goal > 2.0:
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.pose.position.x = got['odom'].pose.pose.position.x + 3.0
            g.pose.position.y = got['odom'].pose.pose.position.y
            g.pose.orientation.w = 1.0
            goal_pub.publish(g)
            last_goal = time.time()
    # let TF accumulate a little more
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    lines, failures = [], []

    def check(ok, text):
        lines.append(('PASS  ' if ok else 'FAIL  ') + text)
        if not ok:
            failures.append(text)

    def frame_of(name):
        m = got.get(name)
        return None if m is None else m.header.frame_id

    check('scan' in got, 'received /scan')
    check(frame_of('scan') == 'laser', f"/scan header.frame_id == 'laser' (got {frame_of('scan')!r})")
    check(frame_of('scan') in urdf, f"/scan frame is a URDF link ({sorted(urdf)})")
    check('odom' in got, 'received /odom')
    if 'odom' in got:
        check(got['odom'].header.frame_id == 'odom', f"/odom header.frame_id == 'odom' (got {got['odom'].header.frame_id!r})")
        check(got['odom'].child_frame_id == 'base_link', f"/odom child_frame_id == 'base_link' (got {got['odom'].child_frame_id!r})")
        check(got['odom'].child_frame_id in urdf, '/odom child frame is a URDF link')
    for name in ('costmap', 'obstacles', 'plan'):
        check(name in got, f'received /{name}' + (' (non-empty)' if name == 'plan' else ''))
        check(frame_of(name) == 'map', f"/{name} header.frame_id == 'map' (got {frame_of(name)!r})")

    all_tf = tf_frames_dynamic | tf_frames_static
    robot_tf = all_tf - WORLD_FRAMES
    check(bool(all_tf), 'received TF')
    check(robot_tf <= urdf, f'every robot frame in TF is a URDF link (extra: {sorted(robot_tf - urdf)})')
    check('laser_link' not in all_tf and 'laser_link' not in urdf, "no 'laser_link' anywhere (frame name is exactly 'laser')")
    for needed in ('base_link', 'laser', 'camera'):
        check(needed in all_tf, f"frame '{needed}' is in the TF tree")
    check({'map', 'odom'} <= all_tf, "frames 'map' and 'odom' are in the TF tree")
    if 'camera' in got:
        check(got['camera'].header.frame_id == 'camera', "camera image frame_id == 'camera'")
    else:
        lines.append("SKIP  camera image frame (the Gazebo camera sensor is disabled in headless runs; "
                     "'camera' still exists in the URDF and TF)")

    lines.append('')
    lines.append(f'URDF links      : {sorted(urdf)}')
    lines.append(f'TF frames       : {sorted(all_tf)}')
    text = '\n'.join(lines)
    print(text)
    if args.report:
        with open(args.report, 'w') as fh:
            fh.write(text + '\n')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
