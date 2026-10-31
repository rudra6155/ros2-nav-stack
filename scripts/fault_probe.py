#!/usr/bin/env python3
"""Passive probe used by scripts/fault_injection.sh while a run_episodes experiment is running.

Records for --duration seconds (wall): the frame_id of /scan, the number of non-unknown and
occupied cells in /costmap (max over time), and the number of /plan messages (empty / non-empty).
Prints one JSON object.  usage: fault_probe.py --duration 60 [--wait 120]
"""
import argparse
import json
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--wait', type=float, default=120.0, help='max wall seconds to wait for the first /scan')
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('fault_probe')
    st = {'scan_frame': None, 'scan_msgs': 0, 'costmap_msgs': 0, 'max_known_cells': 0, 'max_occupied_cells': 0,
          'plan_msgs': 0, 'plan_empty': 0, 'plan_nonempty': 0}

    def scan(m):
        st['scan_frame'] = m.header.frame_id
        st['scan_msgs'] += 1

    def costmap(m):
        st['costmap_msgs'] += 1
        known = sum(1 for v in m.data if v >= 0)
        occ = sum(1 for v in m.data if v >= 65)
        st['max_known_cells'] = max(st['max_known_cells'], known)
        st['max_occupied_cells'] = max(st['max_occupied_cells'], occ)

    def plan(m):
        st['plan_msgs'] += 1
        st['plan_nonempty' if m.poses else 'plan_empty'] += 1

    n.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid, '/costmap', costmap, 1)
    n.create_subscription(Path, '/plan', plan, 10)
    t_wait = time.time() + a.wait
    while st['scan_msgs'] == 0 and time.time() < t_wait:
        rclpy.spin_once(n, timeout_sec=0.2)
    t_end = time.time() + a.duration
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(json.dumps(st))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
