#!/usr/bin/env python3
"""Wait until every node in a list exists in the ROS graph.

    wait_for_nodes.py --nodes scripts/expected_nodes.txt --timeout 60 [--t0 <epoch seconds>]

Polls rclpy's graph (fast, no CLI daemon). Prints the elapsed time since --t0 (default: since
this script started) at which the last missing node appeared. Exit 0 if all present, 1 on timeout.
"""
import argparse
import sys
import time

import rclpy


def read_list(path):
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.split('#')[0].strip()
            if line:
                out.append(line)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nodes', required=True)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--t0', type=float, default=None)
    args = ap.parse_args()
    t0 = args.t0 if args.t0 is not None else time.time()
    expected = read_list(args.nodes)

    rclpy.init()
    node = rclpy.create_node('wait_for_nodes_probe')
    deadline = time.time() + args.timeout
    missing = list(expected)
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        present = {(ns.rstrip('/') + '/' + n) for n, ns in node.get_node_names_and_namespaces()}
        missing = [n for n in expected if n not in present]
        if not missing:
            print(f'ALL {len(expected)} nodes present after {time.time() - t0:.1f} s')
            node.destroy_node()
            rclpy.shutdown()
            return 0
    print(f'TIMEOUT after {time.time() - t0:.1f} s; missing: {missing}')
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
