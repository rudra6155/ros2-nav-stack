#!/usr/bin/env python3
"""GIF helper (display runs only): draw the current goal as a translucent yellow disc, radius = the 0.3 m
success radius, in the Gazebo client. Spawns it when /goal_pose arrives and removes it on /goal_cancel.
It is a visual-only static model (no collision), so it cannot influence the episode."""
import sys

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

SDF = """<sdf version='1.6'><model name='goal_marker'><static>true</static><pose>{x} {y} 0.011 0 0 0</pose>
<link name='l'><visual name='v'><geometry><cylinder><radius>0.3</radius><length>0.02</length></cylinder></geometry>
<material><ambient>1 0.85 0 1</ambient><diffuse>1 0.85 0 1</diffuse></material><transparency>0.3</transparency></visual></link></model></sdf>"""


def main():
    rclpy.init()
    n = rclpy.create_node('gif_goal_marker')
    spawn = n.create_client(SpawnEntity, '/spawn_entity')
    delete = n.create_client(DeleteEntity, '/delete_entity')

    def on_goal(m):
        req = SpawnEntity.Request()
        req.name = 'goal_marker'
        req.xml = SDF.format(x=m.pose.position.x, y=m.pose.position.y)
        spawn.call_async(req)

    def on_cancel(_):
        req = DeleteEntity.Request()
        req.name = 'goal_marker'
        delete.call_async(req)

    n.create_subscription(PoseStamped, '/goal_pose', on_goal, 10)
    n.create_subscription(Empty, '/goal_cancel', on_cancel, 10)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
