"""The navigation stack only (perception + planner + controller). Assumes /scan, /odom and TF are provided
(by sim.launch.py). Everything runs on simulation time."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('bringup'), 'config', 'stack_params.yaml')
    limits = os.path.join(
        get_package_share_directory('robot_description'), 'config', 'robot_limits.yaml')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('lookahead', default_value='0.6',
                              description='pure-pursuit lookahead distance [m]'),
        DeclareLaunchArgument('inflation_radius', default_value='0.30',
                              description='A* obstacle inflation radius [m]'),

        Node(package='perception', executable='obstacle_detector', name='obstacle_detector',
             output='screen', parameters=[params, sim_time]),
        Node(package='navigation', executable='astar_planner', name='astar_planner',
             output='screen',
             parameters=[params, sim_time, {
                 'inflation_radius': ParameterValue(
                     LaunchConfiguration('inflation_radius'), value_type=float)}]),
        Node(package='control', executable='waypoint_follower', name='waypoint_follower',
             output='screen',
             parameters=[params, limits, sim_time, {
                 'lookahead_distance': ParameterValue(
                     LaunchConfiguration('lookahead'), value_type=float)}]),
    ])
