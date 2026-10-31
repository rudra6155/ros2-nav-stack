"""Gazebo Classic (gazebo11) + robot + navigation stack.

    ros2 launch bringup sim.launch.py world:=simple headless:=true

Brings up: gzserver (+ gzclient when headless:=false), robot_state_publisher, the robot spawn,
the static map->odom transform, and (with_stack:=true) perception, planner and controller.

Time contract: every node runs on /clock. `use_sim_time` is asserted here - the launch refuses to
start with use_sim_time:=false, because a wall-clock stack under Gazebo silently breaks TF lookups,
timers and message filters.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _assert_sim_time(context, *args, **kwargs):
    value = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    if value not in ('true', '1'):
        raise RuntimeError(
            "bringup requires use_sim_time:=true (got '%s'): this stack must not run on wall-clock "
            "time under Gazebo." % value)
    return [LogInfo(msg='use_sim_time asserted true for all nodes')]


def _gazebo(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    if not world.endswith('.world'):
        world = os.path.join(get_package_share_directory('bringup'), 'worlds', world + '.world')
    if not os.path.isfile(world):
        raise RuntimeError('world file not found: %s' % world)
    gazebo_ros = get_package_share_directory('gazebo_ros')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false', 'pause': 'false'}.items())]
    headless = LaunchConfiguration('headless').perform(context).strip().lower() in ('true', '1')
    if not headless:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))))
    return actions


def generate_launch_description():
    bringup = get_package_share_directory('bringup')
    description = os.path.join(
        get_package_share_directory('robot_description'), 'launch', 'description.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple',
                              description="'simple', 'cluttered' or a path to a .world file"),
        DeclareLaunchArgument('headless', default_value='true',
                              description='true: gzserver only (no GUI, no GL needed)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='must be true; asserted'),
        DeclareLaunchArgument('enable_camera', default_value='false',
                              description='enable the Gazebo RGB camera sensor (needs GL)'),
        DeclareLaunchArgument('lidar_frame', default_value='laser',
                              description='frame_id of /scan (fault injection only; must be "laser")'),
        DeclareLaunchArgument('with_stack', default_value='true',
                              description='also start perception/planner/controller'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('lookahead', default_value='0.6'),
        DeclareLaunchArgument('inflation_radius', default_value='0.30'),

        OpaqueFunction(function=_assert_sim_time),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),  # never reach for the network
        OpaqueFunction(function=_gazebo),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description),
            launch_arguments={'enable_camera': LaunchConfiguration('enable_camera'),
                              'lidar_frame': LaunchConfiguration('lidar_frame')}.items()),

        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_diffbot',
             output='screen',
             arguments=['-entity', 'diffbot', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', '0.06', '-Y', LaunchConfiguration('yaw')]),

        # odom is Gazebo's ground-truth world pose, so map and odom coincide.
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_odom_static_tf',
             output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0',
                        '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'stack.launch.py')),
            launch_arguments={'lookahead': LaunchConfiguration('lookahead'),
                              'inflation_radius': LaunchConfiguration('inflation_radius')}.items(),
            condition=IfCondition(LaunchConfiguration('with_stack'))),
    ])
