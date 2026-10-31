"""Publish the robot's URDF on /robot_description and its fixed TF (robot_state_publisher)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'diffbot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' enable_camera:=', LaunchConfiguration('enable_camera'),
                 ' lidar_frame:=', LaunchConfiguration('lidar_frame')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='false',
                              description='enable the Gazebo RGB camera sensor (needs GL)'),
        DeclareLaunchArgument('lidar_frame', default_value='laser',
                              description='frame_id of /scan; must equal the URDF link "laser" (override only for fault injection)'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
    ])
