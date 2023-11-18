#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='srs_feetech_driver',
            executable='pan_tilt_node',
            namespace='/device/head_turret',
            parameters=[os.path.join(get_package_share_directory(
                'espresso_frame_bringup'), 'config', 'feetech_turret.yaml')],
            emulate_tty=True,
        ),
        Node(
            package='srs_feetech_driver',
            executable='joy_to_cmd_rate',
            namespace='/device/head_turret',
        ),
    ])
