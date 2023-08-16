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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_device_launch'), '/launch/mavros.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_device_launch'), '/launch/feetech_turret.launch.py']),
        ),
        Node(
            package='espresso_frame_sample_app',
            executable='joy_to_twist_node',
            namespace='/app',
            name='turret_joy_node',
            remappings=[('joy', '/device/mavros/rc_joy/joy'), ('twist', '/device/head_turret/cmd_rate')],
            parameters=[os.path.join(get_package_share_directory(
                'espresso_frame_sample_app'), 'config', 'turret_joy.yaml')],
            emulate_tty=True,
        ),
    ])
