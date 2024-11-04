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
        DeclareLaunchArgument(
            'sample_app', default_value='false'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_bringup'), '/launch/mavros.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_bringup'), '/launch/feetech_turret.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_bringup'), '/launch/realsense.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_bringup'), '/launch/ld19.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('espresso_frame_bringup'), '/launch/webcam.launch.py']),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', str(get_package_share_path('espresso_frame_bringup')) + '/urdf/all.urdf']))}]
        ),
        Node(
            package='espresso_frame_bringup',
            executable='sample_app.py',
            namespace='/app',
            condition=IfCondition(LaunchConfiguration('sample_app'))
        ),
    ])
