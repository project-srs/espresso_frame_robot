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
from launch.actions import TimerAction
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay', default_value='0.0'
        ),
        TimerAction(period=LaunchConfiguration('delay'),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', str(get_package_share_path('espresso_frame_bringup') / 'rviz/robot.rviz')],
                ),
            ]),
    ])