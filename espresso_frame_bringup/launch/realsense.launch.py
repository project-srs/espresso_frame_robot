import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace='/device',
            name='head_camera',
            executable='realsense2_camera_node',
            parameters=[os.path.join(get_package_share_directory(
                'espresso_frame_bringup'), 'config', 'realsense.yaml')],
            emulate_tty=True,
            output='screen',
        ),
    ])
