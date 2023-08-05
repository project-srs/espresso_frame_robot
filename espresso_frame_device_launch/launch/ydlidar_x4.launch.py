
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return LaunchDescription([
        Node(package='ydlidar_ros2_driver',
             executable='ydlidar_ros2_driver_node',
             namespace='/device/front_laser',
             output='screen',
             emulate_tty=True,
             parameters=[os.path.join(
                 get_package_share_directory('espresso_frame_device_launch'), 'config', 'ydlidar_x4.yaml')],
             )
    ])
