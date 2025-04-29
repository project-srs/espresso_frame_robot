from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      namespace='/device/front_laser',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'front_laser_link'},
        {'port_name': '/dev/ldlidar'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
    ),
    Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        namespace='/device/front_laser',
        parameters=[os.path.join(get_package_share_path("espresso_frame_bringup"), "config", "front_laser_filter.yaml")],
    )
  ])
