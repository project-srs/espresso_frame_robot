
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='/device/top_camera',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    "camera_name":"top_camera",
                    "video_device":"/dev/webcam",
                    "image_width": 640,
                    "image_height": 480,
                    "framerate":15.0,
                    "pixel_format":"mjpeg2rgb",
                    "frame_id":"top_camera_optical_link",
                    "camera_info_url":"file:///home/ubuntu/data/device/top_camera_640_480_info.yaml"
            	 }
            ],
        ),
    ])
