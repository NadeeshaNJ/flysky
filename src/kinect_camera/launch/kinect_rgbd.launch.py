"""Launch the Kinect RGB-D driver node on its own."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinect_camera',
            executable='kinect_rgbd',
            name='kinect_rgbd',
            output='screen',
            parameters=[{
                'rgb_topic': '/kinect/rgb/image_raw',
                'depth_topic': '/kinect/depth/image_raw',
                'frame_rate': 30.0,
            }],
        ),
    ])
