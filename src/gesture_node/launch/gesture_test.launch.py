"""One-command gesture test: Kinect RGB camera + gesture recognizer (debug on).

    ros2 launch gesture_node gesture_test.launch.py

Brings up the Kinect driver and the gesture_command_node together so a single
Ctrl-C tears both down cleanly (launch forwards SIGINT to every child — this is
the safe way to stop the Kinect, which must never be kill -9'd or it leaves the
USB device claimed).

Args:
    debug:=true|false        per-frame feature logging (default true)
    mirror_horizontal:=true  flip turn_left/right if pointing is mirrored
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kinect_launch = os.path.join(
        get_package_share_directory('kinect_camera'),
        'launch', 'kinect_rgbd.launch.py')

    debug = LaunchConfiguration('debug')
    mirror = LaunchConfiguration('mirror_horizontal')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('mirror_horizontal', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinect_launch)),
        Node(
            package='gesture_node',
            executable='gesture_command_node',
            name='gesture_command_node',
            output='screen',
            parameters=[{
                'debug': debug,
                'mirror_horizontal': mirror,
            }],
        ),
    ])
