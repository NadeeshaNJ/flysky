"""Full QBot bringup: Kinect driver, vision, gesture, and behavior nodes.

    ros2 launch behavior_node qbot.launch.py
    ros2 launch behavior_node qbot.launch.py use_kinect:=false   # use a bag/other source

The Kobuki base driver is launched separately and folded in here once the robot
is wired up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_kinect = LaunchConfiguration('use_kinect')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_kinect', default_value='true',
            description='Start the Kinect RGB-D driver node.'),

        Node(
            package='kinect_camera', executable='kinect_rgbd',
            name='kinect_rgbd', output='screen',
            condition=IfCondition(use_kinect),
        ),
        Node(
            package='vision_node', executable='face_tracker_node',
            name='face_tracker_node', output='screen',
        ),
        Node(
            package='gesture_node', executable='gesture_command_node',
            name='gesture_command_node', output='screen',
        ),
        Node(
            package='behavior_node', executable='pet_behavior_node',
            name='pet_behavior_node', output='screen',
        ),
    ])
