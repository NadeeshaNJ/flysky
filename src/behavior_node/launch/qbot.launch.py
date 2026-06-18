"""Full QBot bringup: Kobuki base, Kinect driver, vision, gesture, behavior.

    ros2 launch behavior_node qbot.launch.py
    ros2 launch behavior_node qbot.launch.py use_kinect:=false   # use a bag/other source
    ros2 launch behavior_node qbot.launch.py use_base:=false      # no robot (desk testing)

Wiring:
  pet_behavior_node --(/commands/velocity, Twist)--> kobuki base
  kobuki base        --(/odom, /sensors/core, ...)--> (telemetry)

The Kobuki driver subscribes to ``/commands/velocity`` (not ``/cmd_vel``), so the
behavior node is pointed straight at that topic via its ``cmd_vel_topic`` param.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _kobuki_params():
    share = get_package_share_directory('kobuki_node')
    with open(os.path.join(share, 'config', 'kobuki_node_params.yaml')) as f:
        return yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']


def generate_launch_description():
    use_kinect = LaunchConfiguration('use_kinect')
    use_base = LaunchConfiguration('use_base')
    use_face = LaunchConfiguration('use_face')
    linear_speed = LaunchConfiguration('linear_speed')
    turn_speed = LaunchConfiguration('turn_speed')

    return LaunchDescription([
        DeclareLaunchArgument('use_kinect', default_value='true',
                              description='Start the Kinect RGB-D driver node.'),
        DeclareLaunchArgument('use_base', default_value='true',
                              description='Start the Kobuki base driver.'),
        DeclareLaunchArgument('use_face', default_value='false',
                              description='Start the face tracker (off: nothing '
                                          'consumes it yet, and it costs CPU/frame rate).'),
        DeclareLaunchArgument('linear_speed', default_value='0.12',
                              description='Drive speed m/s (lower = gentler).'),
        DeclareLaunchArgument('turn_speed', default_value='1.0',
                              description='Turn speed rad/s (lower = gentler).'),

        # Kobuki mobile base driver (subscribes /commands/velocity, publishes /odom).
        Node(
            package='kobuki_node', executable='kobuki_ros_node', name='kobuki',
            output='screen', parameters=[_kobuki_params()],
            condition=IfCondition(use_base),
        ),

        # Kinect RGB-D camera.
        Node(
            package='kinect_camera', executable='kinect_rgbd',
            name='kinect_rgbd', output='screen',
            condition=IfCondition(use_kinect),
        ),

        # Perception.
        Node(
            package='vision_node', executable='face_tracker_node',
            name='face_tracker_node', output='screen',
            condition=IfCondition(use_face),
        ),
        Node(
            package='gesture_node', executable='gesture_command_node',
            name='gesture_command_node', output='screen',
        ),

        # Behavior: drive the base on /commands/velocity, emit abstract cues on /qbot/sound.
        Node(
            package='behavior_node', executable='pet_behavior_node',
            name='pet_behavior_node', output='screen',
            parameters=[{
                'cmd_vel_topic': '/commands/velocity',
                'sound_topic': '/qbot/sound',
                'linear_speed': ParameterValue(linear_speed, value_type=float),
                'turn_speed': ParameterValue(turn_speed, value_type=float),
            }],
        ),
    ])
