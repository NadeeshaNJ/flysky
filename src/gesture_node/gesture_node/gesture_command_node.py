#!/usr/bin/env python3
"""gesture_command_node — classifies hand gestures into robot command strings.

Subscribes:  /kinect/rgb/image_raw  (sensor_msgs/Image)
Publishes:   /gesture/tracking       (std_msgs/String)

Recognised command vocabulary (see QBOT_OVERVIEW.md):
    "beckon"        -> come closer / move forward
    "palm"          -> stop / hold position
    "circle"        -> rotate in place
    "wave_left"     -> step left
    "wave_right"    -> step right
    "wag"           -> oscillate left-right (tail wag)
    "none"          -> no gesture

This is a SCAFFOLD: the real classifier (OpenCV contour / landmark analysis, or a
lightweight hand-landmark model) is filled in once the Kinect is connected. For
now it loads frames, runs a placeholder, and publishes "none" so the graph wires
up end-to-end and the behavior node can be tested with replayed commands.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

GESTURES = ('beckon', 'palm', 'circle', 'wave_left', 'wave_right', 'wag', 'none')


class GestureCommandNode(Node):
    def __init__(self):
        super().__init__('gesture_command_node')

        self.declare_parameter('rgb_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('command_topic', '/gesture/tracking')

        rgb_topic = self.get_parameter('rgb_topic').value
        command_topic = self.get_parameter('command_topic').value

        self.bridge = CvBridge()
        self.last_command = 'none'

        self.sub = self.create_subscription(Image, rgb_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, command_topic, 10)
        self.get_logger().info(
            f'gesture_command_node up: {rgb_topic} -> {command_topic}')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge convert failed: {exc}',
                                   throttle_duration_sec=5.0)
            return

        command = self.classify(frame)
        if command != self.last_command:
            self.get_logger().info(f'gesture -> {command}')
            self.last_command = command
        self.pub.publish(String(data=command))

    def classify(self, frame) -> str:
        """Placeholder gesture classifier. Returns one of GESTURES.

        TODO: implement once the Kinect is attached — segment the hand (skin /
        depth mask), extract contour features, and map to a gesture in GESTURES.
        """
        return 'none'


def main(args=None):
    rclpy.init(args=args)
    node = GestureCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
