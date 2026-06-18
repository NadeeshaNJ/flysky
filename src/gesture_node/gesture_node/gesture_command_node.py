#!/usr/bin/env python3
"""gesture_command_node — recognises hand gestures from the Kinect depth feed.

Subscribes:  /kinect/depth/image_raw  (sensor_msgs/Image, 16UC1)
Publishes:   /gesture/tracking         (std_msgs/String) — command events

Pipeline: depth -> nearest-blob hand segmentation (hand_tracker) -> temporal
gesture classification (gesture_classifier) -> command event. Commands are
edge-triggered (published once when recognised, with a cooldown); the behavior
node holds the resulting state. See gesture_classifier.py for the vocabulary.

MediaPipe isn't available on ARM64, so this is a classical-CV pipeline driven by
the Kinect's depth image. Thresholds (declared as ROS params) need on-hand
calibration; set the ``debug`` param to log per-frame features.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from gesture_node.hand_tracker import analyse
from gesture_node.gesture_classifier import GestureClassifier


class GestureCommandNode(Node):
    def __init__(self):
        super().__init__('gesture_command_node')

        self.declare_parameter('depth_topic', '/kinect/depth/image_raw')
        self.declare_parameter('command_topic', '/gesture/tracking')
        self.declare_parameter('near_band', 60)         # depth slab width (raw units)
        self.declare_parameter('min_area_frac', 0.01)   # min hand size (image frac)
        self.declare_parameter('invert_depth', False)   # True if higher = closer
        self.declare_parameter('swipe_dx', 0.5)         # swipe sensitivity
        self.declare_parameter('debug', False)

        self.depth_topic = self.get_parameter('depth_topic').value
        command_topic = self.get_parameter('command_topic').value
        self.near_band = int(self.get_parameter('near_band').value)
        self.min_area_frac = float(self.get_parameter('min_area_frac').value)
        self.invert_depth = bool(self.get_parameter('invert_depth').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        self.classifier = GestureClassifier(swipe_dx=float(self.get_parameter('swipe_dx').value))

        self.sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.pub = self.create_publisher(String, command_topic, 10)
        self.get_logger().info(
            f'gesture_command_node up (depth): {self.depth_topic} -> {command_topic}')

    def on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as exc:
            self.get_logger().warn(f'depth convert failed: {exc}',
                                   throttle_duration_sec=5.0)
            return

        feat = analyse(depth, near_band=self.near_band,
                       min_area_frac=self.min_area_frac, invert=self.invert_depth)

        if self.debug and feat.present:
            self.get_logger().info(
                f'hand: fingers={feat.fingers} solidity={feat.solidity:.2f} '
                f'cx={feat.cx:+.2f} area={feat.area_frac:.3f}',
                throttle_duration_sec=0.3)

        t = self.get_clock().now().nanoseconds * 1e-9
        cmd = self.classifier.update(t, feat)
        if cmd:
            self.get_logger().info(f'gesture recognised -> {cmd}')
            self.pub.publish(String(data=cmd))


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
