#!/usr/bin/env python3
"""gesture_command_node — recognises hand gestures from the Kinect RGB feed.

Subscribes:  /kinect/rgb/image_raw  (sensor_msgs/Image)
Publishes:   /gesture/tracking       (std_msgs/String) — command events

Pipeline: RGB -> ONNX palm detection + 21 hand landmarks (hand_tracker) ->
temporal gesture classification (gesture_classifier) -> command event. Commands
are edge-triggered (published once when recognised, with a cooldown); the
behavior node holds the resulting state. See gesture_classifier.py for the
vocabulary.

MediaPipe has no ARM64 wheel, so the landmark models run under onnxruntime (see
mp_models/ and models/). Set the ``debug`` param to log per-frame finger state.
The two models cost ~60-80 ms/frame on the Pi, so we process at a capped rate.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from gesture_node.hand_tracker import HandLandmarkTracker
from gesture_node.gesture_classifier import GestureClassifier


class GestureCommandNode(Node):
    def __init__(self):
        super().__init__('gesture_command_node')

        default_models = os.path.join(
            get_package_share_directory('gesture_node'), 'models')

        self.declare_parameter('rgb_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('command_topic', '/gesture/tracking')
        self.declare_parameter('model_dir', default_models)
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('process_hz', 12.0)   # cap inference rate
        self.declare_parameter('swipe_dx', 0.45)
        self.declare_parameter('debug', False)

        rgb_topic = self.get_parameter('rgb_topic').value
        command_topic = self.get_parameter('command_topic').value
        model_dir = self.get_parameter('model_dir').value
        self.min_period = 1.0 / float(self.get_parameter('process_hz').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        self.tracker = HandLandmarkTracker(
            model_dir,
            score_threshold=float(self.get_parameter('score_threshold').value),
            conf_threshold=float(self.get_parameter('conf_threshold').value))
        self.classifier = GestureClassifier(
            swipe_dx=float(self.get_parameter('swipe_dx').value))

        # Keep-last-1, best-effort: the transport holds only the freshest frame so
        # we never build a backlog (ONNX inference is slower than the frame rate).
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._latest = None
        self.sub = self.create_subscription(Image, rgb_topic, self.on_rgb, qos)
        self.pub = self.create_publisher(String, command_topic, 10)
        # Process on a timer so we always run on the most recent frame and drop any
        # intermediate ones, instead of working through a queue of stale frames.
        self.timer = self.create_timer(self.min_period, self.process_latest)
        self.get_logger().info(
            f'gesture_command_node up (ONNX landmarks): {rgb_topic} -> {command_topic}')

    def on_rgb(self, msg: Image):
        self._latest = msg              # cheap: just stash the newest frame

    def process_latest(self):
        msg = self._latest
        if msg is None:
            return
        self._latest = None             # consume so each frame is handled once
        t = self.get_clock().now().nanoseconds * 1e-9

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'rgb convert failed: {exc}',
                                   throttle_duration_sec=5.0)
            return

        feat = self.tracker.process(bgr)

        if self.debug and feat.present:
            self.get_logger().info(
                f'hand: fingers={feat.fingers} index_only={feat.index_only} '
                f'thumb_down={feat.thumb_down} cx={feat.cx:+.2f} cy={feat.cy:+.2f}',
                throttle_duration_sec=1.0)

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
