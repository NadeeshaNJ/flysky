#!/usr/bin/env python3
"""face_tracker_node — detects and tracks a human face in the Kinect RGB stream.

Subscribes:  /kinect/rgb/image_raw  (sensor_msgs/Image)
Publishes:   /vision/target          (geometry_msgs/PointStamped)

The published Point encodes the tracked face as normalised image coordinates so
downstream nodes are resolution-independent:
    x =  horizontal offset from centre, range [-1, 1] (left negative)
    y =  vertical offset from centre,   range [-1, 1] (up negative)
    z =  face size as a fraction of image width, range (0, 1]  (proxy for distance)

Detection uses the OpenCV Haar cascade bundled with opencv-python, so it runs
with no extra model downloads. Swap in a DNN/landmark detector later if needed.
"""

import os

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')

        self.declare_parameter('rgb_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('target_topic', '/vision/target')
        self.declare_parameter('min_neighbors', 6)
        self.declare_parameter('scale_factor', 1.1)

        rgb_topic = self.get_parameter('rgb_topic').value
        target_topic = self.get_parameter('target_topic').value
        self.min_neighbors = int(self.get_parameter('min_neighbors').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)

        self.bridge = CvBridge()
        cascade_path = self._find_cascade('haarcascade_frontalface_default.xml')
        self.detector = cv2.CascadeClassifier(cascade_path) if cascade_path else cv2.CascadeClassifier()
        if cascade_path is None or self.detector.empty():
            self.get_logger().error(
                'Failed to load the frontal-face Haar cascade; face detection disabled.')

        self.sub = self.create_subscription(Image, rgb_topic, self.on_image, 10)
        self.pub = self.create_publisher(PointStamped, target_topic, 10)
        self.get_logger().info(
            f'face_tracker_node up: {rgb_topic} -> {target_topic}')

    @staticmethod
    def _find_cascade(name: str):
        """Locate an OpenCV Haar cascade across packaging layouts.

        Debian's python3-opencv has no ``cv2.data``; its cascades live under
        /usr/share/opencv4/haarcascades/. pip wheels expose ``cv2.data`` instead.
        """
        candidates = []
        data = getattr(cv2, 'data', None)
        if data is not None:
            candidates.append(os.path.join(data.haarcascades, name))
        candidates.append(os.path.join('/usr/share/opencv4/haarcascades', name))
        for path in candidates:
            if os.path.isfile(path):
                return path
        return None

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge convert failed: {exc}',
                                   throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detectMultiScale(
            gray, scaleFactor=self.scale_factor, minNeighbors=self.min_neighbors,
            minSize=(40, 40))

        if len(faces) == 0:
            return

        h, w = gray.shape[:2]
        # Pick the largest face (closest / most prominent person).
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
        cx = x + fw / 2.0
        cy = y + fh / 2.0

        target = PointStamped()
        target.header = msg.header
        target.point.x = (cx - w / 2.0) / (w / 2.0)
        target.point.y = (cy - h / 2.0) / (h / 2.0)
        target.point.z = float(fw) / float(w)
        self.pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
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
