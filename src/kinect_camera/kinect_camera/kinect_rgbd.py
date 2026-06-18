#!/usr/bin/env python3
"""kinect_rgbd — publishes RGB and depth frames from an Xbox Kinect v1 / Kinect 360.

Uses the libfreenect Python bindings (``import freenect``). The node is written
so that it starts cleanly even when no Kinect is attached: if the device or the
``freenect`` module is unavailable it logs a throttled warning instead of
crashing, so the rest of the QBot graph can still be brought up for testing.

Hardware wiring (firmware, udev rules, libfreenect build) is handled by
``setup_qbot_env.sh`` — see QBOT_OVERVIEW.md.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import freenect  # libfreenect python binding
    _HAVE_FREENECT = True
except Exception:  # pragma: no cover - hardware/driver may be absent
    freenect = None
    _HAVE_FREENECT = False


class KinectRGBD(Node):
    def __init__(self):
        super().__init__('kinect_rgbd')

        self.declare_parameter('rgb_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('depth_topic', '/kinect/depth/image_raw')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_id', 'kinect_rgb_optical_frame')

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        rate = float(self.get_parameter('frame_rate').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)

        if not _HAVE_FREENECT:
            self.get_logger().warn(
                "libfreenect python binding not found — running in NO-DEVICE mode. "
                "Install it via setup_qbot_env.sh and attach the Kinect, then restart."
            )

        period = 1.0 / rate if rate > 0 else 1.0 / 30.0
        self.timer = self.create_timer(period, self.capture_and_publish)
        self.get_logger().info(
            f'kinect_rgbd up: rgb -> {rgb_topic}, depth -> {depth_topic} @ {rate:.0f} Hz'
        )

    def capture_and_publish(self):
        if not _HAVE_FREENECT:
            self.get_logger().warn('No Kinect device available.', throttle_duration_sec=10.0)
            return

        try:
            rgb, _ = freenect.sync_get_video()      # HxWx3 uint8 (RGB)
            depth, _ = freenect.sync_get_depth()    # HxW uint16 (mm-ish)
        except Exception as exc:  # pragma: no cover - runtime device errors
            self.get_logger().warn(
                f'Kinect read failed: {exc}', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.frame_id
        self.rgb_pub.publish(rgb_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinectRGBD()
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
