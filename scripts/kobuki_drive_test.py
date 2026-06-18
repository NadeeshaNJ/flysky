import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        self.pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.x = None
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _odom(self, msg):
        self.x = msg.pose.pose.position.x

    def wait_odom(self, timeout=8.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.x

    def drive(self, vx, seconds, hz=20.0):
        period = 1.0 / hz
        n = int(seconds * hz)
        tw = Twist(); tw.linear.x = vx
        for _ in range(n):
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=period)
        # Explicit stop (several times), then let cmd_vel_timeout hold it.
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    n = DriveTest()
    x0 = n.wait_odom()
    if x0 is None:
        print('RESULT: FAIL (no /odom — driver not connected)')
    else:
        print(f'odom x before: {x0:+.4f} m')
        n.drive(vx=0.10, seconds=1.0)   # ~10 cm forward
        # settle and read final odom
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(n, timeout_sec=0.1)
        x1 = n.x
        print(f'odom x after:  {x1:+.4f} m')
        print(f'delta:         {x1 - x0:+.4f} m')
        print('RESULT:', 'OK (base moved)' if abs(x1 - x0) > 0.02 else 'NO MOTION DETECTED')
    # final safety stop
    for _ in range(5):
        n.pub.publish(Twist()); time.sleep(0.02)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
