"""Closed-loop sim: integrate /commands/velocity -> /odom, drive the behavior
node through maneuvers, and report the outcome. No real robot involved."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Sim(Node):
    def __init__(self):
        super().__init__('sim')
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = 0.0
        self.create_subscription(Twist, '/commands/velocity', self._cmd, 10)
        self.odom = self.create_publisher(Odometry, '/odom', 20)
        self.gpub = self.create_publisher(String, '/gesture/tracking', 10)
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)

    def _cmd(self, m):
        self.vx, self.wz = m.linear.x, m.angular.z

    def _tick(self):
        self.yaw += self.wz * self.dt
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt
        o = Odometry()
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.odom.publish(o)

    def send(self, cmd):
        self.gpub.publish(String(data=cmd))

    def settle(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.01)


def run(label, cmd, wait, sim):
    sim.x = sim.y = sim.yaw = 0.0
    sim.settle(0.5)
    y0, x0 = sim.yaw, sim.x
    sim.send(cmd)
    sim.settle(wait)
    total_yaw = math.degrees(sim.yaw - y0)
    print(f'{label:11s}: net_yaw={total_yaw:+7.1f} deg  net_x={sim.x-x0:+.2f}m  net_y={sim.y:+.2f}m')


def main():
    rclpy.init()
    sim = Sim()
    # let odom start flowing so behavior node has yaw
    sim.settle(1.0)
    run('rotate360', 'rotate360', 9.0, sim)
    run('turn_left', 'turn_left', 9.0, sim)
    run('turn_right', 'turn_right', 9.0, sim)
    run('tail_wag', 'tail_wag', 7.0, sim)
    # forward then stop
    sim.x = sim.y = sim.yaw = 0.0; sim.settle(0.5); x0 = sim.x
    sim.send('forward'); sim.settle(1.5); moved = sim.x - x0
    sim.send('stop'); sim.settle(1.0)
    print(f'forward/stop: drove {moved:+.2f}m then stopped (vx now {sim.vx:.2f})')
    sim.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
