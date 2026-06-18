#!/usr/bin/env python3
"""pet_behavior_node — executes pet-like maneuvers from gesture commands.

Subscribes:
    /gesture/tracking  (std_msgs/String)            gesture command events
    /vision/target     (geometry_msgs/PointStamped) face presence
    /odom              (nav_msgs/Odometry)          closed-loop yaw/position
Publishes:
    /commands/velocity (geometry_msgs/Twist)        to the Kobuki base
    /qbot/sound        (std_msgs/String)            abstract audio cues

Command -> behavior (see gesture_classifier.py for how each gesture is detected):
    stop        immediate halt
    forward     drive forward until stopped ("come closer")
    backward    drive backward until stopped
    rotate360   spin a full 360 in place (odom-closed-loop)
    turn_left   sidestep left:  turn +90, drive forward, turn -90 (face signaler)
    turn_right  sidestep right: turn -90, drive forward, turn +90
    tail_wag    oscillate left-right 3 times rapidly

Idle behavior: once a face has been seen, the robot wiggles every ~10 s while idle.

Continuous drive (forward/backward) has a safety timeout so a missed "stop" can't
let it run away; "stop" always interrupts whatever is running. One-shot maneuvers
ignore new commands until they finish (except "stop").
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HALF_PI = math.pi / 2.0
TWO_PI = 2.0 * math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


class PetBehaviorNode(Node):
    def __init__(self):
        super().__init__('pet_behavior_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sound_topic', '/qbot/sound')
        self.declare_parameter('linear_speed', 0.12)       # m/s drive
        self.declare_parameter('turn_speed', 1.0)          # rad/s maneuver turns
        self.declare_parameter('wag_speed', 2.2)           # rad/s fast wag/wiggle
        self.declare_parameter('sidestep_distance', 0.30)  # m forward in a sidestep
        self.declare_parameter('drive_timeout', 4.0)       # s safety cap on continuous drive
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('face_memory', 5.0)         # s a face is "still around"
        self.declare_parameter('idle_wiggle_period', 10.0) # s between idle wiggles
        self.declare_parameter('yaw_tol', 0.05)            # rad completion tolerance

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.wag_speed = float(self.get_parameter('wag_speed').value)
        self.sidestep = float(self.get_parameter('sidestep_distance').value)
        self.drive_timeout = float(self.get_parameter('drive_timeout').value)
        self.face_memory = float(self.get_parameter('face_memory').value)
        self.idle_wiggle_period = float(self.get_parameter('idle_wiggle_period').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        hz = float(self.get_parameter('control_hz').value)

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.sound_pub = self.create_publisher(
            String, self.get_parameter('sound_topic').value, 10)
        self.create_subscription(String, '/gesture/tracking', self.on_gesture, 10)
        self.create_subscription(PointStamped, '/vision/target', self.on_face, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

        # odom state
        self.yaw = None
        self.px = self.py = 0.0

        # control state
        self.mode = 'idle'            # 'idle' | 'continuous' | 'maneuver'
        self.cont_vx = 0.0
        self.cont_deadline = 0.0
        self.steps = deque()          # queue of step dicts
        self.step = None              # active step (with live progress fields)

        self.last_face = -1e9
        self.last_wiggle = 0.0

        self.timer = self.create_timer(1.0 / hz, self.control_step)
        self.get_logger().info('pet_behavior_node up (maneuver executor)')

    # ---- inputs ---------------------------------------------------------
    def on_odom(self, msg: Odometry):
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def on_face(self, _msg):
        self.last_face = self._now()

    def on_gesture(self, msg: String):
        cmd = (msg.data or '').strip()
        if cmd in ('', 'none'):
            return

        if cmd == 'stop':
            self._go_idle('stop')
            return
        # One-shot maneuvers lock out new commands until done (stop already handled).
        if self.mode == 'maneuver':
            return

        if cmd == 'forward':
            self._start_continuous(+self.linear_speed, 'approach')
        elif cmd == 'backward':
            self._start_continuous(-self.linear_speed, 'backup')
        elif cmd == 'rotate360':
            self._start_maneuver([self._rot(TWO_PI, self.turn_speed)], 'spin')
        elif cmd == 'turn_left':
            self._sidestep(+1)
        elif cmd == 'turn_right':
            self._sidestep(-1)
        elif cmd == 'tail_wag':
            self._start_maneuver(self._wag_steps(0.35, cycles=3), 'wag')

    # ---- maneuver construction -----------------------------------------
    def _rot(self, delta, w):
        return {'type': 'rotate', 'delta': delta, 'w': w}

    def _drive(self, dist, vx):
        return {'type': 'drive', 'dist': dist, 'vx': vx}

    def _wag_steps(self, amp, cycles):
        steps = [self._rot(+amp, self.wag_speed)]
        for _ in range(cycles):
            steps.append(self._rot(-2 * amp, self.wag_speed))
            steps.append(self._rot(+2 * amp, self.wag_speed))
        steps.append(self._rot(-amp, self.wag_speed))
        return steps

    def _sidestep(self, sign):
        # sign +1 = left, -1 = right. Turn to the side, advance, turn back to
        # the original heading (still facing the signaler).
        self._start_maneuver([
            self._rot(sign * HALF_PI, self.turn_speed),
            self._drive(self.sidestep, +self.linear_speed),
            self._rot(-sign * HALF_PI, self.turn_speed),
        ], 'left' if sign > 0 else 'right')

    # ---- mode transitions ----------------------------------------------
    def _start_continuous(self, vx, cue):
        self.mode = 'continuous'
        self.cont_vx = vx
        self.cont_deadline = self._now() + self.drive_timeout
        self.cue(cue)

    def _start_maneuver(self, steps, cue):
        if self.yaw is None:
            self.get_logger().warn('no /odom yet — cannot run maneuver', throttle_duration_sec=5.0)
            return
        self.mode = 'maneuver'
        self.steps = deque(steps)
        self.step = None
        self.cue(cue)

    def _go_idle(self, cue=None):
        self.mode = 'idle'
        self.steps.clear()
        self.step = None
        self.cont_vx = 0.0
        self.publish_stop()
        if cue:
            self.cue(cue)

    # ---- control loop ---------------------------------------------------
    def control_step(self):
        if self.mode == 'continuous':
            if self._now() >= self.cont_deadline:
                self._go_idle('stop')
            else:
                self._pub(self.cont_vx, 0.0)
            return

        if self.mode == 'maneuver':
            self._run_maneuver()
            return

        # idle: hold still, and wiggle every period while a face is around.
        self.publish_stop()
        now = self._now()
        if (now - self.last_face) <= self.face_memory and \
                (now - self.last_wiggle) >= self.idle_wiggle_period:
            self.last_wiggle = now
            self._start_maneuver(self._wag_steps(0.25, cycles=2), 'wiggle')

    def _run_maneuver(self):
        if self.step is None:
            if not self.steps:
                self._go_idle()
                return
            self.step = self.steps.popleft()
            self._init_step(self.step)

        s = self.step
        if s['type'] == 'rotate':
            dyaw = abs(wrap(self.yaw - s['_last']))
            s['_remaining'] -= dyaw
            s['_last'] = self.yaw
            if s['_remaining'] <= self.yaw_tol:
                self.step = None
                self._pub(0.0, 0.0)
            else:
                self._pub(0.0, s['_dir'] * s['w'])
        elif s['type'] == 'drive':
            traveled = math.hypot(self.px - s['_sx'], self.py - s['_sy'])
            if traveled >= s['dist']:
                self.step = None
                self._pub(0.0, 0.0)
            else:
                self._pub(s['vx'], 0.0)

    def _init_step(self, s):
        if s['type'] == 'rotate':
            s['_remaining'] = abs(s['delta'])
            s['_dir'] = 1.0 if s['delta'] >= 0 else -1.0
            s['_last'] = self.yaw
        elif s['type'] == 'drive':
            s['_sx'], s['_sy'] = self.px, self.py

    # ---- low-level ------------------------------------------------------
    def _pub(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def publish_stop(self):
        if rclpy.ok():
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass

    def cue(self, name):
        self.sound_pub.publish(String(data=name))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PetBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
