#!/usr/bin/env python3
"""pet_behavior_node — turns vision + gesture inputs into robot motion and sound.

Subscribes:
    /vision/target     (geometry_msgs/PointStamped)  normalised face target
    /gesture/tracking  (std_msgs/String)             gesture command
Publishes:
    /cmd_vel           (geometry_msgs/Twist)         velocity to the Kobuki base
    /commands/sound    (std_msgs/String)             audio-feedback cue

Behavior summary (see QBOT_OVERVIEW.md):
    beckon     -> move forward (approach)
    palm       -> stop / hold
    circle     -> rotate in place
    wave_left  -> step left,   wave_right -> step right
    wag        -> oscillate left-right (tail wag)
    (no gesture, face present) -> re-centre orientation toward the face

A watchdog stops the robot if no gesture/target has been seen recently, so it
never runs away on stale data. Velocities are intentionally gentle for indoor
use; tune in config/behavior.yaml.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String


class PetBehaviorNode(Node):
    def __init__(self):
        super().__init__('pet_behavior_node')

        # NB: the Kobuki base subscribes to /commands/velocity (Twist) and uses a
        # *typed* /commands/sound (kobuki Sound msg). We keep our sound cue on a
        # neutral topic to avoid a type clash; the bringup launch points
        # cmd_vel_topic at /commands/velocity.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sound_topic', '/qbot/sound')
        self.declare_parameter('target_topic', '/vision/target')
        self.declare_parameter('gesture_topic', '/gesture/tracking')
        self.declare_parameter('linear_speed', 0.12)      # m/s
        self.declare_parameter('angular_speed', 0.6)      # rad/s
        self.declare_parameter('center_gain', 0.8)        # P-gain for face centring
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('input_timeout', 1.0)      # s before watchdog stop

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.center_gain = float(self.get_parameter('center_gain').value)
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        control_hz = float(self.get_parameter('control_hz').value)

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.sound_pub = self.create_publisher(
            String, self.get_parameter('sound_topic').value, 10)
        self.create_subscription(
            PointStamped, self.get_parameter('target_topic').value,
            self.on_target, 10)
        self.create_subscription(
            String, self.get_parameter('gesture_topic').value,
            self.on_gesture, 10)

        self.gesture = 'none'
        self.target = None
        self._last_input = self.get_clock().now()
        self._last_sound = None
        self._wag_phase = 0.0

        self.timer = self.create_timer(1.0 / control_hz, self.control_step)
        self.get_logger().info('pet_behavior_node up')

    # ----- input callbacks -------------------------------------------------
    def on_target(self, msg: PointStamped):
        self.target = msg.point
        self._last_input = self.get_clock().now()

    def on_gesture(self, msg: String):
        self.gesture = msg.data or 'none'
        self._last_input = self.get_clock().now()

    # ----- control loop ----------------------------------------------------
    def control_step(self):
        age = (self.get_clock().now() - self._last_input).nanoseconds * 1e-9
        if age > self.input_timeout:
            self.publish_stop()
            return

        twist = Twist()
        g = self.gesture

        if g == 'beckon':
            twist.linear.x = self.linear_speed
            self.cue('approach')
        elif g == 'palm':
            self.cue('stop')
        elif g == 'circle':
            twist.angular.z = self.angular_speed
            self.cue('spin')
        elif g == 'wave_left':
            twist.angular.z = self.angular_speed
            self.cue('left')
        elif g == 'wave_right':
            twist.angular.z = -self.angular_speed
            self.cue('right')
        elif g == 'wag':
            self._wag_phase += 0.6
            twist.angular.z = self.angular_speed * math.sin(self._wag_phase)
            self.cue('wag')
        else:
            # No active gesture: gently re-centre toward the detected face.
            if self.target is not None:
                twist.angular.z = -self.center_gain * self.angular_speed * self.target.x
                self.cue('track')

        self.cmd_pub.publish(twist)

    def publish_stop(self):
        # Guard against publishing during/after shutdown (SIGINT teardown race).
        if rclpy.ok():
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass

    def cue(self, name: str):
        if name != self._last_sound:
            self.sound_pub.publish(String(data=name))
            self._last_sound = name


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
