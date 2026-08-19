#!/usr/bin/env python3
"""Publish TurtleBot4 velocity commands from an interactive WASD terminal."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


HELP = """
WASD TurtleBot control
----------------------
  W: forward      S: reverse
  A: turn left    D: turn right
  Space or X: stop
  Q: quit
"""


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.linear_speed = 0.25
        self.angular_speed = 0.8

    def publish(self, linear=0.0, angular=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = WasdTeleop()
    settings = termios.tcgetattr(sys.stdin)
    bindings = {
        'w': (node.linear_speed, 0.0),
        's': (-node.linear_speed, 0.0),
        'a': (0.0, node.angular_speed),
        'd': (0.0, -node.angular_speed),
        ' ': (0.0, 0.0),
        'x': (0.0, 0.0),
    }

    print(HELP)
    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            key = sys.stdin.read(1).lower()
            if key in ('q', '\x03'):
                break
            node.publish(*bindings.get(key, (0.0, 0.0)))
    finally:
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
