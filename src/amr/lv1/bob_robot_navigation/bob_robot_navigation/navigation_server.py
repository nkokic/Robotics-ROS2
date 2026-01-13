#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from math import atan2, sqrt, radians, cos, sin, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

from bob_robot_interfaces.action import NavigateRelative


def normalize_angle(a):
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a


class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_fake', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.action_server = ActionServer(
            self,
            NavigateRelative,
            'navigate_relative',
            self.execute_callback
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan = None

        self.get_logger().info("Navigation action server started")

    # ---------------- Callbacks ----------------

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        self.scan = msg.ranges

    # ---------------- Motion helpers ----------------

    def stop(self):
        self.cmd_pub.publish(Twist())

    def rotate_to(self, target_yaw, speed=0.6, tol=0.03):
        r = self.create_rate(30)
        while rclpy.ok():
            err = normalize_angle(target_yaw - self.yaw)
            if abs(err) < tol:
                break
            cmd = Twist()
            cmd.angular.z = speed if err > 0 else -speed
            self.cmd_pub.publish(cmd)
            r.sleep()
        self.stop()

    def forward(self, speed=0.25):
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

    def obstacle_ahead(self, limit=0.6):
        if self.scan is None:
            return False
        center = len(self.scan) // 2
        for i in range(center - 10, center + 10):
            if self.scan[i] < limit:
                return True
        return False

    def find_free_direction(self):
        center = len(self.scan) // 2
        for i in range(30, center):
            if self.scan[center + i] > 1.0:
                return self.yaw - radians(i)
            if self.scan[center - i] > 1.0:
                return self.yaw + radians(i)
        return self.yaw

    # ---------------- Action execution ----------------

    async def execute_callback(self, goal_handle):
        self.get_logger().info("New navigation goal received")

        dx = goal_handle.request.x
        dy = goal_handle.request.y
        dtheta = radians(goal_handle.request.theta_deg)

        start_x, start_y, start_yaw = self.x, self.y, self.yaw

        target_x = start_x + dx * cos(start_yaw) - dy * sin(start_yaw)
        target_y = start_y + dx * sin(start_yaw) + dy * cos(start_yaw)
        target_yaw = normalize_angle(start_yaw + dtheta)

        # 1️⃣ Rotate towards target
        heading = atan2(target_y - self.y, target_x - self.x)
        self.rotate_to(heading)

        rate = self.create_rate(10)

        # 2️⃣ Move forward (Bug algorithm)
        while rclpy.ok():
            dist = sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

            feedback = NavigateRelative.Feedback()
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            if dist < 0.15:
                break

            if self.obstacle_ahead():
                self.stop()
                free_yaw = self.find_free_direction()
                self.rotate_to(free_yaw)
            else:
                self.forward()

            rate.sleep()

        self.stop()

        # 3️⃣ Final orientation
        self.rotate_to(target_yaw)

        goal_handle.succeed()
        result = NavigateRelative.Result()
        result.success = True
        self.get_logger().info("Goal reached")
        return result


def main():
    rclpy.init()
    node = NavigationServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
