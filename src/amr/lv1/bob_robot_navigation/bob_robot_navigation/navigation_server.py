#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, radians, sin, cos, pi
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

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Action server
        self.action_server = ActionServer(
            self,
            NavigateRelative,
            'navigate_relative',
            self.execute_callback
        )

        # Timer (10 Hz control loop)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan = None

        # Navigation state
        self.active_goal = None
        self.state = 'IDLE'

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        self.get_logger().info("Navigation server (timer-based) started")

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        self.scan = msg.ranges

    # --------------------------------------------------
    # Action callback (NON-BLOCKING)
    # --------------------------------------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info("New navigation goal accepted")

        dx = goal_handle.request.x
        dy = goal_handle.request.y
        dtheta = radians(goal_handle.request.theta_deg)

        start_yaw = self.yaw

        # Relative → absolute goal
        self.target_x = self.x + dx * cos(start_yaw) - dy * sin(start_yaw)
        self.target_y = self.y + dx * sin(start_yaw) + dy * cos(start_yaw)
        self.target_yaw = normalize_angle(start_yaw + dtheta)

        self.active_goal = goal_handle
        self.state = 'ROTATE_TO_TARGET'

        return NavigateRelative.Result()  # NE BLOKIRATI

    # --------------------------------------------------
    # Control loop (TIMER)
    # --------------------------------------------------

    def control_loop(self):

        if self.state == 'IDLE' or self.active_goal is None:
            return

        # Distance to target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = sqrt(dx*dx + dy*dy)

        # Feedback
        feedback = NavigateRelative.Feedback()
        feedback.distance_remaining = distance
        self.active_goal.publish_feedback(feedback)

        # ---------------- STATES ----------------
        
        self.get_logger().info(f"{self.state} \n x: {self.x},\n y:{self.y},\n angle:{self.yaw}")

        if self.state == 'ROTATE_TO_TARGET':
            desired_yaw = atan2(dy, dx)
            err = normalize_angle(desired_yaw - self.yaw)

            if abs(err) < 0.05:
                self.stop()
                self.state = 'FORWARD'
            else:
                self.rotate(err)

        elif self.state == 'FORWARD':
            if distance < 0.15:
                self.stop()
                self.state = 'ROTATE_FINAL'
            elif self.obstacle_ahead():
                self.stop()
                self.state = 'AVOID_OBSTACLE'
            else:
                self.forward()

        elif self.state == 'AVOID_OBSTACLE':
            free_yaw = self.find_free_direction()
            err = normalize_angle(free_yaw - self.yaw)

            if abs(err) < 0.05:
                self.state = 'FORWARD'
            else:
                self.rotate(err)

        elif self.state == 'ROTATE_FINAL':
            err = normalize_angle(self.target_yaw - self.yaw)

            if abs(err) < 0.05:
                self.stop()
                self.active_goal.succeed()
                result = NavigateRelative.Result()
                result.success = True
                self.active_goal.set_result(result)
                self.active_goal = None
                self.state = 'IDLE'
                self.get_logger().info("Goal reached")
            else:
                self.rotate(err)

    # --------------------------------------------------
    # Motion helpers
    # --------------------------------------------------

    def stop(self):
        self.cmd_pub.publish(Twist())

    def rotate(self, err):
        cmd = Twist()
        cmd.angular.z = 0.6 if err > 0 else -0.6
        self.cmd_pub.publish(cmd)
        

    def forward(self):
        cmd = Twist()
        cmd.linear.x = 0.25
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
        for i in range(20, center):
            if self.scan[center + i] > 1.0:
                return self.yaw - radians(i)
            if self.scan[center - i] > 1.0:
                return self.yaw + radians(i)
        return self.yaw


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    rclpy.init()
    node = NavigationServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
