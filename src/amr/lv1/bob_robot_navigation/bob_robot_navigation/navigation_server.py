import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time

from bob_robot_interfaces.action import NavigateRelative


class BugNavigationServer(Node):

    def __init__(self):
        super().__init__('bug_navigation_server')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.action_server = ActionServer(
            self,
            NavigateRelative,
            'navigate_relative',
            self.execute_callback)

        self.scan = None
        self.pose = None
        self.goal = None

        # Parameters
        self.safe_distance = 1.0
        self.linear_speed = 0.4
        self.angular_speed = 0.6

        self.get_logger().info("Bug Navigation Server started")

    # ----------------------------------------------------

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    # ----------------------------------------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info("New navigation goal received")

        # Wait for sensors
        while self.scan is None or self.pose is None:
            rclpy.spin_once(self)

        closest_point_dist = float('inf')
        closest_point_pose = None
        hit_point = None
        following_wall = False

        # Relativni cilj iz akcije (base_link)
        start_x = self.pose.position.x
        start_y = self.pose.position.y
        start_yaw = self.get_yaw()

        # Transformacija base_link → odom
        goal_x = start_x + goal_handle.request.x * math.cos(start_yaw) - goal_handle.request.y * math.sin(start_yaw)
        goal_y = start_y + goal_handle.request.x * math.sin(start_yaw) + goal_handle.request.y * math.cos(start_yaw)
        goal_yaw = self.normalize_angle(start_yaw + goal_handle.request.theta_deg * math.pi / 180.0)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            distance = math.sqrt((goal_x - self.pose.position.x) ** 2 +
                                 (goal_y - self.pose.position.y) ** 2)

            feedback = NavigateRelative.Feedback()
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.2:
                self.stop()
                self.rotate_to(goal_yaw)
                goal_handle.succeed()
                return NavigateRelative.Result(success=True)

            if self.is_obstacle_ahead():
                if not following_wall:
                    hit_point = (self.pose.position.x, self.pose.position.y)
                    closest_point_dist = distance
                    closest_point_pose = hit_point
                    following_wall = True

                self.follow_wall()

                d = self.distance_to_goal(goal_x, goal_y)
                if d < closest_point_dist:
                    closest_point_dist = d
                    closest_point_pose = (
                        self.pose.position.x,
                        self.pose.position.y
                    )

                if self.distance_between(
                        (self.pose.position.x, self.pose.position.y),
                        hit_point) < 0.3:
                    # Completed one loop
                    self.go_to_point(closest_point_pose)
                    following_wall = False

            else:
                self.go_straight_to(goal_x, goal_y)

            time.sleep(0.1)

        goal_handle.abort()
        return NavigateRelative.Result(success=False)

    # ----------------------------------------------------
    # Motion primitives
    # ----------------------------------------------------

    def go_straight_to(self, x, y):
        target_angle = math.atan2(
            y - self.pose.position.y,
            x - self.pose.position.x)

        angle_error = self.normalize_angle(target_angle - self.get_yaw())

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angle_error
        self.cmd_pub.publish(cmd)

    def follow_wall(self):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -self.angular_speed
        self.cmd_pub.publish(cmd)

    def rotate_to(self, target_yaw):
        while abs(self.normalize_angle(target_yaw - self.get_yaw())) > 0.05:
            cmd = Twist()
            cmd.angular.z = self.angular_speed * \
                math.copysign(1, self.normalize_angle(target_yaw - self.get_yaw()))
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self)

        self.stop()

    def go_to_point(self, point):
        while self.distance_between(
                (self.pose.position.x, self.pose.position.y), point) > 0.2:
            self.go_straight_to(point[0], point[1])
            rclpy.spin_once(self)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ----------------------------------------------------
    # Helpers
    # ----------------------------------------------------

    def is_obstacle_ahead(self):
        center = len(self.scan.ranges) // 2
        front = self.scan.ranges[center - 20:center + 20]
        return min(front) < self.safe_distance

    def distance_to_goal(self, x, y):
        return math.hypot(
            x - self.pose.position.x,
            y - self.pose.position.y)

    def distance_between(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_yaw(self):
        q = self.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = BugNavigationServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
