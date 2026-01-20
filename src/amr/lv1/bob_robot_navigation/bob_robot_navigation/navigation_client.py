import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bob_robot_interfaces.action import NavigateRelative


class BugNavigationClient(Node):

    def __init__(self):
        super().__init__('bug_navigation_client')
        self.client = ActionClient(self, NavigateRelative, 'navigate_relative')

    def send_goal(self, x, y, theta):
        goal = NavigateRelative.Goal()
        goal.x = x
        goal.y = y
        goal.theta_deg = theta

        self.client.wait_for_server()
        self.send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb)

        self.send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback):
        self.get_logger().info(
            f"Remaining distance: {feedback.feedback.distance_remaining:.2f} m")

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation success: {result.success}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = BugNavigationClient()

    x = float(input("Enter x [m]: "))
    y = float(input("Enter y [m]: "))
    theta = float(input("Enter theta [deg]: "))

    node.send_goal(x, y, theta)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
