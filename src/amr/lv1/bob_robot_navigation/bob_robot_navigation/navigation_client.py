#!/usr/bin/env python3

import rclpy
import asyncio
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from bob_robot_interfaces.action import NavigateRelative


class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self.client = ActionClient(self, NavigateRelative, 'navigate_relative')

    async def send_goal(self):

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return

        goal = NavigateRelative.Goal()
        goal.x = float(input("Unesi x [m]: "))
        goal.y = float(input("Unesi y [m]: "))
        goal.theta_deg = float(input("Unesi theta [deg]: "))

        self.get_logger().info("Sending goal")

        goal_handle = await self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        try:
            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=120.0
            )
            print("\n=== RESULT ===")
            print("Success:", result.result.success)

        except asyncio.TimeoutError:
            self.get_logger().error("Timeout – cancelling goal")
            await goal_handle.cancel_goal_async()

    def feedback_cb(self, msg):
        print(f"[FEEDBACK] Distance remaining: {msg.feedback.distance_remaining:.2f} m")


def main():
    rclpy.init()

    node = NavigationClient()

    # ROS executor in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # asyncio logic
    asyncio.run(node.send_goal())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
