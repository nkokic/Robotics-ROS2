#!/usr/bin/env python3

import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionClient
from bob_robot_interfaces.action import NavigateRelative


class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self.client = ActionClient(self, NavigateRelative, 'navigate_relative')

    async def send_goal(self):
        self.client.wait_for_server()

        goal = NavigateRelative.Goal()
        goal.x = float(input("Unesi x [m]: "))
        goal.y = float(input("Unesi y [m]: "))
        goal.theta_deg = float(input("Unesi theta [deg]: "))

        self.get_logger().info("Sending navigation goal...")
        send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )

        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        try:
            result_future = goal_handle.get_result_async()
            result = await asyncio.wait_for(result_future, timeout=120.0)
            print("\n=== RESULT ===")
            print("Success:", result.result.success)
        except asyncio.TimeoutError:
            self.get_logger().error("Timeout! Cancelling goal.")
            await goal_handle.cancel_goal_async()

    def feedback_cb(self, msg):
        print(f"Udaljenost do cilja: {msg.feedback.distance_remaining:.2f} m")


def main():
    rclpy.init()
    node = NavigationClient()
    asyncio.run(node.send_goal())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
