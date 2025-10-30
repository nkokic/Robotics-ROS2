#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyROS2Node(Node):
    def __init__(self):
        super().__init__('my_node')
        self.m_counter = 0
        self.m_timer = self.create_timer(1.0, self.print_hello)

    def print_hello(self):
        self.get_logger().info("Helloooo! Count: " + str(self.m_counter))
        self.m_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyROS2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()