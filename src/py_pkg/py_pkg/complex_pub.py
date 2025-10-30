#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import Complex
import random

class ComplexPublisherNode(Node):
    def __init__(self):
        super().__init__("complex_pub")
        self.complex_publisher = self.create_publisher(Complex, "my_friends_are_imaginary", 10)
        self.status_timer = self.create_timer(1.0, self.publish_callback)
        self.get_logger().info("Complex number publisher has been started.")

    def publish_callback(self):
        msg = Complex()
        msg.real = random.random()
        msg.imaginary = random.random()
        self.complex_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComplexPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()