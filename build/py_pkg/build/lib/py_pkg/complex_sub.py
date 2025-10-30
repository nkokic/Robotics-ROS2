#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import Complex

class ComplexSubscriberNode(Node):
    def __init__(self):
        super().__init__("complex_sub")
        self.complex_subscriber = self.create_subscription(Complex, "my_friends_are_imaginary",
        self.recieved_callback, 10)
        self.get_logger().info("Complex number subscriber has been started.")
        
    def recieved_callback(self, msg):
        self.get_logger().info("Complex number received - real: " + str(msg.real) + " imaginary: " +
        str(msg.imaginary))

def main(args=None):
    rclpy.init(args=args)
    node = ComplexSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()