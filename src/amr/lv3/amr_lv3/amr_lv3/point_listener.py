import rclpy
from rclpy.node import Node
#!/usr/bin/env python
from geometry_msgs.msg import PointStamped
import numpy as np

global x,y,z
x = 0.0
y = 0.0
z = 0.0

class PointListener(Node):
    def __init__(self):
        super().__init__('point_listener')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg: PointStamped):
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "/map"
        point.point.x = msg.point.x      
        point.point.y = msg.point.y
        point.point.z = msg.point.z
        self.get_logger().info(f"coordinates:x={point.point.x} y={point.point.y}")


if __name__ == '__main__':
    rclpy.init()
    point_listener = PointListener()
    rclpy.spin(point_listener)
    point_listener.destroy_node()
    rclpy.shutdown()