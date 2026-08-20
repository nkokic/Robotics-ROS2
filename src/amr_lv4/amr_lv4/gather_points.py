#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os


class PatrollingPointGatherer(Node):
    def __init__(self):
        super().__init__('patrolling_point_gatherer')
        
        # Declare and get parameter for maximum number of points
        self.declare_parameter('max_points', 5)
        self.maxPoints = self.get_parameter('max_points').value
        
        # Declare parameter for output file path
        self.declare_parameter('output_file', 'patrol_points.yaml')
        self.outputFile = self.get_parameter('output_file').value
        
        # List to store points
        self.points = []
        
        # Subscribe to /clicked_point topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.ClickedPointCallback,
            10
        )
        
        self.get_logger().info(f'Patrolling Point Gatherer started. Max points: {self.maxPoints}')
        self.get_logger().info(f'Click points in RViz. Points will be saved to: {self.outputFile}')
    
    def ClickedPointCallback(self, msg):
        if len(self.points) < self.maxPoints:
            pointData = {
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            }
            self.points.append(pointData)
            self.get_logger().info(f'Point {len(self.points)}/{self.maxPoints} added: '
                                   f'x={pointData["x"]:.2f}, y={pointData["y"]:.2f}, z={pointData["z"]:.2f}')
            
            if len(self.points) >= self.maxPoints:
                self.SavePoints()
                self.get_logger().info(f'Reached maximum points ({self.maxPoints}). Points saved.')
        else:
            self.get_logger().warn(f'Maximum number of points ({self.maxPoints}) already reached.')
    
    def SavePoints(self):
        data = {'patrol_points': self.points}
        
        try:
            with open(self.outputFile, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)
            if rclpy.ok():
                self.get_logger().info(
                    f'Successfully saved {len(self.points)} points to {self.outputFile}'
                )
        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f'Failed to save points: {str(e)}')


def Main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointGatherer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save points on shutdown if any were collected
        if node.points and len(node.points) > 0:
            node.SavePoints()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    Main()
