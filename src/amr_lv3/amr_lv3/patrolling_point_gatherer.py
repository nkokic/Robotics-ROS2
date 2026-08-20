#!/usr/bin/env python3

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


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
        self.clickedPointSubscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.ClickedPointCallback,
            10
        )
        
        self.get_logger().info(f'Patrolling Point Gatherer started. Max points: {self.maxPoints}')
        self.get_logger().info(f'Click points in RViz. Points will be saved to: {self.outputFile}')
    
    def ClickedPointCallback(self, message):
        if len(self.points) < self.maxPoints:
            pointData = {
                'x': message.point.x,
                'y': message.point.y,
                'z': message.point.z
            }
            self.points.append(pointData)
            self.get_logger().info(
                f'Point {len(self.points)}/{self.maxPoints} added: '
                f'x={pointData["x"]:.2f}, y={pointData["y"]:.2f}, '
                f'z={pointData["z"]:.2f}'
            )
            
            if len(self.points) >= self.maxPoints:
                self.SavePoints()
                self.get_logger().info(f'Reached maximum points ({self.maxPoints}). Points saved.')
        else:
            self.get_logger().warn(f'Maximum number of points ({self.maxPoints}) already reached.')
    
    def SavePoints(self):
        data = {'patrol_points': self.points}
        
        try:
            with open(self.outputFile, 'w', encoding='utf-8') as outputStream:
                yaml.dump(data, outputStream, default_flow_style=False)
            self.get_logger().info(f'Successfully saved {len(self.points)} points to {self.outputFile}')
        except (OSError, yaml.YAMLError) as exception:
            self.get_logger().error(f'Failed to save points: {exception}')


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointGatherer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save points on shutdown if any were collected
        if node.points:
            node.SavePoints()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
