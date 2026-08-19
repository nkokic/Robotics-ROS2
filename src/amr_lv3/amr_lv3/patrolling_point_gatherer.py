import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os

#!/usr/bin/env python3



class PatrollingPointGatherer(Node):
    def __init__(self):
        super().__init__('patrolling_point_gatherer')
        
        # Declare and get parameter for maximum number of points
        self.declare_parameter('max_points', 5)
        self.max_points = self.get_parameter('max_points').value
        
        # Declare parameter for output file path
        self.declare_parameter('output_file', 'patrol_points.yaml')
        self.output_file = self.get_parameter('output_file').value
        
        # List to store points
        self.points = []
        
        # Subscribe to /clicked_point topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        self.get_logger().info(f'Patrolling Point Gatherer started. Max points: {self.max_points}')
        self.get_logger().info(f'Click points in RViz. Points will be saved to: {self.output_file}')
    
    def clicked_point_callback(self, msg):
        if len(self.points) < self.max_points:
            point_data = {
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            }
            self.points.append(point_data)
            self.get_logger().info(f'Point {len(self.points)}/{self.max_points} added: '
                                   f'x={point_data["x"]:.2f}, y={point_data["y"]:.2f}, z={point_data["z"]:.2f}')
            
            if len(self.points) >= self.max_points:
                self.save_points()
                self.get_logger().info(f'Reached maximum points ({self.max_points}). Points saved.')
        else:
            self.get_logger().warn(f'Maximum number of points ({self.max_points}) already reached.')
    
    def save_points(self):
        data = {'patrol_points': self.points}
        
        try:
            with open(self.output_file, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)
            self.get_logger().info(f'Successfully saved {len(self.points)} points to {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save points: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointGatherer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save points on shutdown if any were collected
        if node.points and len(node.points) > 0:
            node.save_points()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()