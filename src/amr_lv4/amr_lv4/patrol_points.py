#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import yaml
import os


class PatrollingPointNavigator(Node):
    def __init__(self):
        super().__init__('patrolling_point_navigator')
        
        # Declare parameters
        self.declare_parameter('input_file', 'patrol_points.yaml')
        
        self.input_file = self.get_parameter('input_file').value
        
        # Initialize Basic Navigator
        self.navigator = BasicNavigator()
        
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')
        
        # Load patrol points
        self.patrol_points = self.load_points()
        
        if not self.patrol_points:
            self.get_logger().error('No patrol points loaded. Exiting.')
            return
        
        self.get_logger().info(f'Loaded {len(self.patrol_points)} patrol points')
        
        # Create timer for patrol loop
        self.timer = self.create_timer(1.0, self.patrol_callback)
        self.current_direction = 'forward'  # 'forward' or 'backward'
        self.is_navigating = False
        self.current_waypoint_index = 0  # Track which waypoint we're currently heading to
        
    def load_points(self):
        """Load patrol points from YAML file"""
        if not os.path.exists(self.input_file):
            self.get_logger().error(f'Input file not found: {self.input_file}')
            return []
        
        try:
            with open(self.input_file, 'r') as file:
                data = yaml.safe_load(file)
                points = data.get('patrol_points', [])
                self.get_logger().info(f'Successfully loaded {len(points)} points from {self.input_file}')
                return points
        except Exception as e:
            self.get_logger().error(f'Failed to load points: {str(e)}')
            return []
    
    def create_pose_stamped(self, x, y, z=0.0):
        """Create a PoseStamped message from x, y, z coordinates"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Neutral orientation
        return pose
    
    def patrol_callback(self):
        """Main patrol loop callback"""
        # Normal patrol logic
        if self.is_navigating:
            # Check if navigation is complete
            if not self.navigator.isTaskComplete():
                return
            
            # Get the result
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
                
                # Move to next waypoint
                if self.current_direction == 'forward':
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.patrol_points):
                        # Reached end, switch to backward
                        self.get_logger().info('Reached end of patrol, switching to backward direction')
                        self.current_direction = 'backward'
                        self.current_waypoint_index = len(self.patrol_points) - 1
                else:  # backward
                    self.current_waypoint_index -= 1
                    if self.current_waypoint_index < 0:
                        # Reached start, switch to forward
                        self.get_logger().info('Reached start of patrol, switching to forward direction')
                        self.current_direction = 'forward'
                        self.current_waypoint_index = 0
                
                self.is_navigating = False
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Patrol navigation was canceled!')
                self.is_navigating = False
            elif result == TaskResult.FAILED:
                self.get_logger().error('Patrol navigation failed!')
                self.is_navigating = False
        else:
            # Navigate to next waypoint
            if 0 <= self.current_waypoint_index < len(self.patrol_points):
                point = self.patrol_points[self.current_waypoint_index]
                goal_pose = self.create_pose_stamped(point['x'], point['y'], point.get('z', 0.0))
                
                self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index} '
                                      f'({self.current_direction}): '
                                      f'({point["x"]:.2f}, {point["y"]:.2f})')
                
                self.navigator.goToPose(goal_pose)
                self.is_navigating = True
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down patrolling navigator')
        if self.is_navigating:
            self.navigator.cancelTask()


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
