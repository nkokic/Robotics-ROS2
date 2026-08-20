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
        
        self.inputFile = self.get_parameter('input_file').value
        
        # Initialize Basic Navigator
        self.navigator = BasicNavigator()
        
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')
        
        # Load patrol points
        self.patrolPoints = self.LoadPoints()
        
        if not self.patrolPoints:
            self.get_logger().error('No patrol points loaded. Exiting.')
            return
        
        self.get_logger().info(f'Loaded {len(self.patrolPoints)} patrol points')
        
        # Create timer for patrol loop
        self.timer = self.create_timer(1.0, self.PatrolCallback)
        self.currentDirection = 'forward'  # 'forward' or 'backward'
        self.isNavigating = False
        self.currentWaypointIndex = 0  # Track which waypoint we're currently heading to
        
    def LoadPoints(self):
        """Load patrol points from YAML file"""
        if not os.path.exists(self.inputFile):
            self.get_logger().error(f'Input file not found: {self.inputFile}')
            return []
        
        try:
            with open(self.inputFile, 'r') as file:
                data = yaml.safe_load(file)
                points = data.get('patrol_points', [])
                self.get_logger().info(f'Successfully loaded {len(points)} points from {self.inputFile}')
                return points
        except Exception as e:
            self.get_logger().error(f'Failed to load points: {str(e)}')
            return []
    
    def CreatePoseStamped(self, x, y, z=0.0):
        """Create a PoseStamped message from x, y, z coordinates"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Neutral orientation
        return pose
    
    def PatrolCallback(self):
        """Main patrol loop callback"""
        # Normal patrol logic
        if self.isNavigating:
            # Check if navigation is complete
            if not self.navigator.isTaskComplete():
                return
            
            # Get the result
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached waypoint {self.currentWaypointIndex}')
                
                # Move to next waypoint
                if self.currentDirection == 'forward':
                    self.currentWaypointIndex += 1
                    if self.currentWaypointIndex >= len(self.patrolPoints):
                        # Reached end, switch to backward
                        self.get_logger().info('Reached end of patrol, switching to backward direction')
                        self.currentDirection = 'backward'
                        self.currentWaypointIndex = len(self.patrolPoints) - 1
                else:  # backward
                    self.currentWaypointIndex -= 1
                    if self.currentWaypointIndex < 0:
                        # Reached start, switch to forward
                        self.get_logger().info('Reached start of patrol, switching to forward direction')
                        self.currentDirection = 'forward'
                        self.currentWaypointIndex = 0
                
                self.isNavigating = False
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Patrol navigation was canceled!')
                self.isNavigating = False
            elif result == TaskResult.FAILED:
                self.get_logger().error('Patrol navigation failed!')
                self.isNavigating = False
        else:
            # Navigate to next waypoint
            if 0 <= self.currentWaypointIndex < len(self.patrolPoints):
                point = self.patrolPoints[self.currentWaypointIndex]
                goalPose = self.CreatePoseStamped(point['x'], point['y'], point.get('z', 0.0))
                
                self.get_logger().info(f'Navigating to waypoint {self.currentWaypointIndex} '
                                      f'({self.currentDirection}): '
                                      f'({point["x"]:.2f}, {point["y"]:.2f})')
                
                self.navigator.goToPose(goalPose)
                self.isNavigating = True
    
    def Shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down patrolling navigator')
        if self.isNavigating:
            self.navigator.cancelTask()


def Main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.Shutdown()
        node.destroy_node()
        rclpy.Shutdown()


if __name__ == '__main__':
    Main()
