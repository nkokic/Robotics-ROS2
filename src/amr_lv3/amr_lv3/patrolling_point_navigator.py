#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Point
import yaml
import os
import math
import time


class PatrollingPointNavigator(Node):
    def __init__(self):
        super().__init__('patrolling_point_navigator')
        
        # Declare parameters
        self.declare_parameter('input_file', 'patrol_points.yaml')
        self.declare_parameter('safe_distance', 1.0)  # Safe distance from object in meters
        self.declare_parameter('wait_time', 3.0)  # Time to wait at object location in seconds
        
        self.input_file = self.get_parameter('input_file').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.wait_time = self.get_parameter('wait_time').value
        
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
        self.get_logger().info(f'Safe distance from object: {self.safe_distance}m')
        self.get_logger().info(f'Wait time at object: {self.wait_time}s')
        
        # Subscribe to detected object topic
        self.object_subscription = self.create_subscription(
            Point,
            '/detected_object_point',
            self.object_detected_callback,
            10
        )
        
        # Create timer for patrol loop
        self.timer = self.create_timer(1.0, self.patrol_callback)
        self.current_direction = 'forward'  # 'forward' or 'backward'
        self.is_navigating = False
        self.is_investigating_object = False
        self.wait_end_time = None
        self.investigation_phase = None  # 'going_to_object', 'waiting'
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
    
    def object_detected_callback(self, msg):
        """Handle detected object - interrupt patrol and go to object"""
        if self.is_investigating_object:
            # Already investigating an object, ignore new detections
            return
        
        object_x = msg.x
        object_y = msg.y
        
        self.get_logger().info(f'Object detected at ({object_x:.2f}, {object_y:.2f})')
        self.get_logger().info('Interrupting patrol to investigate object...')
        
        # Cancel current navigation if active
        if self.is_navigating:
            self.navigator.cancelTask()
            self.is_navigating = False
            self.get_logger().info('Current patrol navigation canceled')
        
        # Calculate approach position (safe distance from object)
        # For simplicity, approach from the negative x direction
        approach_x = object_x - self.safe_distance
        approach_y = object_y
        
        self.get_logger().info(f'Approaching object at safe position: ({approach_x:.2f}, {approach_y:.2f})')
        
        # Create goal pose at safe distance from object
        goal_pose = self.create_pose_stamped(approach_x, approach_y, 0.0)
        
        # Navigate to object
        self.navigator.goToPose(goal_pose)
        self.is_investigating_object = True
        self.investigation_phase = 'going_to_object'
    
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
    
    def get_next_goal(self):
        """Get the next goal in the patrol sequence"""
        if self.current_direction == 'forward':
            # Go from first to last
            goals = [self.create_pose_stamped(p['x'], p['y'], p.get('z', 0.0)) 
                     for p in self.patrol_points]
        else:  # backward
            # Go from last to first (reversed)
            goals = [self.create_pose_stamped(p['x'], p['y'], p.get('z', 0.0)) 
                     for p in reversed(self.patrol_points)]
        
        return goals
    
    def patrol_callback(self):
        """Main patrol loop callback"""
        # Handle object investigation states
        if self.is_investigating_object:
            
            # Phase 1: Going to object
            if self.investigation_phase == 'going_to_object':
                if not self.navigator.isTaskComplete():
                    return
                
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Reached object location!')
                    self.get_logger().info(f'Waiting for {self.wait_time} seconds...')
                    self.wait_end_time = time.time() + self.wait_time
                    self.investigation_phase = 'waiting'
                elif result == TaskResult.CANCELED or result == TaskResult.FAILED:
                    self.get_logger().warn('Failed to reach object. Returning to patrol.')
                    self.is_investigating_object = False
                    self.investigation_phase = None
                    self.interrupted_waypoint = None
                return
            
            # Phase 2: Waiting at object
            elif self.investigation_phase == 'waiting':
                if self.wait_end_time is not None:
                    current_time = time.time()
                    if current_time >= self.wait_end_time:
                        self.get_logger().info('Wait time completed.')
                        self.get_logger().info('Resuming patrol - moving to next waypoint.')
                        self.wait_end_time = None
                        
                        # Clear investigation state - patrol will continue to next waypoint
                        self.is_investigating_object = False
                        self.investigation_phase = None
                return
        
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
