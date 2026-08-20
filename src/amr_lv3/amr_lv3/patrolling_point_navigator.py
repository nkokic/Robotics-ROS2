#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
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
        
        self.inputFile = self.get_parameter('input_file').value
        self.safeDistance = self.get_parameter('safe_distance').value
        self.waitTime = self.get_parameter('wait_time').value
        
        # Initialize Basic Navigator
        self.navigator = BasicNavigator()

        self.currentRobotPose = None
        self.poseSubscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.CurrentPoseCallback,
            10
        )
        
        # AMCL is initialized by robot_tb4_bringup. Passing a non-AMCL
        # localizer prevents BasicNavigator from publishing its own default
        # (0, 0, 0) pose while it waits for the navigation stack.
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.get_logger().info('Nav2 is ready!')
        
        # Load patrol points
        self.patrolPoints = self.LoadPoints()
        
        if not self.patrolPoints:
            self.get_logger().error('No patrol points loaded. Exiting.')
            return
        
        self.get_logger().info(f'Loaded {len(self.patrolPoints)} patrol points')
        self.get_logger().info(f'Safe distance from object: {self.safeDistance}m')
        self.get_logger().info(f'Wait time at object: {self.waitTime}s')
        
        # Subscribe to detected object topic
        self.objectSubscription = self.create_subscription(
            Point,
            '/detected_object_point',
            self.ObjectDetectedCallback,
            10
        )
        
        # Create timer for patrol loop
        self.patrolTimer = self.create_timer(1.0, self.PatrolCallback)
        self.currentDirection = 'forward'
        self.isNavigating = False
        self.isInvestigatingObject = False
        self.objectInvestigated = False
        self.waitEndTime = None
        self.investigationPhase = None
        self.currentWaypointIndex = 0

    def LoadPoints(self):
        """Load patrol points from YAML file"""
        if not os.path.exists(self.inputFile):
            self.get_logger().error(f'Input file not found: {self.inputFile}')
            return []
        
        try:
            with open(self.inputFile, 'r', encoding='utf-8') as inputStream:
                data = yaml.safe_load(inputStream) or {}
                points = data.get('patrol_points', [])
                self.get_logger().info(f'Successfully loaded {len(points)} points from {self.inputFile}')
                return points
        except (OSError, yaml.YAMLError, AttributeError) as exception:
            self.get_logger().error(f'Failed to load points: {exception}')
            return []

    def CurrentPoseCallback(self, message):
        """Store the latest localized robot pose for object approach planning."""
        self.currentRobotPose = message.pose.pose
    
    def ObjectDetectedCallback(self, message):
        """Handle detected object - interrupt patrol and go to object"""
        if self.isInvestigatingObject:
            # Already investigating an object, ignore new detections
            return
        
        objectX = message.x
        objectY = message.y

        if self.currentRobotPose is None:
            self.get_logger().warn('Cannot approach object before an AMCL pose is available')
            return
        
        self.get_logger().info(f'Object detected at ({objectX:.2f}, {objectY:.2f})')
        self.get_logger().info('Interrupting patrol to investigate object...')
        
        # Cancel current navigation if active
        if self.isNavigating:
            self.navigator.cancelTask()
            self.isNavigating = False
            self.get_logger().info('Current patrol navigation canceled')
        
        robotX = self.currentRobotPose.position.x
        robotY = self.currentRobotPose.position.y
        deltaX = objectX - robotX
        deltaY = objectY - robotY
        objectDistance = math.hypot(deltaX, deltaY)

        if objectDistance <= self.safeDistance:
            self.get_logger().info('Robot is already within the configured safe distance')
            return

        # Stop on the line from the robot to the object, one safeDistance away
        # from the object. This avoids assuming that negative X is reachable.
        approachX = objectX - (deltaX / objectDistance) * self.safeDistance
        approachY = objectY - (deltaY / objectDistance) * self.safeDistance
        approachYaw = math.atan2(deltaY, deltaX)
        
        self.get_logger().info(f'Approaching object at safe position: ({approachX:.2f}, {approachY:.2f})')
        
        # Create goal pose at safe distance from object
        goalPose = self.CreatePoseStamped(approachX, approachY, 0.0, approachYaw)
        
        # Navigate to object
        self.navigator.goToPose(goalPose)
        self.isInvestigatingObject = True
        self.investigationPhase = 'going_to_object'
    
    def CreatePoseStamped(self, x, y, z=0.0, yaw=0.0):
        """Create a PoseStamped message from x, y, z coordinates"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose
    
    def GetNextGoals(self):
        """Get the next goal in the patrol sequence"""
        if self.currentDirection == 'forward':
            # Go from first to last
            goals = [self.CreatePoseStamped(point['x'], point['y'], point.get('z', 0.0))
                     for point in self.patrolPoints]
        else:  # backward
            # Go from last to first (reversed)
            goals = [self.CreatePoseStamped(point['x'], point['y'], point.get('z', 0.0))
                     for point in reversed(self.patrolPoints)]
        
        return goals
    
    def PatrolCallback(self):
        """Main patrol loop callback"""
        # Handle object investigation states
        if self.isInvestigatingObject and not self.objectInvestigated:
            
            # Phase 1: Going to object
            if self.investigationPhase == 'going_to_object':
                if not self.navigator.isTaskComplete():
                    return
                
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Reached object location!')
                    self.get_logger().info(f'Waiting for {self.waitTime} seconds...')
                    self.waitEndTime = time.time() + self.waitTime
                    self.investigationPhase = 'waiting'
                elif result in (TaskResult.CANCELED, TaskResult.FAILED):
                    self.get_logger().warn('Failed to reach object. Returning to patrol.')
                    self.isInvestigatingObject = False
                    self.investigationPhase = None
                return
            
            # Phase 2: Waiting at object
            elif self.investigationPhase == 'waiting':
                if self.waitEndTime is not None:
                    currentTime = time.time()
                    if currentTime >= self.waitEndTime:
                        self.get_logger().info('Wait time completed.')
                        self.get_logger().info('Resuming patrol - moving to next waypoint.')
                        self.waitEndTime = None
                        
                        # Clear investigation state - patrol will continue to next waypoint
                        self.isInvestigatingObject = False
                        self.objectInvestigated = True
                        self.investigationPhase = None
                return
        
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
                        self.objectInvestigated = False
                        self.currentWaypointIndex = len(self.patrolPoints) - 1
                else:  # backward
                    self.currentWaypointIndex -= 1
                    if self.currentWaypointIndex < 0:
                        # Reached start, switch to forward
                        self.get_logger().info('Reached start of patrol, switching to forward direction')
                        self.currentDirection = 'forward'
                        self.objectInvestigated = False
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


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingPointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.Shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
