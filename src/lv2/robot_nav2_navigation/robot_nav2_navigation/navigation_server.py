#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time

waypoints = [
    (2.389, 6.087, -0.001),
    (11.332, 6.280, -0.001),
    (21.297, 6.117, -0.001),
    (21.324, -0.658, -0.001),
    (17.281, -6.546, -0.001),
    (8.809, -6.618, -0.001),
    (0.438, -6.158, -0.001),
    (-5.818, -3.079, -0.001),
    (-5.804, 5.440, -0.001),
    (1.082, 6.012, 0.002),
    (2.389, 6.087, -0.001),
]



def main(args=None):
    rclpy.init(args=args)
    # Wait for navigation to fully activate
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # Create a separate node for publishing goal position
    goal_publisher_node = Node('goal_position_publisher')
    goal_publisher = goal_publisher_node.create_publisher(Pose, "pozicija_cilja", 10)

    for idx, point in enumerate(waypoints):
        repeat_count = 0
        
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]

        goal_publisher.publish(pose)
        
        while repeat_count < 6:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = point[0]
            goal_pose.pose.position.y = point[1]
            goal_pose.pose.orientation.w = 1.0
            goal_pose.pose.orientation.z = 0.0
            #Activate action
            navigator.goToPose(goal_pose)
            while not navigator.isTaskComplete():
                time.sleep(1)
                # Feedback
                feedback = navigator.getFeedback()
                if feedback:
                    print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    ) + ' seconds.'
                )
            # Fetch result
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Goal {idx} succeeded!')
                break
            elif result == TaskResult.CANCELED:
                print(f'Goal {idx} was canceled!')
                break
            elif result == TaskResult.FAILED:
                print(f'Goal {idx} failed!')
                if repeat_count < 3:
                    print(f'Retrying goal {idx}...')
                    repeat_count += 1
            else:
                print(f'Goal {idx} has an invalid return status!')
                break
    
    # Cleanup
    goal_publisher_node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
