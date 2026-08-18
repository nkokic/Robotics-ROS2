#!/usr/bin/env python3

"""
ROS 2 Nav2 waypoint navigator.

Navigates the robot through a predefined sequence of waypoints.

For every waypoint:
    1. Publish the goal position on /goal_position.
    2. Send the waypoint to Nav2.
    3. Wait until navigation completes.
    4. Report the navigation result.
    5. Retry failed goals up to the configured retry limit.

The code intentionally uses C#-style naming conventions:
    - PascalCase for classes.
    - camelCase for methods, fields, and local variables.
"""

import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)
from rclpy.duration import Duration
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

WAYPOINTS = [
    (0.0, 6.0, 0.0),    # 1
    (12.0, 6.0, 0.0),   # 2
    (12.0, 1.0, 0.0),   # 3
    (20.0, 1.0, 0.0),   # 4
    (20.0, 6.0, 0.0),   # 5
    (22.0, 6.0, 0.0),   # 6
    (22.0, -6.0, 0.0),   # 7
    (9.5, -3.0, 0.0),   # 8
    (-5.0, -4.0, 0.0),   # 9
    (-5.0, 6.0, 0.0),   # 10
]

MAP_FRAME = 'map'
GOAL_TOPIC = 'goal_position'

MAX_RETRIES = 3
FEEDBACK_INTERVAL_SECONDS = 1.0


class WaypointNavigator:
    """
    Controls navigation through a predefined sequence of waypoints.

    The class wraps Nav2's BasicNavigator and provides:
        - Goal publishing.
        - Pose creation.
        - Navigation execution.
        - Feedback reporting.
        - Retry handling.
    """

    def __init__(
        self,
        waypoints: list[tuple[float, float, float]]
    ) -> None:
        """
        Initialize the waypoint navigator.

        Args:
            waypoints: Sequence of (x, y, z) waypoint coordinates.
        """
        self.waypoints = waypoints

        self.navigator = BasicNavigator()

        self.goalPublisherNode = Node(
            'goal_position_publisher'
        )

        self.goalPublisher = (
            self.goalPublisherNode.create_publisher(
                Pose,
                GOAL_TOPIC,
                10
            )
        )

    def start(self) -> None:
        """Start navigation and process all configured waypoints."""
        print('Waiting for Nav2 to become active...')

        self.navigator.waitUntilNav2Active()

        print('Nav2 is active.')
        print(
            f'Starting navigation through '
            f'{len(self.waypoints)} waypoints.'
        )

        for waypointIndex, waypoint in enumerate(
            self.waypoints
        ):
            self.navigateToWaypoint(
                waypointIndex,
                waypoint
            )

    def navigateToWaypoint(
        self,
        waypointIndex: int,
        waypoint: tuple[float, float, float]
    ) -> bool:
        """
        Navigate to a single waypoint.

        Args:
            waypointIndex: Zero-based waypoint index.
            waypoint: Waypoint coordinates (x, y, z).

        Returns:
            True if the waypoint was reached successfully.
            False if all attempts failed.
        """
        print(
            f'\nNavigating to waypoint '
            f'{waypointIndex}: '
            f'({waypoint[0]:.3f}, {waypoint[1]:.3f})'
        )

        self.publishGoalPosition(waypoint)

        retryCount = 0

        while retryCount <= MAX_RETRIES:
            goalPose = self.createGoalPose(waypoint)

            print(
                f'Sending waypoint {waypointIndex} '
                f'to Nav2 '
                f'(attempt {retryCount + 1}/'
                f'{MAX_RETRIES + 1})...'
            )

            self.navigator.goToPose(goalPose)

            result = self.waitForNavigation()

            if result == TaskResult.SUCCEEDED:
                print(
                    f'✓ Goal {waypointIndex} succeeded!'
                )
                return True

            if result == TaskResult.CANCELED:
                print(
                    f'⚠ Goal {waypointIndex} was canceled!'
                )
                return False

            if result == TaskResult.FAILED:
                print(
                    f'✗ Goal {waypointIndex} failed!'
                )

                if retryCount < MAX_RETRIES:
                    retryCount += 1

                    print(
                        f'Retrying goal {waypointIndex} '
                        f'({retryCount}/{MAX_RETRIES})...'
                    )

                    continue

                print(
                    f'✗ Goal {waypointIndex} failed after '
                    f'{MAX_RETRIES + 1} attempts.'
                )

                return False

            print(
                f'⚠ Goal {waypointIndex} returned an '
                f'unknown navigation status.'
            )

            return False

        return False

    def waitForNavigation(self) -> TaskResult:
        """
        Wait until the current Nav2 task is complete.

        While navigation is running, estimated time remaining
        is printed periodically.

        Returns:
            The final Nav2 TaskResult.
        """
        while not self.navigator.isTaskComplete():
            time.sleep(FEEDBACK_INTERVAL_SECONDS)

            feedback = self.navigator.getFeedback()

            if feedback is not None:
                self.printNavigationFeedback(feedback)

        return self.navigator.getResult()

    @staticmethod
    def printNavigationFeedback(feedback) -> None:
        """Print the estimated navigation time remaining."""
        remainingSeconds = (
            Duration.from_msg(
                feedback.estimated_time_remaining
            ).nanoseconds
            / 1e9
        )

        print(
            'Estimated time of arrival: '
            f'{remainingSeconds:.0f} seconds.'
        )

    def publishGoalPosition(
        self,
        waypoint: tuple[float, float, float]
    ) -> None:
        """
        Publish the waypoint position on /goal_position.

        This topic is separate from the Nav2 action and can be used
        by other nodes or tools to visualize the currently selected
        goal.
        """
        goalPosition = Pose()

        goalPosition.position.x = waypoint[0]
        goalPosition.position.y = waypoint[1]
        goalPosition.position.z = waypoint[2]

        # The original code did not set orientation on this message.
        # Keep the same behavior here.

        self.goalPublisher.publish(goalPosition)

    def createGoalPose(
        self,
        waypoint: tuple[float, float, float]
    ) -> PoseStamped:
        """
        Create a PoseStamped message for Nav2.

        Args:
            waypoint: Waypoint coordinates (x, y, z).

        Returns:
            A PoseStamped configured for the map frame.
        """
        goalPose = PoseStamped()

        goalPose.header.frame_id = MAP_FRAME
        goalPose.header.stamp = (
            self.navigator.get_clock()
            .now()
            .to_msg()
        )

        goalPose.pose.position.x = waypoint[0]
        goalPose.pose.position.y = waypoint[1]
        goalPose.pose.position.z = waypoint[2]

        # Identity orientation.
        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = 0.0
        goalPose.pose.orientation.w = 1.0

        return goalPose

    def shutdown(self) -> None:
        """Release ROS 2 resources."""
        self.goalPublisherNode.destroy_node()


def main(args=None) -> int:
    """Application entry point."""
    rclpy.init(args=args)

    waypointNavigator = WaypointNavigator(WAYPOINTS)

    try:
        waypointNavigator.start()
        return 0

    except KeyboardInterrupt:
        print('\nNavigation interrupted by user.')
        return 1

    except Exception as exception:
        print(f'Navigation error: {exception}')
        return 1

    finally:
        waypointNavigator.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
