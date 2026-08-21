#!/usr/bin/env python3
# zadatak2_mission.py

import math
import time

import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty


class DroneMissionController(Node):
    NODE_NAME: str = "drone_mission_controller"

    CMD_VEL_TOPIC: str = "/cmd_vel"
    TAKEOFF_TOPIC: str = "/takeoff"
    LAND_TOPIC: str = "/land"
    ARUCO_POSE_TOPIC: str = "/aruco_pose_detected"
    ODOM_TOPIC: str = "/odom"

    VALID_MARKER_IDS: tuple[int, ...] = (23, 42)

    DEFAULT_TARGET_DISTANCE: float = 0.6
    DEFAULT_SEARCH_SPEED: float = 1

    REQUIRED_DETECTIONS: int = 5

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # Parameters
        self.declare_parameter(
            "target_distance",
            self.DEFAULT_TARGET_DISTANCE
        )

        self.declare_parameter(
            "search_speed",
            self.DEFAULT_SEARCH_SPEED
        )

        self._targetDistance: float = self.get_parameter(
            "target_distance"
        ).value

        self._searchSpeed: float = self.get_parameter(
            "search_speed"
        ).value

        # Controller gains
        self._linearKp: float = 1
        self._angularKp: float = 1

        # Publishers
        self._cmdVelPublisher = self.create_publisher(
            Twist,
            self.CMD_VEL_TOPIC,
            10
        )

        self._takeoffPublisher = self.create_publisher(
            Empty,
            self.TAKEOFF_TOPIC,
            1
        )

        self._landPublisher = self.create_publisher(
            Empty,
            self.LAND_TOPIC,
            1
        )

        # Subscribers
        self._arucoSubscription = self.create_subscription(
            PoseStamped,
            self.ARUCO_POSE_TOPIC,
            self.ArucoDataCallback,
            10
        )

        self._odomSubscription = self.create_subscription(
            Odometry,
            self.ODOM_TOPIC,
            self.OdomCallback,
            10
        )

        # Mission state
        self._state: str = "INIT"

        self._startPose = None
        self._currentPose = None

        self._stateStartTime: float = 0.0

        # Marker tracking
        self._visitedMarkers: set[int] = set()

        self._currentTargetMarker: int | None = None
        self._markerPosition: tuple[float, float, float] | None = None

        # Detection confirmation
        self._detectionCounter: int = 0
        self._lastSeenId: int = -1
        self._lastMessageTime: float = 0.0

        self.get_logger().info(
            "Controller is waiting for pose request..."
        )

        # 20 Hz control loop
        self._controlTimer = self.create_timer(
            0.05,
            self.ControlLoop
        )

    def OdomCallback(self, message: Odometry) -> None:
        """Updates the current drone pose."""

        self._currentPose = message.pose.pose

        if self._startPose is None and self._state == "INIT":
            self._startPose = message.pose.pose

    def ArucoDataCallback(self, message: PoseStamped) -> None:
        """Receives marker data produced by aruco detector."""

        try:
            # Stores the marker ID in header.frame_id.
            markerId = int(message.header.frame_id)

        except ValueError:
            return

        # Ignore markers that have already been completed,
        # except while backing away from a marker.
        if (
            markerId in self._visitedMarkers
            and self._state != "BACKING_UP"
        ):
            return

        # Detection debouncing
        if markerId == self._lastSeenId:
            self._detectionCounter += 1

        else:
            self._detectionCounter = 1
            self._lastSeenId = markerId

        # Save marker position relative to camera.
        self._markerPosition = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z
        )

        self._lastMessageTime = time.time()

        # Confirm marker after several consecutive detections.
        if self._detectionCounter >= self.REQUIRED_DETECTIONS:
            self._currentTargetMarker = markerId

            # Keep the counter saturated.
            self._detectionCounter = self.REQUIRED_DETECTIONS

    def SendVelocity(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0
    ) -> None:
        """Publishes a velocity command to the drone."""

        velocityMessage = Twist()

        velocityMessage.linear.x = float(x)
        velocityMessage.linear.y = float(y)
        velocityMessage.linear.z = float(z)

        velocityMessage.angular.z = float(yaw)

        self._cmdVelPublisher.publish(velocityMessage)

    def ControlLoop(self) -> None:
        """Main mission state machine."""

        currentTime = time.time()

        # ---------------------------------------------------------
        # 1. TAKEOFF
        # ---------------------------------------------------------

        if self._state == "INIT":

            if self._startPose is not None:
                self._takeoffPublisher.publish(Empty())

                self._stateStartTime = currentTime
                self._state = "TAKING_OFF"

        elif self._state == "TAKING_OFF":

            if currentTime - self._stateStartTime < 8.0:
                self.SendVelocity(
                    x=0.0,
                    y=0.0,
                    z=0.3,
                    yaw=0.0
                )

            else:
                self.SendVelocity()
                self._state = "SEARCHING"

        # ---------------------------------------------------------
        # 2. SEARCHING
        # ---------------------------------------------------------

        elif self._state == "SEARCHING":

            allMarkersVisited = all(
                markerId in self._visitedMarkers
                for markerId in self.VALID_MARKER_IDS
            )

            if allMarkersVisited:
                self.get_logger().info(
                    "Returning to home position.",
                    throttle_duration_sec=2.0
                )

                self._state = "RETURNING"
                return

            markerRecentlySeen = (
                currentTime - self._lastMessageTime < 0.5
            )

            markerConfirmed = (
                self._detectionCounter >= self.REQUIRED_DETECTIONS
            )

            if markerRecentlySeen and markerConfirmed:

                self.get_logger().info(
                    f"Investigating marker {self._currentTargetMarker}"
                )

                self.SendVelocity()
                self._state = "APPROACHING"

            else:
                self.get_logger().info(
                    "Scanning...",
                    throttle_duration_sec=2.0
                )

                self.SendVelocity(
                    yaw=self._searchSpeed
                )

        # ---------------------------------------------------------
        # 3. APPROACHING MARKER
        # ---------------------------------------------------------

        elif self._state == "APPROACHING":

            signalLost = (
                currentTime - self._lastMessageTime > 2.0
            )

            if signalLost:
                self.get_logger().warning(
                    "Lost signal. Standing by."
                )

                self.SendVelocity()
                self._state = "SEARCHING"

                return

            if self._markerPosition is None:
                return

            markerX, markerY, markerZ = self._markerPosition

            distanceError = (
                markerZ - self._targetDistance
            )

            # Marker reached.
            if distanceError < 0.1:

                self.get_logger().info(
                    f"Done investigating marker {self._currentTargetMarker}"
                )

                if self._currentTargetMarker is not None:
                    self._visitedMarkers.add(
                        self._currentTargetMarker
                    )

                self.SendVelocity()

                self._state = "BACKING_UP"
                self._stateStartTime = currentTime

                return

            # Proportional control
            forwardVelocity = np.clip(
                self._linearKp * distanceError,
                -1.0,
                1.0
            )

            yawVelocity = np.clip(
                -self._angularKp * markerX,
                -1.0,
                1.0
            )

            verticalVelocity = np.clip(
                -self._linearKp * markerY,
                -1.0,
                1.0
            )

            self.SendVelocity(
                x=forwardVelocity,
                y=0.0,
                z=verticalVelocity,
                yaw=yawVelocity
            )

        # ---------------------------------------------------------
        # 4. BACKING AWAY
        # ---------------------------------------------------------

        elif self._state == "BACKING_UP":

            if currentTime - self._stateStartTime < 2.0:

                self.SendVelocity(
                    x=-0.3
                )

            else:
                self.SendVelocity()

                self._state = "SEARCHING"

                self._detectionCounter = 0
                self._currentTargetMarker = None
                self._markerPosition = None

        # ---------------------------------------------------------
        # 5. RETURN HOME
        # ---------------------------------------------------------

        elif self._state == "RETURNING":

            if (
                self._currentPose is None
                or self._startPose is None
            ):
                return

            currentX = self._currentPose.position.x
            currentY = self._currentPose.position.y

            startX = self._startPose.position.x
            startY = self._startPose.position.y

            deltaX = startX - currentX
            deltaY = startY - currentY

            distanceToHome = math.sqrt(
                deltaX ** 2 + deltaY ** 2
            )

            # Drone reached the starting position.
            if distanceToHome < 0.4:

                self.SendVelocity()

                self._state = "LANDING"
                self._stateStartTime = currentTime

                return

            targetYaw = math.atan2(
                deltaY,
                deltaX
            )

            orientation = self._currentPose.orientation

            currentYaw = math.atan2(
                2.0 * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0 - 2.0 * (
                    orientation.y ** 2
                    + orientation.z ** 2
                )
            )

            yawError = targetYaw - currentYaw
            yawError = self.NormalizeAngle(yawError)

            # First rotate toward home.
            if abs(yawError) > 0.3:

                self.SendVelocity(
                    yaw=0.5 * np.sign(yawError)
                )

            # Then move forward while correcting orientation.
            else:

                self.SendVelocity(
                    x=0.3,
                    yaw=0.5 * yawError
                )

        # ---------------------------------------------------------
        # 6. LANDING
        # ---------------------------------------------------------

        elif self._state == "LANDING":

            self.SendVelocity(
                z=-0.3
            )

            self._landPublisher.publish(
                Empty()
            )

            if currentTime - self._stateStartTime > 8.0:

                self.SendVelocity()

                self._state = "FINISHED"

                self.get_logger().info(
                    "Landed!"
                )

        # ---------------------------------------------------------
        # 7. FINISHED
        # ---------------------------------------------------------

        elif self._state == "FINISHED":
            self.SendVelocity()

    @staticmethod
    def NormalizeAngle(angle: float) -> float:
        """Normalizes an angle into the [-pi, pi] range."""

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


def Main(args=None) -> None:
    rclpy.init(args=args)

    node = DroneMissionController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    Main()