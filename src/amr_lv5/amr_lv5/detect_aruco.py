#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class ArucoDetectorNode(Node):
    NODE_NAME: str = "aruco_detector_node"

    IMAGE_TOPIC: str = "/camera/image_raw"
    CAMERA_INFO_TOPIC: str = "/camera/camera_info"
    POSE_TOPIC: str = "/aruco_pose_detected"

    DEFAULT_MARKER_SIZE: float = 0.12
    VALID_MARKER_IDS: tuple[int, ...] = (23, 42)

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # Parameters
        self.declare_parameter("marker_size", self.DEFAULT_MARKER_SIZE)
        self._markerSize: float = self.get_parameter("marker_size").value

        # ROS publishers / broadcasters
        self._tfBroadcaster = TransformBroadcaster(self)

        self._posePublisher = self.create_publisher(
            PoseStamped,
            self.POSE_TOPIC,
            10
        )

        # ROS subscriptions
        self._imageSubscription = self.create_subscription(
            Image,
            self.IMAGE_TOPIC,
            self.ImageCallback,
            10
        )

        self._cameraInfoSubscription = self.create_subscription(
            CameraInfo,
            self.CAMERA_INFO_TOPIC,
            self.CameraInfoCallback,
            10
        )

        # OpenCV tools
        self._bridge = CvBridge()
        self._arucoDictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self._arucoParameters = aruco.DetectorParameters_create()

        self._arucoParameters.minMarkerPerimeterRate = 0.03

        # Camera calibration
        self._cameraMatrix: np.ndarray | None = None
        self._distortionCoefficients: np.ndarray | None = None

        self.get_logger().info(
            f"Detection started. Broadcasting to: {self.POSE_TOPIC}"
        )

    def CameraInfoCallback(self, message: CameraInfo) -> None:
        """Stores the camera calibration data once it becomes available."""

        if self._cameraMatrix is not None:
            return

        self._cameraMatrix = np.array(message.k).reshape((3, 3))
        self._distortionCoefficients = np.array(message.d)

    def ImageCallback(self, message: Image) -> None:
        """Detects ArUco markers and publishes their positions."""

        if self._cameraMatrix is None:
            return

        try:
            cvImage = self._bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as exception:
            self.get_logger().warning(
                f"Could not convert ROS image to OpenCV image: {exception}"
            )
            return

        grayImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)

        corners, markerIds, _ = aruco.detectMarkers(
            grayImage,
            self._arucoDictionary,
            parameters=self._arucoParameters
        )

        if markerIds is None:
            return

        _, translationVectors, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self._markerSize,
            self._cameraMatrix,
            self._distortionCoefficients
        )

        for index, markerIdArray in enumerate(markerIds):
            markerId = int(markerIdArray[0])

            if markerId not in self.VALID_MARKER_IDS:
                continue

            translationVector = translationVectors[index][0]

            self.BroadcastTransform(
                markerId,
                translationVector,
                message.header
            )

            self.PublishPose(
                markerId,
                translationVector,
                message.header
            )

    def BroadcastTransform(
        self,
        markerId: int,
        translationVector: np.ndarray,
        header
    ) -> None:
        """Broadcasts the marker position through TF."""

        transform = TransformStamped()

        transform.header = header
        transform.child_frame_id = f"aruco_marker_{markerId}"

        transform.transform.translation.x = float(translationVector[0])
        transform.transform.translation.y = float(translationVector[1])
        transform.transform.translation.z = float(translationVector[2])

        # No marker rotation is currently used.
        transform.transform.rotation.w = 1.0

        self._tfBroadcaster.sendTransform(transform)

    def PublishPose(
        self,
        markerId: int,
        translationVector: np.ndarray,
        header
    ) -> None:
        """Publishes the detected marker position."""

        poseMessage = PoseStamped()

        poseMessage.header = header

        # Marker ID is stored in frame_id so Zadatak 2 knows
        # which marker produced this pose.
        poseMessage.header.frame_id = str(markerId)

        poseMessage.pose.position.x = float(translationVector[0])
        poseMessage.pose.position.y = float(translationVector[1])
        poseMessage.pose.position.z = float(translationVector[2])

        poseMessage.pose.orientation.w = 1.0

        self._posePublisher.publish(poseMessage)


def Main(args=None) -> None:
    rclpy.init(args=args)

    node = ArucoDetectorNode()

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