#!/usr/bin/env python3
# zadatak1_detector.py
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv2 import aruco
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('marker_size', 0.12)
        self.marker_size = self.get_parameter('marker_size').value
        self.valid_ids = [23, 42]

        # Publisheri
        self.tf_broadcaster = TransformBroadcaster(self)
        # Ovdje šaljemo podatke za Zadatak 2
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose_detected', 10)

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_info_callback, 10)

        # CV Alati
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.minMarkerPerimeterRate = 0.03
        
        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info("Z1: Detektor pokrenut. Šaljem podatke na /aruco_pose_detected")

    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception: return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in self.valid_ids: continue

                # 1. TF Broadcast (Zadatak 1 dio A)
                self.broadcast_tf(marker_id, tvecs[i][0], msg.header)
                
                # 2. Slanje podataka na Topic (Zadatak 1 dio B -> za Zadatak 2)
                self.publish_pose(marker_id, tvecs[i][0], msg.header)

    def broadcast_tf(self, marker_id, tvec, header):
        t = TransformStamped()
        t.header = header
        t.child_frame_id = f"aruco_marker_{marker_id}"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tvec
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self, marker_id, tvec, header):
        # Pakiramo podatke u PoseStamped poruku
        # U header.frame_id (koji je string) ćemo "prošvercati" ID markera
        # da Zadatak 2 zna koji marker gleda.
        msg = PoseStamped()
        msg.header = header
        msg.header.frame_id = str(marker_id) # <--- OVDJE ŠALJEMO ID
        msg.pose.position.x = tvec[0]
        msg.pose.position.y = tvec[1]
        msg.pose.position.z = tvec[2]
        self.pose_pub.publish(msg)

def main(args=None):
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

if __name__ == '__main__':
    main()