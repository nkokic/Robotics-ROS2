#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import tf2_ros


class ArmTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('arm_trajectory_publisher')

        # TF Buffer i listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher za Path
        self.path_pub = self.create_publisher(Path, "/arm_path", 10)

        # Strukturirana Path poruka
        self.path_msg = Path()
        self.path_msg.header.frame_id = "base_link"

        # Timer za periodičko dohvaćanje TF-a
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Ciljani link
        self.target_frame = "hand"
        self.reference_frame = "base_link"

        self.get_logger().info("ArmTrajectoryPublisher started.")

    def timer_callback(self):
        try:
            # Dohvat transformacije hand -> base_link
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time()
            )

            # Kreiranje točke u Path-u
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            # Rotacija nije bitna za vizualizaciju putanje, ali može se dodati:
            pose.pose.orientation = transform.transform.rotation

            # Dodavanje u Path
            self.path_msg.header.stamp = pose.header.stamp
            self.path_msg.poses.append(pose)

            # Slanje Path poruke
            self.path_pub.publish(self.path_msg)

        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f"TF not available: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
