#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityFactory

class CabinetSpawner(Node):
    def __init__(self, urdf_path, initial_pose, world_name: str = "default"):
        super().__init__('cabinet_spawner')

        # Resolve absolute path and validate
        self.urdf_path = os.path.abspath(urdf_path)
        if not os.path.isfile(self.urdf_path):
            self.get_logger().error(f"URDF file not found: {self.urdf_path}")

        # Publisher to Gazebo Harmonic
        topic = f"/world/{world_name}/create"
        self.pub = self.create_publisher(
            EntityFactory,
            topic,
            10
        )
        self.get_logger().info(f"Publishing EntityFactory to topic: {topic}")
        self.initial_pose = initial_pose
        # Delay spawning slightly so node has time to connect
        self.timer = self.create_timer(1.0, self.spawn_once)

        self.has_spawned = False

    def spawn_once(self):
        if self.has_spawned:
            return

        self.has_spawned = True

        # Load URDF text
        try:
            with open(self.urdf_path, 'r') as f:
                urdf_xml = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read URDF '{self.urdf_path}': {e}")
            return

        msg = EntityFactory()
        msg.name = "cabinet"
        msg.allow_renaming = True
        msg.sdf = urdf_xml

        # Initial pose
        msg.pose.position.x = self.initial_pose[0]
        msg.pose.position.y = self.initial_pose[1]
        msg.pose.position.z = self.initial_pose[2]

        msg.pose.orientation.x = self.initial_pose[3]
        msg.pose.orientation.y = self.initial_pose[4]
        msg.pose.orientation.z = self.initial_pose[5]
        msg.pose.orientation.w = self.initial_pose[6]

        self.pub.publish(msg)
        self.get_logger().info(f"Spawn request sent for URDF: {self.urdf_path}")


def main(args=None):
    rclpy.init(args=args)

    # Use path relative to workspace root; will be resolved to absolute
    urdf_path = "./nrs_lv_ws/src/lv3/cabinet.urdf"
    initial_pose = [0., 0.45, 0.8, 0., 0., -0.706825, 0.707388]
    node = CabinetSpawner(urdf_path, initial_pose, world_name="default")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()