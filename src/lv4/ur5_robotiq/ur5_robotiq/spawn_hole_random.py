import os
import random
import subprocess

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
random.seed(None)

class HoleSpawner(Node):
    def __init__(self):
        super().__init__('hole_spawner')

        # Nominal pose on table
        self.nominal_x = self.declare_parameter('nominal_x', 0.0).value
        self.nominal_y = self.declare_parameter('nominal_y', 0.0).value
        self.nominal_z = self.declare_parameter('nominal_z', 0.8).value

        # Max random offsets (meters, radians)
        self.max_xy_error = self.declare_parameter('max_xy_error', 0.005).value  # ±5 mm

        self.world = self.declare_parameter('world_name', 'empty').value

        self.spawn()

    def spawn(self):
        # Sample random offsets
        dx = 0
        dy = 0
        while abs(dx) < 0.003 and abs(dy) < 0.003:
            dx = random.uniform(-self.max_xy_error, self.max_xy_error)
            dy = random.uniform(-self.max_xy_error, self.max_xy_error)

        x = self.nominal_x + dx
        y = self.nominal_y + dy
        z = self.nominal_z

        hole_urdf = os.path.join(
            get_package_share_directory('ur5_robotiq'),
            'models', 'hole.sdf'
        )

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world,
            '-name', 'hole',
            '-file', hole_urdf,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ]
        subprocess.run(cmd, check=True)

        # Once spawned, we can shut down this node
        self.get_logger().info("Hole spawned, shutting down")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HoleSpawner()


if __name__ == '__main__':
    main()