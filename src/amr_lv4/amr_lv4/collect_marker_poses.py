#!/usr/bin/env python3
"""
Skripta za prikupljanje pozicija ArUco markera pomoću RViz 2D Nav Goal alata.

Upute:
1. Pokreni simulaciju i RViz
2. Pokreni ovu skriptu: ros2 run amr_lv4 collect_marker_poses
3. U RVizu koristi "2D Nav Goal" alat za postavljanje 5 pozicija markera
4. Skripta će ispisati YAML konfiguraciju za aruco_markers.yaml
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import yaml


def quaternion_to_euler(x, y, z, w):
    """Pretvori quaternion u Euler kuteve (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MarkerPoseCollector(Node):
    def __init__(self):
        super().__init__('marker_pose_collector')
        
        self.marker_ids = [1, 5, 10, 15, 20]  # ID-jevi markera
        self.marker_height = 0.35  # Visina markera (visina kamere robota)
        
        self.poses = []
        self.current_index = 0
        self.total_markers = 5
        
        # Subscribe na goal_pose topic (2D Nav Goal u RVizu)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Marker Pose Collector')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Postavi {self.total_markers} pozicija markera koristeci "2D Nav Goal" u RVizu')
        self.get_logger().info(f'Marker ID-jevi: {self.marker_ids}')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Cekam poziciju za marker ID {self.marker_ids[0]}...')

    def goal_callback(self, msg: PoseStamped):
        if self.current_index >= self.total_markers:
            return
        
        marker_id = self.marker_ids[self.current_index]
        
        # Izvuci poziciju
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = self.marker_height  # Koristimo fiksnu visinu
        
        # Izvuci orijentaciju (quaternion -> euler)
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
        
        # Za marker na zidu: roll=90° (okomit na pod), yaw prema smjeru strelice
        marker_data = {
            'id': marker_id,
            'x': round(x, 3),
            'y': round(y, 3),
            'z': z,
            'roll': round(math.pi / 2, 4),  # 1.5708 - marker okomit na pod
            'pitch': 0.0,
            'yaw': round(yaw, 4)
        }
        
        self.poses.append(marker_data)
        self.current_index += 1
        
        self.get_logger().info(f'Marker {marker_id}: x={x:.3f}, y={y:.3f}, yaw={yaw:.4f} rad ({math.degrees(yaw):.1f}°)')
        
        if self.current_index < self.total_markers:
            next_id = self.marker_ids[self.current_index]
            self.get_logger().info(f'Cekam poziciju za marker ID {next_id}... ({self.current_index}/{self.total_markers})')
        else:
            self.get_logger().info('=' * 60)
            self.get_logger().info('Svi markeri prikupljeni! Generiranje YAML konfiguracije...')
            self.get_logger().info('=' * 60)
            self.print_yaml_config()

    def print_yaml_config(self):
        """Ispiši YAML konfiguraciju za aruco_markers.yaml."""
        
        yaml_data = {'markers': self.poses}
        
        print('\n' + '=' * 60)
        print('YAML KONFIGURACIJA - kopiraj u aruco_markers.yaml:')
        print('=' * 60 + '\n')
        
        # Formatirani ispis
        print("# ArUco markeri - generirano pomocu collect_marker_poses.py")
        print("# DICT_4X4_50, velicina 0.2m")
        print("markers:")
        
        for pose in self.poses:
            print(f"  - id: {pose['id']}")
            print(f"    x: {pose['x']}")
            print(f"    y: {pose['y']}")
            print(f"    z: {pose['z']}")
            print(f"    roll: {pose['roll']}")
            print(f"    pitch: {pose['pitch']}")
            print(f"    yaw: {pose['yaw']}")
            print()
        
        print('=' * 60)
        print('Gotovo! Kopiraj gornju konfiguraciju u:')
        print('amr_lv4/config/aruco_markers.yaml')
        print('=' * 60)
        
        # Također spremi u datoteku
        output_path = '/tmp/aruco_markers_collected.yaml'
        with open(output_path, 'w') as f:
            f.write("# ArUco markeri - generirano pomocu collect_marker_poses.py\n")
            f.write("# DICT_4X4_50, velicina 0.2m\n")
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info(f'Konfiguracija spremljena u: {output_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
