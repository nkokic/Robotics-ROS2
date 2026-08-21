#!/usr/bin/env python3
"""Collect patrol points and ArUco poses while visualizing them in RViz."""

import math
import os

from geometry_msgs.msg import PointStamped, PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from visualization_msgs.msg import Marker, MarkerArray

import yaml


def quaternion_yaw(quaternion):
    """Return the planar yaw angle represented by a quaternion."""
    numerator = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    denominator = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(numerator, denominator)


class PatrolAndMarkerCollector(Node):
    """Collect both kinds of map annotations used by the AMR exercise."""

    def __init__(self):
        """Initialize subscriptions, parameters, and visualization output."""
        super().__init__('patrol_and_marker_collector')
        self.declare_parameter('max_points', 5)
        self.declare_parameter('patrol_output_file', 'patrol_points.yaml')
        self.declare_parameter('marker_ids', [1, 5, 10, 15, 20])
        self.declare_parameter(
            'marker_output_file', '/tmp/aruco_markers_collected.yaml')
        self.declare_parameter('marker_height', 0.35)

        self.max_points = self.get_parameter('max_points').value
        self.patrol_output = self.get_parameter(
            'patrol_output_file').value
        self.marker_ids = list(self.get_parameter('marker_ids').value)
        self.marker_output = self.get_parameter(
            'marker_output_file').value
        self.marker_height = self.get_parameter('marker_height').value
        self.patrol_points = []
        self.marker_poses = []

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'collected_points', marker_qos)
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.get_logger().info(
            'Use Publish Point for patrol points and 2D Goal Pose for '
            'ArUco markers in RViz.')
        self.get_logger().info(
            f'Patrol points: 0/{self.max_points}; marker IDs: '
            f'{self.marker_ids}')

    @staticmethod
    def ensure_parent_directory(path):
        """Create the output parent directory when needed."""
        parent = os.path.dirname(os.path.abspath(os.path.expanduser(path)))
        os.makedirs(parent, exist_ok=True)

    def clicked_point_callback(self, message):
        """Store and visualize an RViz Publish Point event."""
        if len(self.patrol_points) >= self.max_points:
            self.get_logger().warning('Maximum patrol point count reached.')
            return
        point = {
            'x': float(message.point.x),
            'y': float(message.point.y),
            'z': float(message.point.z),
            '_frame': message.header.frame_id or 'map',
        }
        self.patrol_points.append(point)
        self.save_patrol_points()
        self.publish_visualization()
        count = len(self.patrol_points)
        self.get_logger().info(
            f'Patrol point {count}/{self.max_points}: '
            f'({point["x"]:.2f}, {point["y"]:.2f}, {point["z"]:.2f})')

    def goal_pose_callback(self, message):
        """Store and visualize an RViz 2D Goal Pose event."""
        index = len(self.marker_poses)
        if index >= len(self.marker_ids):
            self.get_logger().warning('All marker poses are collected.')
            return
        pose = {
            'id': int(self.marker_ids[index]),
            'x': round(float(message.pose.position.x), 3),
            'y': round(float(message.pose.position.y), 3),
            'z': float(self.marker_height),
            'roll': round(math.pi / 2.0, 4),
            'pitch': 0.0,
            'yaw': round(quaternion_yaw(message.pose.orientation), 4),
            '_frame': message.header.frame_id or 'map',
        }
        self.marker_poses.append(pose)
        self.save_marker_poses()
        self.publish_visualization()
        self.get_logger().info(
            f'Marker {pose["id"]} ({index + 1}/{len(self.marker_ids)}): '
            f'({pose["x"]:.2f}, {pose["y"]:.2f}), yaw={pose["yaw"]:.2f}')

    def save_patrol_points(self):
        """Write collected patrol points to YAML."""
        self.ensure_parent_directory(self.patrol_output)
        points = [
            {key: value for key, value in point.items() if key != '_frame'}
            for point in self.patrol_points
        ]
        with open(self.patrol_output, 'w', encoding='utf-8') as output:
            yaml.safe_dump({'patrol_points': points}, output, sort_keys=False)

    def save_marker_poses(self):
        """Write collected marker poses to YAML."""
        self.ensure_parent_directory(self.marker_output)
        poses = [
            {key: value for key, value in pose.items() if key != '_frame'}
            for pose in self.marker_poses
        ]
        with open(self.marker_output, 'w', encoding='utf-8') as output:
            output.write(
                '# ArUco markers generated by collect_patrol_and_markers\n')
            yaml.safe_dump({'markers': poses}, output, sort_keys=False)

    def publish_visualization(self):
        """Publish persistent spheres, arrows, and labels for RViz."""
        result = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(self.patrol_points):
            marker = self.make_marker(
                point['_frame'], stamp, 'patrol_points', index,
                Marker.SPHERE, point['x'], point['y'], point['z'] + 0.10)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.20
            marker.color.r = 0.10
            marker.color.g = 0.75
            marker.color.b = marker.color.a = 1.0
            result.markers.append(marker)
            result.markers.append(self.make_label(
                marker, 'patrol_labels', f'P{index + 1}',
                point['z'] + 0.35, 1.0, 1.0, 1.0))

        for index, pose in enumerate(self.marker_poses):
            arrow = self.make_marker(
                pose['_frame'], stamp, 'aruco_markers', index,
                Marker.ARROW, pose['x'], pose['y'], pose['z'])
            half_yaw = pose['yaw'] / 2.0
            arrow.pose.orientation.z = math.sin(half_yaw)
            arrow.pose.orientation.w = math.cos(half_yaw)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.45, 0.09, 0.09
            arrow.color.r = arrow.color.a = 1.0
            arrow.color.g = 0.55
            arrow.color.b = 0.05
            result.markers.append(arrow)
            result.markers.append(self.make_label(
                arrow, 'aruco_labels', f'ArUco {pose["id"]}',
                pose['z'] + 0.30, 1.0, 0.75, 0.15))
        self.marker_publisher.publish(result)

    @staticmethod
    def make_marker(frame, stamp, namespace, marker_id, marker_type,
                    x, y, z):
        """Construct a marker with common pose fields."""
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def make_label(source, namespace, text, z, red, green, blue):
        """Construct a text label for a point marker."""
        label = Marker()
        label.header = source.header
        label.ns = namespace
        label.id = source.id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = source.pose.position.x
        label.pose.position.y = source.pose.position.y
        label.pose.position.z = z
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r, label.color.g, label.color.b = red, green, blue
        label.color.a = 1.0
        label.text = text
        return label


def main(args=None):
    """Run the combined collector."""
    rclpy.init(args=args)
    node = PatrolAndMarkerCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
