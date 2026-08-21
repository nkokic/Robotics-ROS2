"""Spawn configured ArUco markers and start the ArUco patrol node."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the combined marker spawning and patrol workflow."""
    package_share = FindPackageShare('amr_lv4')

    arguments = [
        DeclareLaunchArgument(
            'marker_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'aruco_markers.yaml']),
            description='Path to the marker YAML used for Gazebo spawning'),
        DeclareLaunchArgument(
            'patrol_file',
            default_value=PathJoinSubstitution([
                package_share, 'patrol_points.yaml']),
            description='Path to the patrol point YAML'),
        DeclareLaunchArgument(
            'patrol_start_delay', default_value='3.0',
            description='Seconds to wait before starting the patrol node'),
        DeclareLaunchArgument(
            'wait_time', default_value='5.0',
            description='Seconds to wait in front of a detected marker'),
        DeclareLaunchArgument(
            'stop_distance', default_value='1.0',
            description='Robot stopping distance from a detected marker'),
        DeclareLaunchArgument(
            'detection_cooldown', default_value='30.0',
            description=(
                'Seconds before the same marker may be detected again')),
        DeclareLaunchArgument(
            'marker_size', default_value='0.2',
            description='Physical marker side length in metres'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock'),
    ]

    spawn_markers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share, 'launch', 'spawn_aruco_markers.launch.py'])),
        launch_arguments={
            'marker_file': LaunchConfiguration('marker_file'),
        }.items(),
    )

    patrol_node = Node(
        package='amr_lv4',
        executable='aruco_patrol',
        name='aruco_patrol_node',
        output='screen',
        parameters=[{
            'patrol_file': LaunchConfiguration('patrol_file'),
            'wait_time': ParameterValue(
                LaunchConfiguration('wait_time'), value_type=float),
            'stop_distance': ParameterValue(
                LaunchConfiguration('stop_distance'), value_type=float),
            'detection_cooldown': ParameterValue(
                LaunchConfiguration('detection_cooldown'), value_type=float),
            'marker_size': ParameterValue(
                LaunchConfiguration('marker_size'), value_type=float),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    delayed_patrol = TimerAction(
        period=LaunchConfiguration('patrol_start_delay'),
        actions=[patrol_node],
    )
    return LaunchDescription(arguments + [spawn_markers, delayed_patrol])
