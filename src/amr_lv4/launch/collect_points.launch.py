"""Launch the combined patrol-point and ArUco-pose collector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the collector launch description."""
    arguments = [
        DeclareLaunchArgument(
            'max_points', default_value='5',
            description='Maximum number of patrol points'),
        DeclareLaunchArgument(
            'patrol_output_file', default_value='patrol_points.yaml',
            description='Patrol point YAML output path'),
        DeclareLaunchArgument(
            'marker_output_file',
            default_value='/tmp/aruco_markers_collected.yaml',
            description='ArUco marker YAML output path'),
        DeclareLaunchArgument(
            'marker_height', default_value='0.35',
            description='Fixed ArUco marker height in metres'),
    ]
    collector = Node(
        package='amr_lv4',
        executable='collect_patrol_and_markers',
        name='patrol_and_marker_collector',
        output='screen',
        parameters=[{
            'max_points': LaunchConfiguration('max_points'),
            'patrol_output_file': LaunchConfiguration('patrol_output_file'),
            'marker_output_file': LaunchConfiguration('marker_output_file'),
            'marker_height': LaunchConfiguration('marker_height'),
        }],
    )
    return LaunchDescription(arguments + [collector])
