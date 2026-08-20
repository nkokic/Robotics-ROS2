from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    max_points_arg = DeclareLaunchArgument(
        'max_points',
        default_value='5',
        description='Maximum number of patrol points to gather'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='patrol_points.yaml',
        description='Output file path for patrol points'
    )
    
    # Create node for gathering patrol points
    patrolling_point_gatherer_node = Node(
        package='amr_lv3',
        executable='patrolling_point_gatherer',
        name='patrolling_point_gatherer',
        output='screen',
        parameters=[{
            'max_points': LaunchConfiguration('max_points'),
            'output_file': LaunchConfiguration('output_file')
        }]
    )
    
    return LaunchDescription([
        max_points_arg,
        output_file_arg,
        patrolling_point_gatherer_node
    ])
