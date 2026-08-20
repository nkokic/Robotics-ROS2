from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    inputFileArgument = DeclareLaunchArgument(
        'input_file',
        default_value='patrol_points.yaml',
        description='Input file path containing patrol points'
    )
    
    
    # Create node for patrolling navigation
    patrollingPointNode = Node(
        package='amr_lv4',
        executable='patrol_points',
        name='patrol_points',
        output='screen',
        parameters=[{
            'input_file': LaunchConfiguration('input_file'),
        }]
    )
    
    return LaunchDescription([
        inputFileArgument,
        patrollingPointNode
    ])
