from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='patrol_points.yaml',
        description='Input file path containing patrol points'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='1.0',
        description='Safe distance from detected object in meters'
    )
    
    wait_time_arg = DeclareLaunchArgument(
        'wait_time',
        default_value='3.0',
        description='Time to wait at object location in seconds'
    )
    
    # Create node for patrolling navigation
    patrolling_point_navigator_node = Node(
        package='amr_lv3',
        executable='patrolling_point_navigator',
        name='patrolling_point_navigator',
        output='screen',
        parameters=[{
            'input_file': LaunchConfiguration('input_file'),
            'safe_distance': LaunchConfiguration('safe_distance'),
            'wait_time': LaunchConfiguration('wait_time')
        }]
    )
    
    return LaunchDescription([
        input_file_arg,
        safe_distance_arg,
        wait_time_arg,
        patrolling_point_navigator_node
    ])
