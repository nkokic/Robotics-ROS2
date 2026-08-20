from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maxPointsArgument = DeclareLaunchArgument(
        'max_points',
        default_value='5',
        description='Maximum number of patrol points to gather',
    )
    outputFileArgument = DeclareLaunchArgument(
        'output_file',
        default_value='patrol_points.yaml',
        description='Output file path for patrol points',
    )
    gatherPointsNode = Node(
        package='amr_lv4',
        executable='gather_points',
        output='screen',
        parameters=[{
            'max_points': LaunchConfiguration('max_points'),
            'output_file': LaunchConfiguration('output_file'),
        }],
    )
    return LaunchDescription([
        maxPointsArgument,
        outputFileArgument,
        gatherPointsNode,
    ])
