from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_launch = PathJoinSubstitution([
        FindPackageShare('amr_lv5'),
        'launch',
        'amr_lv5.launch.py',
    ])
    quadcopter_launch = PathJoinSubstitution([
        FindPackageShare('quadcopter_bringup'),
        'launch',
        'quadcopter_basic.launch.xml',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amr_launch),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('tello_bringup'),
                    'config',
                    'rviz.rviz',
                ]),
            ],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(quadcopter_launch),
        ),
    ])
