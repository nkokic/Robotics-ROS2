import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tello_launch = os.path.join(
        get_package_share_directory('tello_bringup'),
        'launch',
        'tello.launch.py',
    )
    quadcopter_launch = os.path.join(
        get_package_share_directory('quadcopter_bringup'),
        'launch',
        'quadcopter_basic.launch.xml',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tello_launch),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(quadcopter_launch),
        ),
    ])
