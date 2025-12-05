from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..'
    )

    return LaunchDescription([
        Node(
            package='nrs_lv2_zad',
            executable='test_moveit',
            name='test_moveit'
        ),

        Node(
            package='nrs_lv2_zad',
            executable='RecordHand',
            name='record_hand'
        )
    ])
