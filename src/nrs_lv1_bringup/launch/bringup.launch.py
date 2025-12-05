from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..'
    )
    bag_dir = os.path.join(pkg_share, 'bag')

    # Provjeri da direktorij postoji
    os.makedirs(bag_dir, exist_ok=True)

    return LaunchDescription([
        # Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            parameters=[{'x': 5.544, 'y': 5.544, 'theta': -3.14}]
        ),

        # Teleport kornjače na početnu poziciju
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/turtle1/teleport_absolute',
                'turtlesim/srv/TeleportAbsolute',
                '{x: 5.544, y: 5.544, theta: 0.785}'
            ],
            shell=False
        ),

        # TurtleControl čvor
        Node(
            package='nrs_lv1',
            executable='turtleControl',
            name='turtle_control'
        ),

        # Znamenitosti čvor
        Node(
            package='nrs_lv1',
            executable='znamenitostiNode',
            name='znamenitosti_node',
            parameters=[{
                'period': 0.5,       # periodično objavljivanje
                'prefiks': 'Polozaj' # prefiks naziva znamenitosti
            }]
        ),

        # ros2 bag record
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', os.path.join(bag_dir, 'bagfile'),
                '/znamenitost',
                '/turtle1/pose'
            ],
            shell=False
        )
    ])
