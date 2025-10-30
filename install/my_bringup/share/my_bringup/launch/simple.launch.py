from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    complex_publisher = Node(package="py_pkg", executable="complex_pub")
    complex_subscriber = Node(package="py_pkg", executable="complex_sub")

    ld.add_action(complex_publisher)
    ld.add_action(complex_subscriber)

    return ld