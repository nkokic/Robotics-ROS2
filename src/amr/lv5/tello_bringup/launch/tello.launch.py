
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # # Tello driver node Old
        # Node(
        #     package='tello',
        #     executable='tello_node_1',
        #     output='screen',
        #     namespace='/',
        #     name='tello',
        #     parameters=[
        #         {'connect_timeout': 10.0},
        #         {'tello_ip': '192.168.10.1'},
        #         {'tf_base': 'map'},
        #         {'tf_drone': 'drone'}
        #     ],
        #     respawn=True
        # ),

        # Tello driver node
        Node(
            package='tello',
            executable='tello_node',
            output='screen',
            namespace='/',
            name='tello_node',
            parameters=[
                {'connect_timeout': 15.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'odom'},
                {'tf_drone': 'drone'}
            ],
            respawn=True
        ),

        # # Tello control node
        # Node(
        #     package='tello_control',
        #     executable='tello_control',
        #     namespace='/',
        #     name='control',
        #     output='screen',
        #     respawn=False
        # ),

        # RViz data visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace='/',
            name='rviz2',
            respawn=True,
            arguments=['-d', '/home/ros2_tello_ws/src/tello_bringup/config/rviz.rviz']
        ),

        # # Static TF publisher
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     namespace='/',
        #     name='tf',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
        #     respawn=True
        # ),
    ]


    return LaunchDescription(nodes)