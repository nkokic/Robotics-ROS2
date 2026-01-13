#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------- Paths ----------------

    robot_description_pkg = FindPackageShare('robot_description')
    bringup_pkg = FindPackageShare('robot_bringup')

    xacro_file = PathJoinSubstitution([
        robot_description_pkg,
        'urdf',
        'bob_ros_urdf.xacro'
    ])

    world_file = PathJoinSubstitution([
        bringup_pkg,
        'worlds',
        'depot.sdf'
    ])

    bridge_config = PathJoinSubstitution([
        bringup_pkg,
        'config',
        'gz_bridge.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        bringup_pkg,
        'config',
        'navigation.rviz'
    ])

    # ---------------- Robot Description ----------------

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    # ---------------- Gazebo ----------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )

    # ---------------- Robot State Publisher ----------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ---------------- Spawn Robot ----------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bob',
            '-x', '-8.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # ---------------- Gazebo ↔ ROS Bridge ----------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }]
    )

    navigation = Node(
        package='bob_robot_navigation',
        executable='navigation_server'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'lidar_link',
            'lidar_frame'
        ],
        output='screen'
    )


    # ---------------- Launch Description ----------------

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        navigation,
        rviz,
        lidar_static_tf
    ])
