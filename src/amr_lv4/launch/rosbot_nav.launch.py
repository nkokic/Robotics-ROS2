#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    rosbot_gazebo_dir = get_package_share_directory('rosbot_gazebo')
    rosbot3_nav_dir = get_package_share_directory('rosbot3_nav')
    
    # Path to map file
    map_file = os.path.join(rosbot3_nav_dir, 'maps', 'map.yaml')
    
    # Debug: print map file path
    print(f"=== Map file path: {map_file} ===")
    print(f"=== Map file exists: {os.path.exists(map_file)} ===")
    
    # Launch Gazebo simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_gazebo_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'robot_model': 'rosbot',
            'rviz': 'false',  # Disable RViz from simulation, navigation will start its own
        }.items()
    )
    
    # Launch navigation with longer delay to allow simulation to fully initialize
    navigation_launch = TimerAction(
        period=20.0,  # Wait 20 seconds for simulation to fully initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rosbot3_nav_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        simulation_launch,
        navigation_launch,
    ])
