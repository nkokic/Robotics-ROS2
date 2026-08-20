#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    rosbotGazeboDirectory = get_package_share_directory('rosbot_gazebo')
    rosbot3NavigationDirectory = get_package_share_directory('rosbot3_nav')
    
    # Path to map file
    mapFile = os.path.join(rosbot3NavigationDirectory, 'maps', 'map.yaml')
    
    # Debug: print map file path
    print(f"=== Map file path: {mapFile} ===")
    print(f"=== Map file exists: {os.path.exists(mapFile)} ===")
    
    # Launch Gazebo simulation
    simulationLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbotGazeboDirectory, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'robot_model': 'rosbot',
            'rviz': 'false',  # Disable RViz from simulation, navigation will start its own
            'x': '0.0',
            'y': '0.0',
            'yaw': '0.0',
        }.items()
    )
    
    # Launch navigation with longer delay to allow simulation to fully initialize
    navigationLaunch = TimerAction(
        period=20.0,  # Wait 20 seconds for simulation to fully initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rosbot3NavigationDirectory, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'map': mapFile,
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        simulationLaunch,
        navigationLaunch,
    ])
