from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments for patrol navigator
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='patrol_points.yaml',
        description='Input file path containing patrol points'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='1.0',
        description='Safe distance from detected object in meters'
    )
    
    wait_time_arg = DeclareLaunchArgument(
        'wait_time',
        default_value='3.0',
        description='Time to wait at object location in seconds'
    )
    
    # Declare launch arguments for object detector
    hsv_lower_arg = DeclareLaunchArgument(
        'hsv_lower',
        default_value='[80, 100, 100]',
        description='HSV lower bounds for cyan detection [H, S, V]'
    )
    
    hsv_upper_arg = DeclareLaunchArgument(
        'hsv_upper',
        default_value='[100, 255, 255]',
        description='HSV upper bounds for cyan detection [H, S, V]'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target frame for transformation (usually map or odom)'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='1000.0',
        description='Minimum contour area to consider as valid object'
    )
    
    # Create object detector node
    object_detector_node = Node(
        package='amr_lv3',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[{
            'hsv_lower': LaunchConfiguration('hsv_lower'),
            'hsv_upper': LaunchConfiguration('hsv_upper'),
            'target_frame': LaunchConfiguration('target_frame'),
            'min_area': LaunchConfiguration('min_area')
        }]
    )
    
    # Create patrol navigator node
    patrolling_point_navigator_node = Node(
        package='amr_lv3',
        executable='patrolling_point_navigator',
        output='screen',
        parameters=[{
            'input_file': LaunchConfiguration('input_file'),
            'safe_distance': LaunchConfiguration('safe_distance'),
            'wait_time': LaunchConfiguration('wait_time')
        }]
    )
    
    return LaunchDescription([
        # ROS_LOCALHOST_ONLY is deprecated in Jazzy and overrides
        # ROS_AUTOMATIC_DISCOVERY_RANGE when inherited from the shell. Keep
        # these nodes in the same discovery scope as the TurtleBot bringup.
        UnsetEnvironmentVariable('ROS_LOCALHOST_ONLY'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        # Arguments
        input_file_arg,
        safe_distance_arg,
        wait_time_arg,
        hsv_lower_arg,
        hsv_upper_arg,
        target_frame_arg,
        min_area_arg,
        # Nodes
        object_detector_node,
        patrolling_point_navigator_node
    ])
