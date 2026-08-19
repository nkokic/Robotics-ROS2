from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    hsv_lower_arg = DeclareLaunchArgument(
        'hsv_lower',
        default_value='[40, 100, 100]',
        description='HSV lower bounds for green detection [H, S, V]'
    )
    
    hsv_upper_arg = DeclareLaunchArgument(
        'hsv_upper',
        default_value='[80, 255, 255]',
        description='HSV upper bounds for green detection [H, S, V]'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target frame for transformation (usually map or odom)'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='200.0',
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
    
    return LaunchDescription([
        hsv_lower_arg,
        hsv_upper_arg,
        target_frame_arg,
        min_area_arg,
        object_detector_node
    ])
