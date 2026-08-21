from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file koji pokreće ArUco detektor i drone kontroler
    """
    
    # ArUco detektor
    aruco_detector_node = Node(
        package='amr_lv5',
        executable='detect_aruco',
        name='aruco_detector',
        output='screen',
    )
    
    # Drone kontroler
    drone_controller_node = Node(
        package='amr_lv5',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
    )
    
    ld = LaunchDescription()
    ld.add_action(aruco_detector_node)
    ld.add_action(drone_controller_node)
    
    return ld
