#!/usr/bin/env python3
"""
ROS 2 Node for FT Sensor Nulling (Biasing).

This node:
1. On startup, moves robot to pre-insertion pose (above hole, oriented downwards)
2. Performs automatic nulling by sampling FT sensor data
3. Continuously publishes compensated data to /ft_wrench_biased
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
import random
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_srvs.srv import Trigger

import numpy as np
import time

# MoveItPy imports (ROS 2 native MoveIt API)
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    from moveit.utils import create_params_file_from_dict
    from moveit_configs_utils import MoveItConfigsBuilder
    from ament_index_python.packages import get_package_share_directory
    MOVEIT_AVAILABLE = True
except ImportError as e:
    MOVEIT_AVAILABLE = False
    _MOVEIT_IMPORT_ERROR = str(e)


class SensorNullingNode(Node):
    """
    Node that performs FT sensor nulling/biasing.
    
    On startup: moves robot to position and performs nulling.
    Then continuously publishes compensated (nulled) data.
    """

    def __init__(self, mpy=None, arm=None):
        super().__init__('sensor_nulling_node')
        
        # MoveItPy instance (passed from main)
        self.mpy = mpy
        self.arm = arm
        
        # Callback group for allowing concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Bias values (initialized to zero)
        self.bias_force_x = 0.0
        self.bias_force_y = 0.0
        self.bias_force_z = 0.0
        self.bias_torque_x = 0.0
        self.bias_torque_y = 0.0
        self.bias_torque_z = 0.0
        
        # Flag to track if nulling is complete
        self.nulling_complete = False
        
        # Current sensor readings (updated by subscriber)
        self.current_wrench = None
        
        # Nulling parameters
        self.declare_parameter('num_samples', 50)
        self.declare_parameter('sample_interval', 0.02)  # 20ms between samples
        self.num_samples = self.get_parameter('num_samples').value
        self.sample_interval = self.get_parameter('sample_interval').value
        
        # Pre-insertion pose parameters in base_link frame
        # Hole is at (0, 0, 0.8) in world frame
        # base_link is at (0, -0.7, 0.8) in world frame
        # So hole in base_link frame is at (0, 0.7, 0)
        # peg_tip is ~25cm below tool0 when pointing down
        # So tool0 should be at z=0.35 to have peg_tip ~10cm above table
        self.declare_parameter('pre_insertion_x', 0.0)
        self.declare_parameter('pre_insertion_y', 0.4)  # Hole Y position in base_link
        self.declare_parameter('pre_insertion_z', 0.4)  # Above hole, accounting for peg length
        
        # Subscriber to raw FT sensor data
        self.ft_subscription = self.create_subscription(
            WrenchStamped,
            '/ft_wrench',
            self.ft_wrench_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for biased (compensated) FT data
        self.ft_biased_publisher = self.create_publisher(
            WrenchStamped,
            '/ft_wrench_biased',
            10
        )
        
        if self.arm is not None:
            self.get_logger().info('MoveItPy initialized successfully.')
        else:
            self.get_logger().warn(
                'MoveItPy not available. Robot will not move before nulling.'
            )
        
        self.get_logger().info('Sensor Nulling Node initialized.')
        self.get_logger().info('Publishing compensated data to /ft_wrench_biased')
        
        # Create a one-shot timer to perform initial nulling after node starts
        # This allows the subscriber to start receiving data first
        self.startup_timer = self.create_timer(
            1.0,  # Wait 1 second for sensor data to arrive
            self.perform_startup_nulling,
            callback_group=self.callback_group
        )

    def perform_startup_nulling(self):
        """
        Perform nulling sequence on startup (called once by timer).
        """
        # Cancel the timer so this only runs once
        self.startup_timer.cancel()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting automatic nulling sequence...')
        self.get_logger().info('=' * 50)
        
        # Step 1: Move to pre-insertion pose
        self.get_logger().info('Step 1: Moving to pre-insertion pose...')
        move_success = self.move_to_pre_insertion_pose()
        
        while not move_success:
            self.get_logger().warn(
                'Movement failed, trying again.'
            )
            move_success = self.move_to_pre_insertion_pose()
        
        # Step 2: Wait for robot to settle
        self.get_logger().info('Step 2: Waiting for robot to settle (2 seconds)...')
        time.sleep(2.0)
        
        # Step 3: Perform nulling
        self.get_logger().info('Step 3: Performing sensor nulling...')
        success, message = self.perform_nulling()
        
        if success:
            self.nulling_complete = True
            self.get_logger().info('=' * 50)
            self.get_logger().info('Nulling complete! Now publishing compensated data.')
            self.get_logger().info('=' * 50)
        else:
            self.get_logger().error(f'Nulling failed: {message}')

    def ft_wrench_callback(self, msg: WrenchStamped):
        """
        Callback for incoming FT sensor data.
        Stores current reading and publishes compensated data.
        """
        self.current_wrench = msg
        
        # Create compensated message
        biased_msg = WrenchStamped()
        biased_msg.header = msg.header
        biased_msg.header.frame_id = msg.header.frame_id
        
        # Apply bias compensation (subtract bias from raw values)
        biased_msg.wrench.force.x = msg.wrench.force.x - self.bias_force_x
        biased_msg.wrench.force.y = msg.wrench.force.y - self.bias_force_y
        biased_msg.wrench.force.z = msg.wrench.force.z - self.bias_force_z
        biased_msg.wrench.torque.x = msg.wrench.torque.x - self.bias_torque_x
        biased_msg.wrench.torque.y = msg.wrench.torque.y - self.bias_torque_y
        biased_msg.wrench.torque.z = msg.wrench.torque.z - self.bias_torque_z
        
        # Publish compensated data
        self.ft_biased_publisher.publish(biased_msg)

    def move_to_pre_insertion_pose(self) -> bool:
        """
        Move robot to pre-insertion pose (above hole, oriented downwards).
        Uses MoveItPy API for planning and execution.
        
        Returns:
            bool: True if movement successful, False otherwise.
        """
        if self.arm is None or self.mpy is None:
            self.get_logger().warn('MoveItPy not available, skipping robot movement.')
            return True  # Return True to continue with nulling anyway
        
        try:
            # Get parameters for pre-insertion position
            x = self.get_parameter('pre_insertion_x').value/2+random.random()*self.get_parameter('pre_insertion_x').value/2 
            y = self.get_parameter('pre_insertion_y').value/2+random.random()*self.get_parameter('pre_insertion_y').value/2 
            z = self.get_parameter('pre_insertion_z').value
            
            self.get_logger().info(f'Moving to pre-insertion pose: ({x}, {y}, {z})')
            
            # Set plan start state to current state
            self.arm.set_start_state_to_current_state()
            
            # Create target pose (PoseStamped for MoveItPy)
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            pose_goal.pose.position.x = float(x)
            pose_goal.pose.position.y = float(y)
            pose_goal.pose.position.z = float(z)
            
            # Orientation pointing downwards (-Z direction)
            # Quaternion for 180° rotation around X-axis (pointing tool down)
            pose_goal.pose.orientation.x = 1.0
            pose_goal.pose.orientation.y = 0.0
            pose_goal.pose.orientation.z = 0.0
            pose_goal.pose.orientation.w = 0.0
            
            # Set goal state using MoveItPy API
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
            
            # Plan trajectory
            self.get_logger().info('Planning trajectory...')
            plan_result = self.arm.plan()
            
            if plan_result:
                # Execute the planned trajectory
                self.get_logger().info('Executing plan...')
                robot_trajectory = plan_result.trajectory
                self.mpy.execute(robot_trajectory, controllers=[])
                self.get_logger().info('Successfully moved to pre-insertion pose.')
                return True
            else:
                self.get_logger().warn('Planning failed for pre-insertion pose.')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Error during movement: {e}')
            return False

    def perform_nulling(self) -> tuple:
        """
        Perform the nulling process by sampling sensor data and computing bias.
        
        Returns:
            tuple: (success: bool, message: str)
        """
        if self.current_wrench is None:
            return False, 'No FT sensor data received yet. Cannot perform nulling.'
        
        self.get_logger().info(
            f'Starting nulling process: collecting {self.num_samples} samples...'
        )
        
        # Collect samples
        force_x_samples = []
        force_y_samples = []
        force_z_samples = []
        torque_x_samples = []
        torque_y_samples = []
        torque_z_samples = []
        
        for i in range(self.num_samples):
            if self.current_wrench is not None:
                force_x_samples.append(self.current_wrench.wrench.force.x)
                force_y_samples.append(self.current_wrench.wrench.force.y)
                force_z_samples.append(self.current_wrench.wrench.force.z)
                torque_x_samples.append(self.current_wrench.wrench.torque.x)
                torque_y_samples.append(self.current_wrench.wrench.torque.y)
                torque_z_samples.append(self.current_wrench.wrench.torque.z)
            
            time.sleep(self.sample_interval)
        
        if len(force_x_samples) == 0:
            return False, 'Failed to collect any samples during nulling.'
        
        # Calculate average bias values
        self.bias_force_x = np.mean(force_x_samples)
        self.bias_force_y = np.mean(force_y_samples)
        self.bias_force_z = np.mean(force_z_samples)
        self.bias_torque_x = np.mean(torque_x_samples)
        self.bias_torque_y = np.mean(torque_y_samples)
        self.bias_torque_z = np.mean(torque_z_samples)
        
        self.get_logger().info('Nulling complete. New bias values:')
        self.get_logger().info(
            f'  Force bias:  [{self.bias_force_x:.4f}, {self.bias_force_y:.4f}, {self.bias_force_z:.4f}]'
        )
        self.get_logger().info(
            f'  Torque bias: [{self.bias_torque_x:.4f}, {self.bias_torque_y:.4f}, {self.bias_torque_z:.4f}]'
        )
        
        return True, (
            f'Nulling successful. Bias: F=[{self.bias_force_x:.3f}, {self.bias_force_y:.3f}, '
            f'{self.bias_force_z:.3f}], T=[{self.bias_torque_x:.3f}, {self.bias_torque_y:.3f}, '
            f'{self.bias_torque_z:.3f}]'
        )


def main(args=None):
    """Main entry point for the sensor nulling node."""
    logger = get_logger("sensor_nulling")
    
    # Initialize MoveItPy before rclpy.init (it handles its own initialization)
    mpy = None
    arm = None
    
    if MOVEIT_AVAILABLE:
        try:
            logger.info("Initializing MoveItPy...")
            
            # Build MoveIt configuration (adjust package_name and robot_name as needed)
            moveit_config = (
                MoveItConfigsBuilder(
                    robot_name="ur5_robotiq", 
                    package_name="ur5_robotiq_moveit_config"
                )
                .robot_description(file_path="config/ur5_robotiq.urdf.xacro")
                .trajectory_execution(file_path="config/moveit_controllers.yaml")
                .moveit_cpp(
                    file_path=os.path.join(
                        get_package_share_directory("ur5_robotiq_moveit_config"),
                        "config",
                        "py_node_config.yaml",
                    )
                )
                .to_moveit_configs()
            ).to_dict()
            
            moveit_config.update({"use_sim_time": True})
            file = create_params_file_from_dict(moveit_config, "/**")
            
            # Instantiate MoveItPy instance
            mpy = MoveItPy(node_name="moveit_py_nulling", launch_params_filepaths=[file])
            arm = mpy.get_planning_component("arm")
            
            # Configure planning parameters
            try:
                arm.set_planning_time(10.0)
            except Exception:
                pass
            
            logger.info("MoveItPy instance created successfully.")
            
        except Exception as e:
            logger.warn(f"Failed to initialize MoveItPy: {e}")
            mpy = None
            arm = None
    else:
        logger.warn(f"MoveItPy not available: {_MOVEIT_IMPORT_ERROR}")
    
    # Initialize rclpy if not already initialized by MoveItPy
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = SensorNullingNode(mpy=mpy, arm=arm)
    
    # Use multi-threaded executor for concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
