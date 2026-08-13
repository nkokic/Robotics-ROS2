#!/usr/bin/env python3
"""
ROS 2 Node for Peg-in-Hole Task using Admittance Control.
Refactored for clarity, robustness, and standard coding practices.
"""

import sys
import os
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

# ROS Messages
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose, PoseArray, Point32
from sensor_msgs.msg import PointCloud
import tf2_ros
from tf_transformations import quaternion_matrix, quaternion_from_matrix

# MoveItPy Imports
try:
    from moveit.planning import MoveItPy
    from moveit.utils import create_params_file_from_dict
    from moveit_configs_utils import MoveItConfigsBuilder
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class PegInHoleNode(Node):
    """
    Control node for Peg-in-Hole assembly using Force/Torque feedback 
    and Admittance Control logic.
    """

    def __init__(self, moveit_py_instance=None, moveit_arm_component=None):
        super().__init__('peg_in_hole_node')
        
        self.mpy = moveit_py_instance
        self.arm = moveit_arm_component
        self.cb_group = ReentrantCallbackGroup()

        # --- State Variables ---
        self.current_wrench: WrenchStamped = None
        self.T_tool0_peg_tip = None # Transformation matrix (4x4)
        self.trajectory_poses = []  # For visualization

        # --- Parameters ---
        self._declare_parameters()
        
        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Communication ---
        self.ft_sub = self.create_subscription(
            WrenchStamped, '/ft_wrench_biased', self.ft_callback, 10, 
            callback_group=self.cb_group
        )
        
        self.vis_pose_pub = self.create_publisher(PoseArray, '/peg_in_hole/poses', 10)
        self.vis_cloud_pub = self.create_publisher(PointCloud, '/peg_in_hole/waypoints', 10)

        # --- Startup ---
        if self.arm:
            self.get_logger().info('MoveItPy component linked.')
        else:
            self.get_logger().warn('MoveItPy NOT initialized. Movement will fail.')

        # Start task timer (one-shot)
        self.start_timer = self.create_timer(2.0, self.run_mission, callback_group=self.cb_group)

    def _declare_parameters(self):
        """Declares all ROS parameters."""
        self.declare_parameter('hole_x', 0.0)
        self.declare_parameter('hole_y', 0.0)
        self.declare_parameter('hole_z', 0.81)
        self.declare_parameter('approach_height', 0.05)
        self.declare_parameter('descent_step', 0.002)
        
        # Thresholds & Gains
        self.declare_parameter('force_contact_th', 5.0)  # Newton
        self.declare_parameter('force_safety_limit', 50.0) # Newton
        self.declare_parameter('admittance_gain_xy', 0.00001)

    # =========================================================================
    # SENSOR & TF METHODS
    # =========================================================================

    def ft_callback(self, msg: WrenchStamped):
        """Callback for Force/Torque sensor data."""
        self.current_wrench = msg

    def get_wrench_in_base_frame(self) -> tuple:
        """
        Transforms the force vector from sensor frame to base_link frame.
        Returns: (fx, fy, fz) in base frame.
        """
        if self.current_wrench is None:
            return 0.0, 0.0, 0.0

        try:
            source_frame = self.current_wrench.header.frame_id
            target_frame = 'base_link'

            # Transform logic: Resolve generic frames if needed
            if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
                # Fallback for Gazebo/Sim inconsistencies
                if self.tf_buffer.can_transform(target_frame, 'robotiq_ft_frame_id', rclpy.time.Time()):
                    source_frame = 'robotiq_ft_frame_id'
                else:
                    source_frame = 'tool0'

            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )

            # Extract rotation matrix
            q = transform.transform.rotation
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

            # Current Force Vector
            f_sensor = np.array([
                self.current_wrench.wrench.force.x,
                self.current_wrench.wrench.force.y,
                self.current_wrench.wrench.force.z
            ])

            # $$ F_{base} = R \cdot F_{sensor} $$
            f_base = np.dot(R, f_sensor)
            return f_base[0], f_base[1], f_base[2]

        except Exception as e:
            self.get_logger().debug(f"TF Error in wrench transform: {e}")
            # Return raw data as fallback to prevent crash, but warn
            return (self.current_wrench.wrench.force.x, 
                    self.current_wrench.wrench.force.y, 
                    self.current_wrench.wrench.force.z)

    def init_peg_transform(self) -> bool:
        """Initializes the static transform from tool0 to peg_tip."""
        self.get_logger().info('Looking up TF: tool0 -> peg_tip...')
        for _ in range(10):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'tool0', 'peg_tip', rclpy.time.Time(), timeout=Duration(seconds=1.0)
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                self.T_tool0_peg_tip = quaternion_matrix([q.x, q.y, q.z, q.w])
                self.T_tool0_peg_tip[0:3, 3] = [t.x, t.y, t.z]
                return True
            except Exception:
                time.sleep(0.5)
        return False

    def solve_ik_pose(self, peg_target_world: np.ndarray) -> np.ndarray:
        """
        Calculates the required 'tool0' pose in 'base_link' to place 
        the 'peg_tip' at 'peg_target_world'.
        """
        try:
            # Get World -> Base
            tf_wb = self.tf_buffer.lookup_transform(
                'base_link', 'world', rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            t = tf_wb.transform.translation
            q = tf_wb.transform.rotation
            T_base_world = quaternion_matrix([q.x, q.y, q.z, q.w])
            T_base_world[0:3, 3] = [t.x, t.y, t.z]
        except Exception as e:
            self.get_logger().error(f"Cannot get world->base transform: {e}")
            return None

        # Define Peg Orientation (Downwards in World Frame)
        # Z-axis points down (-1), Y-axis points ? (-1) -> depends on your setup
        T_peg_world = np.eye(4)
        T_peg_world[0:3, 3] = peg_target_world
        # Hardcoded orientation: facing down. Ensure this matches your gripper setup!
        T_peg_world[0, 0] = 1.0
        T_peg_world[1, 1] = -1.0
        T_peg_world[2, 2] = -1.0

        # Chain: T_base_tool0 = T_base_world * T_world_peg * inv(T_tool0_peg)
        # Simplified: T_tool0_base (target)
        # Note: The original math was likely: T_base_tool0 = T_base_world * T_world_peg * T_peg_tool0
        
        T_peg_tool0 = np.linalg.inv(self.T_tool0_peg_tip)
        T_tool0_world = T_peg_world @ T_peg_tool0
        T_tool0_base = T_base_world @ T_tool0_world
        
        return T_tool0_base

    # =========================================================================
    # MOVEMENT & CONTROL LOGIC
    # =========================================================================

    def move_to_matrix(self, T_matrix: np.ndarray, log_msg: str = "") -> bool:
        """Executes a move to a pose defined by a 4x4 homogenous matrix."""
        if not self.arm: return False
        
        try:
            self.arm.set_start_state_to_current_state()
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "base_link"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Translation
            pose_msg.pose.position.x = float(T_matrix[0, 3])
            pose_msg.pose.position.y = float(T_matrix[1, 3])
            pose_msg.pose.position.z = float(T_matrix[2, 3])
            
            # Rotation
            q = quaternion_from_matrix(T_matrix)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            if log_msg: self.get_logger().info(f"Moving: {log_msg}")
            
            self.arm.set_goal_state(pose_stamped_msg=pose_msg, pose_link="tool0")
            plan = self.arm.plan()
            
            if plan:
                self.mpy.execute(plan.trajectory, controllers=[])
                return True
            else:
                self.get_logger().warn(f"Planning failed: {log_msg}")
                return False
        except Exception as e:
            self.get_logger().error(f"Movement exception: {e}")
            return False

    def execute_admittance_insertion(self, start_x, start_y, start_z) -> bool:
        """
        
        Performs insertion using position-based admittance control.
        Robot complies with forces in X/Y while pushing down in Z.
        """
        self.get_logger().info("--- Starting Admittance Insertion ---")
        
        # Parameters
        hole_z = self.get_parameter('hole_z').value
        gain_xy = self.get_parameter('admittance_gain_xy').value
        z_limit = self.get_parameter('force_safety_limit').value
        
        curr_pos = np.array([start_x, start_y, start_z])
        
        # Control Loop Constants
        DESCENT_SPEED = 0.0005  # m/cycle
        MAX_CORRECTION_STEP = 0.003
        MAX_ITERATIONS = 200
        LATERAL_FORCE_OK = 30.0 # N
        
        for i in range(MAX_ITERATIONS):
            # 1. Get Feedback
            fx, fy, fz = self.get_wrench_in_base_frame()
            f_lat = math.sqrt(fx**2 + fy**2)
            
            # 2. Check Success
            if curr_pos[2] <= (hole_z + 0.002):
                if f_lat < LATERAL_FORCE_OK:
                    self.get_logger().info(f"SUCCESS: Depth reached. F_lat={f_lat:.1f}N")
                    return True
                else:
                    self.get_logger().warn(f"Jammed at depth? F_lat={f_lat:.1f}N. Adjusting...")

            # 3. Compute Admittance (Force -> Displacement)
            # D = K * F
            dx = np.clip(gain_xy * fx, -MAX_CORRECTION_STEP, MAX_CORRECTION_STEP)
            dy = np.clip(gain_xy * fy, -MAX_CORRECTION_STEP, MAX_CORRECTION_STEP)
            
            # 4. Compute Descent (Z-axis)
            # Stop pushing if resistance is too high
            if fz > z_limit or f_lat > 50.0:
                self.get_logger().warn(f"High Resistance! Fz={fz:.1f}, Flat={f_lat:.1f}")
                dz = 0.0
            else:
                dz = -DESCENT_SPEED

            # 5. Update Target
            curr_pos += [dx, dy, dz]
            
            self.get_logger().info(
                f"Iter {i}: Z={curr_pos[2]:.4f} | F=({fx:.1f}, {fy:.1f}, {fz:.1f}) | dXY=({dx*1000:.1f}mm, {dy*1000:.1f}mm)"
            )

            # 6. Execute Move
            T_target = self.solve_ik_pose(curr_pos)
            if T_target is not None:
                self.move_to_matrix(T_target) # Short/fast moves, logging suppressed
            
            # 7. Visualize
            self.record_trajectory_point(curr_pos)

        self.get_logger().error("Insertion Timed Out (Max Iterations)")
        return False

    def run_mission(self):
        """Main State Machine."""
        self.start_timer.cancel() # Run once
        
        # Pre-checks
        if not self.init_peg_transform():
            self.get_logger().error("Could not determine Peg TF. Aborting.")
            return

        self.get_logger().info("Waiting for valid FT data...")
        while self.current_wrench is None:
            time.sleep(0.1)

        # Retrieve Params
        h_x = self.get_parameter('hole_x').value
        h_y = self.get_parameter('hole_y').value
        h_z = self.get_parameter('hole_z').value
        approach_h = self.get_parameter('approach_height').value
        contact_th = self.get_parameter('force_contact_th').value
        descent_step = self.get_parameter('descent_step').value

        # --- PHASE 1: Approach ---
        self.get_logger().info("PHASE 1: Approach")
        target_pos = np.array([h_x, h_y, h_z + approach_h])
        T_app = self.solve_ik_pose(target_pos)
        if not self.move_to_matrix(T_app, "Approach Position"):
            return
        time.sleep(1.0)

        # --- PHASE 2: Guarded Descent ---
        self.get_logger().info("PHASE 2: Guarded Descent (Search for Contact)")
        current_z = h_z + approach_h
        contact_detected = False
        
        while current_z > (h_z - 0.01):
            # Check force magnitude
            f_mag = np.linalg.norm(self.get_wrench_in_base_frame())
            
            if f_mag > contact_th:
                self.get_logger().info(f"CONTACT DETECTED: |F|={f_mag:.2f} N")
                contact_detected = True
                break
            
            current_z -= descent_step
            T_desc = self.solve_ik_pose(np.array([h_x, h_y, current_z]))
            self.move_to_matrix(T_desc)
            time.sleep(0.05) # Allow sensor to stabilize slightly

        if not contact_detected:
            self.get_logger().warn("Reached bottom without contact detection!")
        
        # --- PHASE 3: Insertion ---
        self.trajectory_poses.clear() # Reset visualization
        success = self.execute_admittance_insertion(h_x, h_y, current_z)
        
        self.publish_visualization()
        
        if success:
            self.get_logger().info("MISSION COMPLETE: Peg Inserted.")
        else:
            self.get_logger().error("MISSION FAILED.")

    # =========================================================================
    # VISUALIZATION HELPER
    # =========================================================================
    
    def record_trajectory_point(self, pos: np.ndarray):
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        self.trajectory_poses.append(p)

    def publish_visualization(self):
        if not self.trajectory_poses: return
        
        # Pose Array
        pa = PoseArray()
        pa.header.frame_id = "world"
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = self.trajectory_poses
        self.vis_pose_pub.publish(pa)
        
        # Point Cloud
        pc = PointCloud()
        pc.header.frame_id = "world"
        pc.header.stamp = pa.header.stamp
        for p in self.trajectory_poses:
            pt = Point32()
            pt.x, pt.y, pt.z = p.position.x, p.position.y, p.position.z
            pc.points.append(pt)
        self.vis_cloud_pub.publish(pc)


def get_moveit_instance():
    """Initializes MoveItPy configuration strictly."""
    if not MOVEIT_AVAILABLE: return None, None
    
    try:
        robot_name = "ur5_robotiq"
        pkg_name = "ur5_robotiq_moveit_config"
        
        # Build Config
        moveit_config = (
            MoveItConfigsBuilder(robot_name=robot_name, package_name=pkg_name)
            .robot_description(file_path="config/ur5_robotiq.urdf.xacro")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .moveit_cpp(
                file_path=os.path.join(
                    get_package_share_directory(pkg_name),
                    "config", "py_node_config.yaml"
                )
            )
            .to_moveit_configs()
        ).to_dict()
        
        moveit_config.update({"use_sim_time": True})
        
        # Initialize
        params_file = create_params_file_from_dict(moveit_config, "/**")
        mpy = MoveItPy(node_name="moveit_py_peg", launch_params_filepaths=[params_file])
        arm = mpy.get_planning_component("arm")
        
     
        
        return mpy, arm
    except Exception as e:
        print(f"FAILED TO INIT MOVEIT: {e}")
        return None, None


def main(args=None):
    rclpy.init(args=args)
    
    # Init MoveIt separately to keep main clean
    mpy, arm = get_moveit_instance()
    
    node = PegInHoleNode(moveit_py_instance=mpy, moveit_arm_component=arm)
    
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