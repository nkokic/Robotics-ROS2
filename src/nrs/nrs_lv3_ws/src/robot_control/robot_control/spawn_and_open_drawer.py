from cabinet_generator.cabinet_model import Cabinet, rot_z
from cabinet_generator.generate_and_spawn_cabinet import CabinetSpawner
import numpy as np
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped, Pose, Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf2_ros
from rclpy.duration import Duration
# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.utils import create_params_file_from_dict
from moveit.core.kinematic_constraints import construct_joint_constraint
# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import copy
import time




def main(args=None):
    logger_spawn = get_logger("spawn_logger")
    door_params = np.array([0.28, 0.35, 0.018, 0.3])

    Tz = np.eye(4)
    Tz[:3,:3] = rot_z(np.radians(150))
    T_A_S = np.eye(4)
    T_A_S = Tz
    T_A_S[:3,3] = np.array([-0.3, -0.4, 0.278])
    initial_angle_deg = 20.
    cabinet_model = Cabinet(door_params, 
                            axis_pos=-1,
                            r=np.array([0.01, -0.5*door_params[0]]),
                            T_A_S=T_A_S, 
                            save_path='./src/lv3/cabinet.urdf',
                            has_handle=False,
                            initial_angle_deg=initial_angle_deg
                            )
    logger_spawn.info("Cabinet created")
    rclpy.init(args=args)

    initial_pose = np.array([-0.2, 0.4, 0.948, 0., 0., -0.706825, 0.707388])

    moveit_config = (
    MoveItConfigsBuilder(robot_name="ur5_robotiq_3f", package_name="ur5_robotiq_moveit_config")
        .robot_description(file_path="config/ur5_robotiq_3f.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path=os.path.join(get_package_share_directory("ur5_robotiq_moveit_config"),
                        "config",
                        "py_node_config.yaml",)).to_moveit_configs()).to_dict()
    moveit_config.update({"use_sim_time": True})
    file = create_params_file_from_dict(moveit_config, "/**")
    # MoveItPy Setup
    logger = get_logger("moveit_py.pose_goal")
    # instantiate MoveItPy instance and get planning component
    mpy = MoveItPy(node_name="moveit_py", launch_params_filepaths=[file])
    arm = mpy.get_planning_component("arm")
    logger.info("MoveItPy instance created")
    # instantiate a RobotState instance using the current robot model
    robot_model = mpy.get_robot_model()
    robot_state = RobotState(robot_model)
    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # Build transforms using proper matrix multiplication (not element-wise '*')
    T_O_W = quaternion_matrix(initial_pose[3:])
    T_O_W[0, 3] = initial_pose[0]
    T_O_W[1, 3] = initial_pose[1]
    T_O_W[2, 3] = initial_pose[2]
    T_A_O = cabinet_model.T_A_O_init
    T_D_A = cabinet_model.T_D_A_init

    waypoints_node = rclpy.create_node('waypoints_publisher')
    waypoints_pub = waypoints_node.create_publisher(PointCloud, '/waypoints', 10)

    # TF2 listener to look up transform between tool0 and gripper
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, waypoints_node)
    
    # Wait for the TF to become available (spin while waiting)
    T_T_G = None
    T_W_B = None
    logger.info("Waiting for TF tool0 -> gripper...")
    while T_T_G is None and T_W_B is None:
        rclpy.spin_once(waypoints_node, timeout_sec=0.1)
        try:
            transform = tf_buffer.lookup_transform('tool0', 'gripper', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            # Convert TransformStamped to 4x4 matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            T_T_G = quaternion_matrix([q.x, q.y, q.z, q.w])
            T_T_G[0, 3] = t.x
            T_T_G[1, 3] = t.y
            T_T_G[2, 3] = t.z
            logger.info(f"TF tool0 -> gripper obtained: translation=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            logger.info(f"Waiting for TF... ({type(e).__name__})")
            continue

        logger.info("Waiting for TF world <- base_link...")

        try:
            transform = tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            # Convert TransformStamped to 4x4 matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            T_W_B = quaternion_matrix([q.x, q.y, q.z, q.w])
            T_W_B[0, 3] = t.x
            T_W_B[1, 3] = t.y
            T_W_B[2, 3] = t.z
            logger.info(f"TF world -> base_link obtained: translation=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            logger.info(f"Waiting for TF... ({type(e).__name__})")
            continue

    T_D_W = []
    for i in range(int(initial_angle_deg), 91, 10):
        rotation = quaternion_from_euler(0, 0, math.radians(-i))
        matrix = quaternion_matrix(rotation)
        # Compose transforms with '@' for matrix multiply to maintain a valid rotation
        T_D_W_i = T_O_W @ T_A_O @ matrix @ T_D_A
        T_D_W.append(T_D_W_i)

    # Build target tool poses in base_link frame: T_T_B = T_B_W @ T_D_W @ T_G_D @ T_T_G
    # We need the inverse of T_W_B to go from world to base_link
    T_B_W = np.linalg.inv(T_W_B)
    T_G_T = np.linalg.inv(T_T_G)
    
    T_T_B_list = []
    T_G_D = np.array([
        [0, 0, 1, -0.02],
        [0, 1, 0, 0.03],
        [-1, 0, 0, 0.005],
        [0, 0, 0, 1],
    ])


    for T_D_W_i in T_D_W:
        T_T_B_i = T_B_W @ T_D_W_i @ T_G_D @ T_G_T
        T_T_B_list.append(T_T_B_i)
    

    waypoints = []
    for point in T_T_B_list:
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = float(point[0, 3])
        pose_goal.pose.position.y = float(point[1, 3])
        pose_goal.pose.position.z = float(point[2, 3])
        quaternion = quaternion_from_matrix(point)
        pose_goal.pose.orientation.x = quaternion[0]
        pose_goal.pose.orientation.y = quaternion[1]
        pose_goal.pose.orientation.z = quaternion[2]
        pose_goal.pose.orientation.w = quaternion[3]
        waypoints.append(pose_goal)

    approach_point = copy.deepcopy(waypoints[0])
    approach_point.pose.position.z += 0.1
    waypoints.insert(0, approach_point)

    # Persistent waypoints publisher as a PointCloud on '/waypoints'
    cloud_msg = PointCloud()
    cloud_msg.header.frame_id = "base_link"
    cloud_msg.points = [
        Point32(x=wp.pose.position.x, y=wp.pose.position.y, z=wp.pose.position.z)
        for wp in waypoints
    ]

    
    # Publish periodically so late subscribers can receive messages
    def _publish_cloud():
        cloud_msg.header.stamp = waypoints_node.get_clock().now().to_msg()
        waypoints_pub.publish(cloud_msg)

    waypoints_node.create_timer(1.0, _publish_cloud)

    # Use the correct tip link for the 'arm' group as defined in SRDF (tip_link="tool0")
    # Also give the planner a bit more time per request.
    try:
        arm.set_planning_time(10.0)
    except Exception:
        pass

    for pose in waypoints:
        arm.set_goal_state(pose_stamped_msg = pose, pose_link = "tool0")
        logger.info("Planning trajectory")
        plan_result = arm.plan()
        if not plan_result:
            pose.pose.position.x +=0.001
            pose.pose.position.y +=0.001
            pose.pose.position.z +=0.001
            arm.set_goal_state(pose_stamped_msg = pose, pose_link = "tool0")
            logger.info("Planning trajectory")
            plan_result = arm.plan()
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            mpy.execute(robot_trajectory, controllers=[])
        else:
            logger.error("Planning failed")

    logger.info("Trajectory execution completed. Waypoints publisher running. Press Ctrl+C to exit.")

    try:
        rclpy.spin(waypoints_node)
    except KeyboardInterrupt:
        pass
    finally:
        waypoints_node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()