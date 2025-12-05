from threading import Thread
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from pymoveit2 import MoveIt2

class rob_manip:
    move_group = "arm"
    joint_names = [
        "hip",
        "shoulder",
        "elbow",
        "elbow_2",
        "elbow_3",
        "elbow_4",
        "wrist"
    ]
    base_link_name = "base_link"
    end_effector_name = "hand"

def main():
    rclpy.init()
    # Create node for this example
    node = Node("pose_goal")
    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=rob_manip.joint_names,
        base_link_name=rob_manip.base_link_name,
        end_effector_name=rob_manip.end_effector_name,
        group_name=rob_manip.move_group,
        callback_group=callback_group,
    )

    moveit2.planner_id = "RRTConnectkConfigDefault"

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Parameters
    cartesian = False
    cartesian_max_step = 0.0025
    cartesian_fraction_threshold = 0.0
    cartesian_jump_threshold = 0.0
    cartesian_avoid_collisions = False

    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    def moveToPose(position, rotation):
        # Move to pose
        q = rotation
        for i in range(len(q)):
            q[i] = q[i] / 180 * 3.14
        q = quaternion_from_euler(q[0], q[1], q[2])
        node.get_logger().info("Radians: " + str(rotation))
        quat_xyzw = [q[0], q[1], q[2], q[3]]
        node.get_logger().info(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,)

        moveit2.wait_until_executed()
        print("Path executed.")
        #Start Second move command
        a = input("Press any key to continue.")
    
    #                        x    y    z                x   y   z
    moveToPose(position = [0.38, 0.0, 0.2], rotation = [0, 180, 0])
    moveToPose(position = [0.33, 0.0, 0.3], rotation = [0, 180, 0])
    moveToPose(position = [0, 0.38, 0.2], rotation = [0, 180, 0])
    moveToPose(position = [0, 0.33, 0.3], rotation = [0, 180, 0])

    new_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    moveit2.move_to_configuration(new_joint_states)
    moveit2.wait_until_executed()
    exit(0)

if __name__ == "__main__":
    main()