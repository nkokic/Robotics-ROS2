import os
import math
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
# moveit_py
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit.utils import create_params_file_from_dict
from moveit.core.kinematic_constraints import construct_joint_constraint
# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import numpy as np

def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")

def MoveIK(mpy, arm, logger, position, orientation_angles):
    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set goal using a pose message this time
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "world"
    pose_goal.pose.position.x = position[0]
    pose_goal.pose.position.y = position[1]
    pose_goal.pose.position.z = position[2]

    orientation = orientation_angles
    for i in range(len(orientation)):
        orientation[i] = orientation[i] / 180.0 * math.pi

    q = quaternion_from_euler(orientation[0], orientation[1], orientation[2])

    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    
    arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "hand")

    # plan to goal
    plan_and_execute(mpy, arm, logger)

def InterpolatedMoveIK(mpy, arm, logger, start_pos, end_pos, start_ang, end_ang, steps=10):
      pos_list = np.linspace(start_pos, end_pos, num=steps)
      ang_list = np.linspace(start_ang, end_ang, num=steps)

      for i in range(steps):
            MoveIK(
                mpy,
                arm,
                logger,
                position=pos_list[i],
                orientation_angles=ang_list[i]
            )

def MoveFK(mpy, arm, logger, joint_angles):
    # set plan start state to current state
    arm.set_start_state_to_current_state()

    joint_radians = joint_angles

    for i in range(len(joint_radians)):
        joint_angles[i] = joint_angles[i] / 180.0 * math.pi

    # set constraints message
    # instantiate a RobotState instance using the current robot model
    robot_model = mpy.get_robot_model()
    robot_state = RobotState(robot_model)

    joint_values = {
        "hip": joint_radians[0],
        "shoulder": joint_radians[1],
        "elbow": joint_radians[2],
        "elbow_2": joint_radians[3],
        "elbow_3": joint_radians[4],
        "elbow_4": joint_radians[5],
        "wrist": joint_radians[6],
    }
    robot_state.joint_positions = joint_values

    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot_model.get_joint_model_group("arm"),
    )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = arm.plan()
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        mpy.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")


def main():
    #Define Node configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="bob_manip", package_name="rob_manip_moveit")
            .robot_description(file_path="config/bob_manip.urdf.xacro")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .moveit_cpp(file_path=os.path.join(get_package_share_directory("rob_manip_moveit"),
            "config",
            "py_node_config.yaml",)).to_moveit_configs()).to_dict()
    
    moveit_config.update({"use_sim_time": True})
    file = create_params_file_from_dict(moveit_config, "/**")

    # MoveItPy Setup
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    mpy = MoveItPy(node_name="moveit_py", launch_params_filepaths=[file])
    arm : PlanningComponent = mpy.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    MoveIK(    # Dummy IK movement to catch errors
        mpy,
        arm,
        logger,
        position=[0.35, 0.0, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(    # Dummy IK movement to catch errors
        mpy,
        arm,
        logger,
        position=[0.35, 0.0, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK( # Root
        mpy,
        arm,
        logger,
        position=[0.2, 0.2, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.3, 0.3, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK( # Root
        mpy,
        arm,
        logger,
        position=[0.2, 0.2, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.3, 0.2, 0.3],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK( # Root
        mpy,
        arm,
        logger,
        position=[0.2, 0.2, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.2, 0.3, 0.3],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.3, 0.3, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.3, 0.2, 0.3],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.2, 0.3, 0.3],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK( # Root
        mpy,
        arm,
        logger,
        position=[0.2, 0.2, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK(
        mpy,
        arm,
        logger,
        position=[0.3, 0.2, 0.3],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveIK( # Root
        mpy,
        arm,
        logger,
        position=[0.2, 0.2, 0.2],
        orientation_angles=[0.0, 180.0, 0.0]
    )

    MoveFK(
        mpy,
        arm,
        logger,
        joint_angles=[0.0, 0.0, 90.0, 0.0, 90.0, 0.0, 0.0]
    )
    exit(0)

if __name__ == '__main__':
    main()