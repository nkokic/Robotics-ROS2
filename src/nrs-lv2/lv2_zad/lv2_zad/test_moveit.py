import os
import math
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.utils import create_params_file_from_dict
from moveit.core.kinematic_constraints import construct_joint_constraint
# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

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
    arm = mpy.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    # instantiate a RobotState instance using the current robot model
    robot_model = mpy.get_robot_model()
    robot_state = RobotState(robot_model)

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set goal using a pose message this time
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "world"
    pose_goal.pose.position.x = 0.38
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.2
    q = quaternion_from_euler(0.0, 3.14, 0.0)
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    
    arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "hand")

    # plan to goal
    plan_and_execute(mpy, arm, logger)

    #Start Second move command
    a = input("Press any key")

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set constraints message
    joint_values = {
        "hip": 0.0,
        "shoulder": 0.0,
        "elbow": math.pi / 2,
        "elbow_2": 0.0,
        "elbow_3": 0.0,
        "elbow_4": 0.0,
        "wrist": 0.0,
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
    exit(0)

if __name__ == '__main__':
    main()