#!/usr/bin/env python3

import copy
import math
import os

import numpy as np
import rclpy
import tf2_ros

from ament_index_python.packages import get_package_share_directory
from cabinet_generator.cabinet_model import Cabinet, rot_z
from geometry_msgs.msg import Point32, PoseStamped
from moveit.planning import MoveItPy
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.duration import Duration
from rclpy.logging import get_logger
from sensor_msgs.msg import PointCloud
from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
)


# =============================================================================
# Configuration
# =============================================================================

RobotName = "ur5_robotiq_3f"
MoveItPackageName = "ur5_robotiq_moveit_config"

BaseFrame = "base_link"
WorldFrame = "world"
ToolFrame = "tool0"
GripperFrame = "gripper"

WaypointsTopic = "/waypoints"


# =============================================================================
# Cabinet Setup
# =============================================================================

def CreateCabinet(logger):
    """
    Create and configure the cabinet model.

    Returns:
        Configured Cabinet instance.
    """

    doorParameters = np.array(
        [
            0.28,    # width
            0.35,    # height
            0.018,   # thickness
            0.3,     # static thickness
        ]
    )

    # -------------------------------------------------------------------------
    # Cabinet orientation
    # -------------------------------------------------------------------------

    cabinetToWorldTransform = np.eye(4)

    cabinetToWorldTransform[:3, :3] = rot_z(
        np.radians(150)
    )

    cabinetToWorldTransform[:3, 3] = np.array(
        [
            -0.3,
            -0.4,
            0.278,
        ]
    )

    initialDoorAngleDegrees = 20.0

    cabinet = Cabinet(
        doorParameters,
        axis_pos=-1,
        r=np.array(
            [
                0.01,
                -0.5 * doorParameters[0],
            ]
        ),
        T_A_S=cabinetToWorldTransform,
        save_path="./src/lv3/cabinet.urdf",
        has_handle=False,
        initial_angle_deg=initialDoorAngleDegrees,
    )

    logger.info("Cabinet created.")

    return cabinet


# =============================================================================
# MoveIt Setup
# =============================================================================

def CreateMoveItInstance(logger):
    """
    Initialize MoveItPy and return the planning component.

    Returns:
        Tuple containing:
            - MoveItPy instance
            - Arm planning component
    """

    moveItConfig = (
        MoveItConfigsBuilder(
            robot_name=RobotName,
            package_name=MoveItPackageName,
        )
        .robot_description(
            file_path="config/ur5_robotiq_3f.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory(
                    MoveItPackageName
                ),
                "config",
                "py_node_config.yaml",
            )
        )
        .to_moveit_configs()
    ).to_dict()

    moveItConfig.update(
        {
            "use_sim_time": True,
        }
    )

    parametersFile = create_params_file_from_dict(
        moveItConfig,
        "/**",
    )

    moveItInstance = MoveItPy(
        node_name="moveit_py",
        launch_params_filepaths=[
            parametersFile
        ],
    )

    armPlanningComponent = (
        moveItInstance.get_planning_component("arm")
    )

    # Give the planner additional time for each request.
    try:
        armPlanningComponent.set_planning_time(10.0)
    except Exception:
        pass

    logger.info(
        "MoveItPy instance created."
    )

    return (
        moveItInstance,
        armPlanningComponent,
    )


# =============================================================================
# Transformation Helpers
# =============================================================================

def CreateTransformFromPose(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    """
    Create a 4x4 homogeneous transformation matrix.

    Args:
        position: XYZ translation.
        quaternion: XYZW quaternion.

    Returns:
        4x4 homogeneous transformation matrix.
    """

    transform = quaternion_matrix(quaternion)

    transform[0, 3] = position[0]
    transform[1, 3] = position[1]
    transform[2, 3] = position[2]

    return transform


def GetTransformFromTf(
    transformStamped,
) -> np.ndarray:
    """
    Convert a ROS TransformStamped message into a 4x4 matrix.
    """

    translation = transformStamped.transform.translation
    rotation = transformStamped.transform.rotation

    return CreateTransformFromPose(
        np.array(
            [
                translation.x,
                translation.y,
                translation.z,
            ]
        ),
        np.array(
            [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ]
        ),
    )


# =============================================================================
# TF Lookup
# =============================================================================

def WaitForRequiredTransforms(
    node,
    logger,
):
    """
    Wait until the required TF transforms are available.

    Required transforms:
        tool0 <- gripper
        world <- base_link

    Returns:
        Tuple containing:
            - gripper-to-tool transform
            - robot-base-to-world transform
    """

    tfBuffer = tf2_ros.Buffer()

    tfListener = tf2_ros.TransformListener(
        tfBuffer,
        node,
    )

    gripperToToolTransform = None
    robotBaseToWorldTransform = None

    logger.info(
        "Waiting for TF tool0 -> gripper..."
    )

    while (
        gripperToToolTransform is None
        or robotBaseToWorldTransform is None
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

        # ---------------------------------------------------------------------
        # tool0 <- gripper
        # ---------------------------------------------------------------------

        if gripperToToolTransform is None:
            try:
                transform = tfBuffer.lookup_transform(
                    ToolFrame,
                    GripperFrame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )

                gripperToToolTransform = (
                    GetTransformFromTf(transform)
                )

                translation = (
                    transform.transform.translation
                )

                logger.info(
                    "TF tool0 -> gripper obtained: "
                    f"translation=("
                    f"{translation.x:.3f}, "
                    f"{translation.y:.3f}, "
                    f"{translation.z:.3f})"
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exception:

                logger.info(
                    "Waiting for TF tool0 -> gripper... "
                    f"({type(exception).__name__})"
                )

        # ---------------------------------------------------------------------
        # world <- base_link
        # ---------------------------------------------------------------------

        if robotBaseToWorldTransform is None:
            try:
                transform = tfBuffer.lookup_transform(
                    WorldFrame,
                    BaseFrame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )

                robotBaseToWorldTransform = (
                    GetTransformFromTf(transform)
                )

                translation = (
                    transform.transform.translation
                )

                logger.info(
                    "TF world -> base_link obtained: "
                    f"translation=("
                    f"{translation.x:.3f}, "
                    f"{translation.y:.3f}, "
                    f"{translation.z:.3f})"
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exception:

                logger.info(
                    "Waiting for TF world -> base_link... "
                    f"({type(exception).__name__})"
                )

    return (
        gripperToToolTransform,
        robotBaseToWorldTransform,
        tfListener,
    )


# =============================================================================
# Cabinet Waypoint Generation
# =============================================================================

def CreateCabinetCornerTransforms(
    cabinet,
    cabinetPose: np.ndarray,
    initialDoorAngleDegrees: float,
) -> list[np.ndarray]:
    """
    Generate cabinet corner transforms for each door rotation angle.

    Returns:
        List of corner-to-world transformation matrices.
    """

    cabinetToWorldTransform = CreateTransformFromPose(
        cabinetPose[:3],
        cabinetPose[3:],
    )

    axisToCabinetTransform = (
        cabinet.T_A_O_init
    )

    cornerToAxisTransform = (
        cabinet.T_D_A_init
    )

    cornerToWorldTransforms = []

    for doorAngleDegrees in range(
        int(initialDoorAngleDegrees),
        91,
        15,
    ):
        doorRotation = quaternion_from_euler(
            0.0,
            0.0,
            math.radians(-doorAngleDegrees),
        )

        doorRotationTransform = quaternion_matrix(
            doorRotation
        )

        cornerToWorldTransform = (
            cabinetToWorldTransform
            @ axisToCabinetTransform
            @ doorRotationTransform
            @ cornerToAxisTransform
        )

        cornerToWorldTransforms.append(
            cornerToWorldTransform
        )

    return cornerToWorldTransforms


def CreateRobotToolTransforms(
    cornerToWorldTransforms: list[np.ndarray],
    robotBaseToWorldTransform: np.ndarray,
    gripperToToolTransform: np.ndarray,
) -> list[np.ndarray]:
    """
    Convert cabinet corner transforms into robot tool transforms.

    The transformation chain is:

        T_B_T =
            T_B_W *
            T_W_D *
            T_D_G *
            T_G_T
    """

    worldToRobotBaseTransform = np.linalg.inv(
        robotBaseToWorldTransform
    )

    toolToGripperTransform = np.linalg.inv(
        gripperToToolTransform
    )

    # -------------------------------------------------------------------------
    # Corner-to-gripper transform.
    # -------------------------------------------------------------------------

    cornerToGripperTransform = np.array(
        [
            [0, 0, 1, -0.02],
            [0, 1, 0, 0.03],
            [-1, 0, 0, 0.005],
            [0, 0, 0, 1],
        ]
    )

    robotBaseToToolTransforms = []

    for cornerToWorldTransform in cornerToWorldTransforms:

        robotBaseToToolTransform = (
            worldToRobotBaseTransform
            @ cornerToWorldTransform
            @ cornerToGripperTransform
            @ toolToGripperTransform
        )

        robotBaseToToolTransforms.append(
            robotBaseToToolTransform
        )

    return robotBaseToToolTransforms


# =============================================================================
# Pose Creation
# =============================================================================

def CreateWaypoints(
    robotBaseToToolTransforms: list[np.ndarray],
) -> list[PoseStamped]:
    """
    Convert transformation matrices into ROS PoseStamped waypoints.
    """

    waypoints = []

    for transform in robotBaseToToolTransforms:

        waypoint = PoseStamped()

        waypoint.header.frame_id = BaseFrame

        waypoint.pose.position.x = float(
            transform[0, 3]
        )

        waypoint.pose.position.y = float(
            transform[1, 3]
        )

        waypoint.pose.position.z = float(
            transform[2, 3]
        )

        quaternion = quaternion_from_matrix(
            transform
        )

        waypoint.pose.orientation.x = (
            quaternion[0]
        )

        waypoint.pose.orientation.y = (
            quaternion[1]
        )

        waypoint.pose.orientation.z = (
            quaternion[2]
        )

        waypoint.pose.orientation.w = (
            quaternion[3]
        )

        waypoints.append(waypoint)

    # -------------------------------------------------------------------------
    # Add an approach waypoint above the first waypoint.
    # -------------------------------------------------------------------------

    approachWaypoint = copy.deepcopy(
        waypoints[0]
    )

    approachWaypoint.pose.position.z += 0.1

    waypoints.insert(
        0,
        approachWaypoint,
    )

    return waypoints


# =============================================================================
# Waypoint Visualization
# =============================================================================

def CreateWaypointCloud(
    waypoints: list[PoseStamped],
) -> PointCloud:
    """
    Create a PointCloud message containing all waypoints.
    """

    pointCloud = PointCloud()

    pointCloud.header.frame_id = BaseFrame

    pointCloud.points = [
        Point32(
            x=waypoint.pose.position.x,
            y=waypoint.pose.position.y,
            z=waypoint.pose.position.z,
        )
        for waypoint in waypoints
    ]

    return pointCloud


def CreateWaypointPublisher(
    node,
    waypoints: list[PoseStamped],
):
    """
    Create a periodic waypoint PointCloud publisher.
    """

    waypointPublisher = node.create_publisher(
        PointCloud,
        WaypointsTopic,
        10,
    )

    waypointCloud = CreateWaypointCloud(
        waypoints
    )

    def PublishWaypointCloud():
        waypointCloud.header.stamp = (
            node.get_clock().now().to_msg()
        )

        waypointPublisher.publish(
            waypointCloud
        )

    node.create_timer(
        1.0,
        PublishWaypointCloud,
    )

    return waypointPublisher


# =============================================================================
# Motion Planning
# =============================================================================

def ExecuteWaypointTrajectory(
    moveItInstance,
    armPlanningComponent,
    waypoints: list[PoseStamped],
    logger,
):
    """
    Plan and execute each waypoint sequentially.

    If planning fails, a small XYZ offset is applied and
    planning is attempted again.
    """

    for waypointIndex, waypoint in enumerate(
        waypoints
    ):

        logger.info(
            f"Planning waypoint "
            f"{waypointIndex + 1}/{len(waypoints)}"
        )

        armPlanningComponent.set_goal_state(
            pose_stamped_msg=waypoint,
            pose_link=ToolFrame,
        )

        planResult = (
            armPlanningComponent.plan()
        )

        # ---------------------------------------------------------------------
        # Retry with a small positional offset if planning fails.
        # ---------------------------------------------------------------------

        if not planResult:

            logger.warn(
                "Initial planning attempt failed. "
                "Applying 1 mm XYZ offset and retrying."
            )

            waypoint.pose.position.x += 0.001
            waypoint.pose.position.y += 0.001
            waypoint.pose.position.z += 0.001

            armPlanningComponent.set_goal_state(
                pose_stamped_msg=waypoint,
                pose_link=ToolFrame,
            )

            planResult = (
                armPlanningComponent.plan()
            )

        # ---------------------------------------------------------------------
        # Execute successful plan.
        # ---------------------------------------------------------------------

        if planResult:

            logger.info(
                f"Executing waypoint "
                f"{waypointIndex + 1}/{len(waypoints)}"
            )

            robotTrajectory = (
                planResult.trajectory
            )

            moveItInstance.execute(
                robotTrajectory,
                controllers=[],
            )

        else:
            logger.error(
                f"Planning failed for waypoint "
                f"{waypointIndex + 1}."
            )


# =============================================================================
# Main
# =============================================================================

def Main(args=None):
    """
    Application entry point.
    """

    logger = get_logger(
        "cabinet_waypoint_generator"
    )

    # -------------------------------------------------------------------------
    # Cabinet configuration
    # -------------------------------------------------------------------------

    initialDoorAngleDegrees = 20.0

    cabinetPose = np.array(
        [
            -0.2,
            0.4,
            0.948,
            0.0,
            0.0,
            -0.706825,
            0.707388,
        ]
    )

    cabinet = CreateCabinet(
        logger
    )

    # -------------------------------------------------------------------------
    # Initialize ROS 2
    # -------------------------------------------------------------------------

    rclpy.init(args=args)

    # -------------------------------------------------------------------------
    # Initialize MoveIt
    # -------------------------------------------------------------------------

    (
        moveItInstance,
        armPlanningComponent,
    ) = CreateMoveItInstance(
        logger
    )

    # -------------------------------------------------------------------------
    # Create ROS node used for TF and waypoint publishing.
    # -------------------------------------------------------------------------

    waypointsNode = rclpy.create_node(
        "waypoints_publisher"
    )

    # -------------------------------------------------------------------------
    # Obtain required TF transforms.
    # -------------------------------------------------------------------------

    (
        gripperToToolTransform,
        robotBaseToWorldTransform,
        tfListener,
    ) = WaitForRequiredTransforms(
        waypointsNode,
        logger,
    )

    # -------------------------------------------------------------------------
    # Generate cabinet corner transforms.
    # -------------------------------------------------------------------------

    cornerToWorldTransforms = (
        CreateCabinetCornerTransforms(
            cabinet,
            cabinetPose,
            initialDoorAngleDegrees,
        )
    )

    # -------------------------------------------------------------------------
    # Convert cabinet transforms to robot tool transforms.
    # -------------------------------------------------------------------------

    robotBaseToToolTransforms = (
        CreateRobotToolTransforms(
            cornerToWorldTransforms,
            robotBaseToWorldTransform,
            gripperToToolTransform,
        )
    )

    # -------------------------------------------------------------------------
    # Convert transforms into ROS waypoints.
    # -------------------------------------------------------------------------

    waypoints = CreateWaypoints(
        robotBaseToToolTransforms
    )

    # -------------------------------------------------------------------------
    # Start persistent waypoint visualization.
    # -------------------------------------------------------------------------

    CreateWaypointPublisher(
        waypointsNode,
        waypoints,
    )

    # -------------------------------------------------------------------------
    # Execute robot trajectory.
    # -------------------------------------------------------------------------

    ExecuteWaypointTrajectory(
        moveItInstance,
        armPlanningComponent,
        waypoints,
        logger,
    )

    logger.info(
        "Trajectory execution completed. "
        "Waypoint publisher is running. "
        "Press Ctrl+C to exit."
    )

    # -------------------------------------------------------------------------
    # Keep waypoint publisher alive.
    # -------------------------------------------------------------------------

    try:
        rclpy.spin(
            waypointsNode
        )

    except KeyboardInterrupt:
        pass

    finally:
        waypointsNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    Main()