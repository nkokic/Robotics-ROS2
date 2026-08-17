#!/usr/bin/env python3
"""
ROS 2 Node for Peg-in-Hole Task using Admittance Control.

The implementation uses:
- MoveItPy for robot motion planning
- TF2 for coordinate transformations
- Force/Torque feedback for contact detection
- Position-based admittance control for insertion
"""

import math
import os
import time

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Pose, PoseArray, PoseStamped, WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
import tf2_ros
from tf_transformations import quaternion_from_matrix, quaternion_matrix


try:
    from moveit.planning import MoveItPy
    from moveit.utils import create_params_file_from_dict
    from moveit_configs_utils import MoveItConfigsBuilder

    IsMoveItAvailable = True
except ImportError:
    IsMoveItAvailable = False


class PegInHoleNode(Node):
    """
    ROS 2 control node for Peg-in-Hole assembly.

    Uses Force/Torque feedback and admittance control to guide
    a peg into a hole while allowing lateral compliance.
    """

    def __init__(
        self,
        moveItInstance=None,
        moveItArmComponent=None,
    ):
        super().__init__("peg_in_hole_node")

        # ---------------------------------------------------------------------
        # Dependencies
        # ---------------------------------------------------------------------

        self._moveItInstance = moveItInstance
        self._moveItArmComponent = moveItArmComponent
        self._callbackGroup = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # State
        # ---------------------------------------------------------------------

        self._currentWrench: WrenchStamped | None = None

        # Transformation from tool0 to peg_tip.
        self._tool0ToPegTipTransform: np.ndarray | None = None

        # Recorded trajectory points used for visualization.
        self._trajectoryPoses: list[Pose] = []

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------

        self._DeclareParameters()

        # ---------------------------------------------------------------------
        # TF
        # ---------------------------------------------------------------------

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(
            self._tfBuffer,
            self,
        )

        # ---------------------------------------------------------------------
        # ROS Communication
        # ---------------------------------------------------------------------

        self._forceTorqueSubscription = self.create_subscription(
            WrenchStamped,
            "/ft_wrench_biased",
            self._OnForceTorqueReceived,
            10,
            callback_group=self._callbackGroup,
        )

        self._trajectoryPosePublisher = self.create_publisher(
            PoseArray,
            "/peg_in_hole/poses",
            10,
        )

        self._trajectoryPointCloudPublisher = self.create_publisher(
            PointCloud,
            "/peg_in_hole/waypoints",
            10,
        )

        # ---------------------------------------------------------------------
        # Startup
        # ---------------------------------------------------------------------

        if self._moveItArmComponent:
            self.get_logger().info("MoveItPy component linked.")
        else:
            self.get_logger().warn(
                "MoveItPy is not initialized. Robot movement will fail."
            )

        # Start the insertion task once after startup.
        self._startTimer = self.create_timer(
            2.0,
            self._RunInsertion,
            callback_group=self._callbackGroup,
        )

    # =========================================================================
    # Parameter Setup
    # =========================================================================

    def _DeclareParameters(self):
        """Declare all ROS parameters used by the node."""

        self.declare_parameter("hole_x", 0.0)
        self.declare_parameter("hole_y", 0.0)
        self.declare_parameter("hole_z", 0.81)

        self.declare_parameter("approach_height", 0.05)
        self.declare_parameter("descent_step", 0.002)

        # Force thresholds and control gains.
        self.declare_parameter("force_contact_th", 5.0)
        self.declare_parameter("force_safety_limit", 50.0)
        self.declare_parameter("admittance_gain_xy", 0.00001)

    # =========================================================================
    # Sensor and TF
    # =========================================================================

    def _OnForceTorqueReceived(self, message: WrenchStamped):
        """
        Store the latest Force/Torque sensor measurement.
        """
        self._currentWrench = message

    def _GetWrenchInBaseFrame(self) -> tuple[float, float, float]:
        """
        Transform the force vector from the sensor frame into base_link.

        Returns:
            Tuple containing (force_x, force_y, force_z) in base_link.
        """

        if self._currentWrench is None:
            return 0.0, 0.0, 0.0

        try:
            sourceFrame = self._currentWrench.header.frame_id
            targetFrame = "base_link"

            # Resolve generic/simulation-specific frame names if necessary.
            if not self._tfBuffer.can_transform(
                targetFrame,
                sourceFrame,
                rclpy.time.Time(),
            ):
                if self._tfBuffer.can_transform(
                    targetFrame,
                    "robotiq_ft_frame_id",
                    rclpy.time.Time(),
                ):
                    sourceFrame = "robotiq_ft_frame_id"
                else:
                    sourceFrame = "tool0"

            transform = self._tfBuffer.lookup_transform(
                targetFrame,
                sourceFrame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )

            # Extract rotation matrix from quaternion.
            rotationQuaternion = transform.transform.rotation

            rotationMatrix = quaternion_matrix(
                [
                    rotationQuaternion.x,
                    rotationQuaternion.y,
                    rotationQuaternion.z,
                    rotationQuaternion.w,
                ]
            )[:3, :3]

            # Force vector in sensor coordinates.
            sensorForce = np.array(
                [
                    self._currentWrench.wrench.force.x,
                    self._currentWrench.wrench.force.y,
                    self._currentWrench.wrench.force.z,
                ]
            )

            # F_base = R * F_sensor
            baseForce = np.dot(rotationMatrix, sensorForce)

            return (
                baseForce[0],
                baseForce[1],
                baseForce[2],
            )

        except Exception as exception:
            self.get_logger().debug(
                f"TF error while transforming wrench: {exception}"
            )

            # Return raw sensor data as a fallback.
            return (
                self._currentWrench.wrench.force.x,
                self._currentWrench.wrench.force.y,
                self._currentWrench.wrench.force.z,
            )

    def _InitializePegTransform(self) -> bool:
        """
        Initialize the static transformation from tool0 to peg_tip.
        """

        self.get_logger().info(
            "Looking up TF: tool0 -> peg_tip..."
        )

        for _ in range(10):
            try:
                transform = self._tfBuffer.lookup_transform(
                    "tool0",
                    "peg_tip",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )

                translation = transform.transform.translation
                rotation = transform.transform.rotation

                self._tool0ToPegTipTransform = quaternion_matrix(
                    [
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    ]
                )

                self._tool0ToPegTipTransform[0:3, 3] = [
                    translation.x,
                    translation.y,
                    translation.z,
                ]

                return True

            except Exception:
                time.sleep(0.5)

        return False

    def _SolveIkPose(
        self,
        pegTargetWorld: np.ndarray,
    ) -> np.ndarray | None:
        """
        Calculate the required tool0 pose in base_link so that peg_tip
        reaches the requested world position.

        Args:
            pegTargetWorld: Target XYZ position of the peg tip in world frame.

        Returns:
            4x4 homogeneous transformation matrix for tool0.
        """

        if self._tool0ToPegTipTransform is None:
            self.get_logger().error(
                "Peg transform has not been initialized."
            )
            return None

        try:
            # Get world -> base_link transform.
            worldToBaseTransform = self._tfBuffer.lookup_transform(
                "base_link",
                "world",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )

            translation = worldToBaseTransform.transform.translation
            rotation = worldToBaseTransform.transform.rotation

            baseToWorldTransform = quaternion_matrix(
                [
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ]
            )

            baseToWorldTransform[0:3, 3] = [
                translation.x,
                translation.y,
                translation.z,
            ]

        except Exception as exception:
            self.get_logger().error(
                f"Cannot get world->base transform: {exception}"
            )
            return None

        # ---------------------------------------------------------------------
        # Define peg orientation.
        #
        # The peg points downwards in the world frame.
        # This orientation must match the actual gripper/tool configuration.
        # ---------------------------------------------------------------------

        pegToWorldTransform = np.eye(4)

        pegToWorldTransform[0:3, 3] = pegTargetWorld

        pegToWorldTransform[0, 0] = 1.0
        pegToWorldTransform[1, 1] = -1.0
        pegToWorldTransform[2, 2] = -1.0

        # ---------------------------------------------------------------------
        # Transformation chain:
        #
        # T_base_tool0 =
        #     T_base_world *
        #     T_world_peg *
        #     T_peg_tool0
        # ---------------------------------------------------------------------

        pegToTool0Transform = np.linalg.inv(
            self._tool0ToPegTipTransform
        )

        tool0ToWorldTransform = (
            pegToWorldTransform @ pegToTool0Transform
        )

        tool0ToBaseTransform = (
            baseToWorldTransform @ tool0ToWorldTransform
        )

        return tool0ToBaseTransform

    # =========================================================================
    # Movement
    # =========================================================================

    def _MoveToMatrix(
        self,
        targetTransform: np.ndarray,
        logMessage: str = "",
    ) -> bool:
        """
        Plan and execute a move to the supplied homogeneous transformation.
        """

        if not self._moveItArmComponent:
            return False

        try:
            self._moveItArmComponent.set_start_state_to_current_state()

            poseMessage = PoseStamped()
            poseMessage.header.frame_id = "base_link"
            poseMessage.header.stamp = self.get_clock().now().to_msg()

            # Translation.
            poseMessage.pose.position.x = float(targetTransform[0, 3])
            poseMessage.pose.position.y = float(targetTransform[1, 3])
            poseMessage.pose.position.z = float(targetTransform[2, 3])

            # Rotation.
            quaternion = quaternion_from_matrix(targetTransform)

            poseMessage.pose.orientation.x = quaternion[0]
            poseMessage.pose.orientation.y = quaternion[1]
            poseMessage.pose.orientation.z = quaternion[2]
            poseMessage.pose.orientation.w = quaternion[3]

            if logMessage:
                self.get_logger().info(
                    f"Moving: {logMessage}"
                )

            self._moveItArmComponent.set_goal_state(
                pose_stamped_msg=poseMessage,
                pose_link="tool0",
            )

            plan = self._moveItArmComponent.plan()

            if plan:
                self._moveItInstance.execute(
                    plan.trajectory,
                    controllers=[],
                )
                return True

            self.get_logger().warn(
                f"Planning failed: {logMessage}"
            )
            return False

        except Exception as exception:
            self.get_logger().error(
                f"Movement exception: {exception}"
            )
            return False

    # =========================================================================
    # Admittance Control
    # =========================================================================

    def _ExecuteAdmittanceInsertion(
        self,
        startX: float,
        startY: float,
        startZ: float,
    ) -> bool:
        """
        Perform insertion using position-based admittance control.

        The robot:
        - Complies with lateral forces in X/Y.
        - Continues pushing down in Z.
        - Stops descending when excessive force is detected.
        """

        self.get_logger().info(
            "--- Starting Admittance Insertion ---"
        )

        holeZ = self.get_parameter("hole_z").value
        admittanceGainXY = self.get_parameter(
            "admittance_gain_xy"
        ).value
        forceSafetyLimit = self.get_parameter(
            "force_safety_limit"
        ).value

        currentPosition = np.array(
            [
                startX,
                startY,
                startZ,
            ]
        )

        # ---------------------------------------------------------------------
        # Control constants
        # ---------------------------------------------------------------------

        DescentSpeed = 0.0005
        MaxCorrectionStep = 0.003
        MaxIterations = 200
        MaximumLateralForce = 30.0

        for iteration in range(MaxIterations):

            # -----------------------------------------------------------------
            # 1. Read force feedback
            # -----------------------------------------------------------------

            forceX, forceY, forceZ = self._GetWrenchInBaseFrame()

            lateralForce = math.sqrt(
                forceX**2 + forceY**2
            )

            # -----------------------------------------------------------------
            # 2. Check whether insertion depth has been reached
            # -----------------------------------------------------------------

            if currentPosition[2] <= holeZ + 0.002:

                if lateralForce < MaximumLateralForce:
                    self.get_logger().info(
                        f"SUCCESS: Depth reached. "
                        f"F_lat={lateralForce:.1f}N"
                    )
                    return True

                self.get_logger().warn(
                    f"Jammed at depth? "
                    f"F_lat={lateralForce:.1f}N. Adjusting..."
                )

            # -----------------------------------------------------------------
            # 3. Calculate lateral admittance correction
            #
            # D = K * F
            # -----------------------------------------------------------------

            correctionX = np.clip(
                admittanceGainXY * forceX,
                -MaxCorrectionStep,
                MaxCorrectionStep,
            )

            correctionY = np.clip(
                admittanceGainXY * forceY,
                -MaxCorrectionStep,
                MaxCorrectionStep,
            )

            # -----------------------------------------------------------------
            # 4. Calculate Z descent
            # -----------------------------------------------------------------

            if (
                forceZ > forceSafetyLimit
                or lateralForce > 50.0
            ):
                self.get_logger().warn(
                    f"High resistance! "
                    f"Fz={forceZ:.1f}, "
                    f"Flat={lateralForce:.1f}"
                )

                descentStep = 0.0

            else:
                descentStep = -DescentSpeed

            # -----------------------------------------------------------------
            # 5. Update target position
            # -----------------------------------------------------------------

            currentPosition += [
                correctionX,
                correctionY,
                descentStep,
            ]

            self.get_logger().info(
                f"Iteration {iteration}: "
                f"Z={currentPosition[2]:.4f} | "
                f"F=({forceX:.1f}, {forceY:.1f}, {forceZ:.1f}) | "
                f"dXY=("
                f"{correctionX * 1000:.1f}mm, "
                f"{correctionY * 1000:.1f}mm)"
            )

            # -----------------------------------------------------------------
            # 6. Calculate and execute target pose
            # -----------------------------------------------------------------

            targetTransform = self._SolveIkPose(
                currentPosition
            )

            if targetTransform is not None:
                # Short/fast moves, so suppress additional movement logging.
                self._MoveToMatrix(targetTransform)

            # -----------------------------------------------------------------
            # 7. Record trajectory
            # -----------------------------------------------------------------

            self._RecordTrajectoryPoint(currentPosition)

        self.get_logger().error(
            "Insertion timed out (maximum iterations reached)."
        )

        return False

    # =========================================================================
    # Main State Machine
    # =========================================================================

    def _RunInsertion(self):
        """Execute the complete peg insertion state machine."""

        # This timer is intended to execute only once.
        self._startTimer.cancel()

        # ---------------------------------------------------------------------
        # Pre-checks
        # ---------------------------------------------------------------------

        if not self._InitializePegTransform():
            self.get_logger().error(
                "Could not determine Peg TF. Aborting."
            )
            return

        self.get_logger().info(
            "Waiting for valid FT data..."
        )

        while self._currentWrench is None:
            time.sleep(0.1)

        # ---------------------------------------------------------------------
        # Retrieve parameters
        # ---------------------------------------------------------------------

        holeX = self.get_parameter("hole_x").value
        holeY = self.get_parameter("hole_y").value
        holeZ = self.get_parameter("hole_z").value

        approachHeight = self.get_parameter(
            "approach_height"
        ).value

        contactForceThreshold = self.get_parameter(
            "force_contact_th"
        ).value

        descentStep = self.get_parameter(
            "descent_step"
        ).value

        # ---------------------------------------------------------------------
        # Phase 1: Approach
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "PHASE 1: Approach"
        )

        approachPosition = np.array(
            [
                holeX,
                holeY,
                holeZ + approachHeight,
            ]
        )

        approachTransform = self._SolveIkPose(
            approachPosition
        )

        if approachTransform is None:
            self.get_logger().error(
                "Could not calculate approach pose."
            )
            return

        if not self._MoveToMatrix(
            approachTransform,
            "Approach Position",
        ):
            return

        time.sleep(1.0)

        # ---------------------------------------------------------------------
        # Phase 2: Guarded Descent
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "PHASE 2: Guarded Descent (Search for Contact)"
        )

        currentZ = holeZ + approachHeight
        contactDetected = False

        while currentZ > holeZ - 0.01:

            # Check total force magnitude.
            forceMagnitude = np.linalg.norm(
                self._GetWrenchInBaseFrame()
            )

            if forceMagnitude > contactForceThreshold:
                self.get_logger().info(
                    f"Contact detected: "
                    f"|F|={forceMagnitude:.2f} N"
                )

                contactDetected = True
                break

            currentZ -= descentStep

            descentPosition = np.array(
                [
                    holeX,
                    holeY,
                    currentZ,
                ]
            )

            descentTransform = self._SolveIkPose(
                descentPosition
            )

            if descentTransform is None:
                self.get_logger().error(
                    "Could not calculate descent pose."
                )
                return

            self._MoveToMatrix(descentTransform)

            # Allow the force sensor to stabilize.
            time.sleep(0.05)

        if not contactDetected:
            self.get_logger().warn(
                "Reached bottom without contact detection!"
            )

        # ---------------------------------------------------------------------
        # Phase 3: Admittance Insertion
        # ---------------------------------------------------------------------

        self._trajectoryPoses.clear()

        insertionSuccessful = self._ExecuteAdmittanceInsertion(
            holeX,
            holeY,
            currentZ,
        )

        self._PublishVisualization()

        if insertionSuccessful:
            self.get_logger().info(
                "PEG INSERTION COMPLETE"
            )
        else:
            self.get_logger().error(
                "PEG INSERTION FAILED"
            )

    # =========================================================================
    # Visualization
    # =========================================================================

    def _RecordTrajectoryPoint(
        self,
        position: np.ndarray,
    ):
        """Add a position to the recorded trajectory."""

        pose = Pose()

        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        self._trajectoryPoses.append(pose)

    def _PublishVisualization(self):
        """Publish the recorded insertion trajectory."""

        if not self._trajectoryPoses:
            return

        # ---------------------------------------------------------------------
        # PoseArray
        # ---------------------------------------------------------------------

        poseArray = PoseArray()

        poseArray.header.frame_id = "world"
        poseArray.header.stamp = (
            self.get_clock().now().to_msg()
        )

        poseArray.poses = self._trajectoryPoses

        self._trajectoryPosePublisher.publish(
            poseArray
        )

        # ---------------------------------------------------------------------
        # PointCloud
        # ---------------------------------------------------------------------

        pointCloud = PointCloud()

        pointCloud.header.frame_id = "world"
        pointCloud.header.stamp = poseArray.header.stamp

        for pose in self._trajectoryPoses:
            point = Point32()

            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z

            pointCloud.points.append(point)

        self._trajectoryPointCloudPublisher.publish(
            pointCloud
        )


# =============================================================================
# MoveIt Initialization
# =============================================================================

def CreateMoveItInstance():
    """
    Initialize the MoveItPy configuration.

    Returns:
        Tuple containing:
            - MoveItPy instance
            - MoveIt planning component
    """

    if not IsMoveItAvailable:
        return None, None

    try:
        robotName = "ur5_robotiq"
        packageName = "ur5_robotiq_moveit_config"

        # ---------------------------------------------------------------------
        # Build MoveIt configuration
        # ---------------------------------------------------------------------

        moveItConfig = (
            MoveItConfigsBuilder(
                robot_name=robotName,
                package_name=packageName,
            )
            .robot_description(
                file_path="config/ur5_robotiq.urdf.xacro"
            )
            .trajectory_execution(
                file_path="config/moveit_controllers.yaml"
            )
            .moveit_cpp(
                file_path=os.path.join(
                    get_package_share_directory(packageName),
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

        # ---------------------------------------------------------------------
        # Initialize MoveItPy
        # ---------------------------------------------------------------------

        parametersFile = create_params_file_from_dict(
            moveItConfig,
            "/**",
        )

        moveItInstance = MoveItPy(
            node_name="moveit_py_peg",
            launch_params_filepaths=[parametersFile],
        )

        moveItArmComponent = (
            moveItInstance.get_planning_component("arm")
        )

        return (
            moveItInstance,
            moveItArmComponent,
        )

    except Exception as exception:
        print(
            f"FAILED TO INIT MOVEIT: {exception}"
        )

        return None, None


# =============================================================================
# Application Entry Point
# =============================================================================

def Main(args=None):
    """ROS 2 application entry point."""

    rclpy.init(args=args)

    # Keep MoveIt initialization separate from the ROS node setup.
    moveItInstance, moveItArmComponent = CreateMoveItInstance()

    node = PegInHoleNode(
        moveItInstance=moveItInstance,
        moveItArmComponent=moveItArmComponent,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    Main()