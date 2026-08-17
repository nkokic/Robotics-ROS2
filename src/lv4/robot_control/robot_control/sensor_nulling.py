#!/usr/bin/env python3
"""
ROS 2 Node for FT Sensor Nulling (Biasing).

This node:
1. Moves the robot to a pre-insertion pose on startup.
2. Performs automatic FT sensor nulling by sampling sensor data.
3. Continuously publishes compensated data to /ft_wrench_biased.
"""

import os
import random
import time

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node


try:
    from moveit.planning import MoveItPy
    from moveit.utils import create_params_file_from_dict
    from moveit_configs_utils import MoveItConfigsBuilder
    from ament_index_python.packages import get_package_share_directory

    IsMoveItAvailable = True
    MoveItImportError = None

except ImportError as exception:
    IsMoveItAvailable = False
    MoveItImportError = str(exception)


class SensorNullingNode(Node):
    """
    ROS 2 node responsible for FT sensor nulling/bias compensation.

    On startup:
    1. Move the robot to the pre-insertion pose.
    2. Wait for the robot to settle.
    3. Collect FT sensor samples.
    4. Calculate the average sensor bias.

    After nulling:
    - Subtract the calculated bias from incoming FT measurements.
    - Publish the compensated wrench on /ft_wrench_biased.
    """

    def __init__(
        self,
        moveItInstance=None,
        moveItArmComponent=None,
    ):
        super().__init__("sensor_nulling_node")

        # =====================================================================
        # Dependencies
        # =====================================================================

        self.moveItInstance = moveItInstance
        self.moveItArmComponent = moveItArmComponent

        self.callbackGroup = ReentrantCallbackGroup()

        # =====================================================================
        # Sensor State
        # =====================================================================

        self.currentWrench: WrenchStamped | None = None

        # =====================================================================
        # Sensor Bias
        # =====================================================================

        self.forceBiasX = 0.0
        self.forceBiasY = 0.0
        self.forceBiasZ = 0.0

        self.torqueBiasX = 0.0
        self.torqueBiasY = 0.0
        self.torqueBiasZ = 0.0

        self.isNullingComplete = False

        # =====================================================================
        # Parameters
        # =====================================================================

        self.DeclareParameters()

        self.numberOfSamples = self.get_parameter(
            "num_samples"
        ).value

        self.sampleInterval = self.get_parameter(
            "sample_interval"
        ).value

        # =====================================================================
        # ROS Communication
        # =====================================================================

        self.forceTorqueSubscription = self.create_subscription(
            WrenchStamped,
            "/ft_wrench",
            self.OnForceTorqueReceived,
            10,
            callback_group=self.callbackGroup,
        )

        self.biasedForceTorquePublisher = self.create_publisher(
            WrenchStamped,
            "/ft_wrench_biased",
            10,
        )

        # =====================================================================
        # Startup
        # =====================================================================

        if self.moveItArmComponent is not None:
            self.get_logger().info(
                "MoveItPy initialized successfully."
            )
        else:
            self.get_logger().warn(
                "MoveItPy not available. "
                "Robot will not move before nulling."
            )

        self.get_logger().info(
            "Sensor Nulling Node initialized."
        )

        self.get_logger().info(
            "Publishing compensated data to /ft_wrench_biased"
        )

        # Delay startup sequence slightly so that the FT subscriber
        # has time to begin receiving sensor data.
        self.startupTimer = self.create_timer(
            1.0,
            self.PerformStartupNulling,
            callback_group=self.callbackGroup,
        )

    # =========================================================================
    # Parameter Setup
    # =========================================================================

    def DeclareParameters(self):
        """Declare all ROS parameters used by this node."""

        # ---------------------------------------------------------------------
        # Nulling parameters
        # ---------------------------------------------------------------------

        self.declare_parameter(
            "num_samples",
            50,
        )

        self.declare_parameter(
            "sample_interval",
            0.02,
        )

        # ---------------------------------------------------------------------
        # Pre-insertion pose
        #
        # These values are expressed in the base_link frame.
        # ---------------------------------------------------------------------

        self.declare_parameter(
            "pre_insertion_x",
            0.0,
        )

        self.declare_parameter(
            "pre_insertion_y",
            0.4,
        )

        self.declare_parameter(
            "pre_insertion_z",
            0.4,
        )

    # =========================================================================
    # Startup Sequence
    # =========================================================================

    def PerformStartupNulling(self):
        """
        Execute the complete startup nulling sequence.

        This method is called once by the startup timer.
        """

        # Ensure this callback only runs once.
        self.startupTimer.cancel()

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "Starting automatic nulling sequence..."
        )
        self.get_logger().info("=" * 50)

        # ---------------------------------------------------------------------
        # Step 1: Move to pre-insertion pose
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "Step 1: Moving to pre-insertion pose..."
        )

        movementSuccessful = (
            self.MoveToPreInsertionPose()
        )

        while not movementSuccessful:
            self.get_logger().warn(
                "Movement failed. Trying again."
            )

            movementSuccessful = (
                self.MoveToPreInsertionPose()
            )

        # ---------------------------------------------------------------------
        # Step 2: Allow the robot to settle
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "Step 2: Waiting for robot to settle (2 seconds)..."
        )

        time.sleep(2.0)

        # ---------------------------------------------------------------------
        # Step 3: Perform sensor nulling
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "Step 3: Performing sensor nulling..."
        )

        nullingSuccessful, resultMessage = (
            self.PerformNulling()
        )

        if nullingSuccessful:
            self.isNullingComplete = True

            self.get_logger().info("=" * 50)
            self.get_logger().info(
                "Nulling complete! "
                "Now publishing compensated data."
            )
            self.get_logger().info("=" * 50)

        else:
            self.get_logger().error(
                f"Nulling failed: {resultMessage}"
            )

    # =========================================================================
    # Force/Torque Sensor
    # =========================================================================

    def OnForceTorqueReceived(
        self,
        message: WrenchStamped,
    ):
        """
        Process incoming raw FT sensor data.

        The raw measurement is stored and a bias-compensated
        measurement is immediately published.
        """

        self.currentWrench = message

        compensatedMessage = WrenchStamped()

        compensatedMessage.header = message.header
        compensatedMessage.header.frame_id = (
            message.header.frame_id
        )

        # ---------------------------------------------------------------------
        # Force compensation
        # ---------------------------------------------------------------------

        compensatedMessage.wrench.force.x = (
            message.wrench.force.x
            - self.forceBiasX
        )

        compensatedMessage.wrench.force.y = (
            message.wrench.force.y
            - self.forceBiasY
        )

        compensatedMessage.wrench.force.z = (
            message.wrench.force.z
            - self.forceBiasZ
        )

        # ---------------------------------------------------------------------
        # Torque compensation
        # ---------------------------------------------------------------------

        compensatedMessage.wrench.torque.x = (
            message.wrench.torque.x
            - self.torqueBiasX
        )

        compensatedMessage.wrench.torque.y = (
            message.wrench.torque.y
            - self.torqueBiasY
        )

        compensatedMessage.wrench.torque.z = (
            message.wrench.torque.z
            - self.torqueBiasZ
        )

        # ---------------------------------------------------------------------
        # Publish compensated measurement
        # ---------------------------------------------------------------------

        self.biasedForceTorquePublisher.publish(
            compensatedMessage
        )

    # =========================================================================
    # Robot Movement
    # =========================================================================

    def MoveToPreInsertionPose(self) -> bool:
        """
        Move the robot to the pre-insertion pose.

        The pose is above the hole with the tool oriented downwards.

        Returns:
            True if the movement succeeds or MoveItPy is unavailable.
            False if planning or execution fails.
        """

        if (
            self.moveItArmComponent is None
            or self.moveItInstance is None
        ):
            self.get_logger().warn(
                "MoveItPy not available. "
                "Skipping robot movement."
            )

            # Continue with nulling even without MoveIt.
            return True

        try:
            # -----------------------------------------------------------------
            # Get target position
            # -----------------------------------------------------------------

            preInsertionX = (
                self.get_parameter(
                    "pre_insertion_x"
                ).value
                / 2.0
                + random.random()
                * self.get_parameter(
                    "pre_insertion_x"
                ).value
                / 2.0
            )

            preInsertionY = (
                self.get_parameter(
                    "pre_insertion_y"
                ).value
                / 2.0
                + random.random()
                * self.get_parameter(
                    "pre_insertion_y"
                ).value
                / 2.0
            )

            preInsertionZ = self.get_parameter(
                "pre_insertion_z"
            ).value

            self.get_logger().info(
                "Moving to pre-insertion pose: "
                f"({preInsertionX}, "
                f"{preInsertionY}, "
                f"{preInsertionZ})"
            )

            # -----------------------------------------------------------------
            # Set current state as planning start state
            # -----------------------------------------------------------------

            self.moveItArmComponent.set_start_state_to_current_state()

            # -----------------------------------------------------------------
            # Create target pose
            # -----------------------------------------------------------------

            targetPose = PoseStamped()

            targetPose.header.frame_id = "base_link"

            targetPose.pose.position.x = float(
                preInsertionX
            )

            targetPose.pose.position.y = float(
                preInsertionY
            )

            targetPose.pose.position.z = float(
                preInsertionZ
            )

            # -----------------------------------------------------------------
            # Set downward-facing orientation
            #
            # Quaternion representing a 180-degree rotation around X.
            # -----------------------------------------------------------------

            targetPose.pose.orientation.x = 1.0
            targetPose.pose.orientation.y = 0.0
            targetPose.pose.orientation.z = 0.0
            targetPose.pose.orientation.w = 0.0

            # -----------------------------------------------------------------
            # Set MoveIt goal
            # -----------------------------------------------------------------

            self.moveItArmComponent.set_goal_state(
                pose_stamped_msg=targetPose,
                pose_link="tool0",
            )

            # -----------------------------------------------------------------
            # Plan
            # -----------------------------------------------------------------

            self.get_logger().info(
                "Planning trajectory..."
            )

            planResult = (
                self.moveItArmComponent.plan()
            )

            if not planResult:
                self.get_logger().warn(
                    "Planning failed for "
                    "pre-insertion pose."
                )
                return False

            # -----------------------------------------------------------------
            # Execute
            # -----------------------------------------------------------------

            self.get_logger().info(
                "Executing plan..."
            )

            robotTrajectory = planResult.trajectory

            self.moveItInstance.execute(
                robotTrajectory,
                controllers=[],
            )

            self.get_logger().info(
                "Successfully moved to "
                "pre-insertion pose."
            )

            return True

        except Exception as exception:
            self.get_logger().error(
                f"Error during movement: {exception}"
            )

            return False

    # =========================================================================
    # Sensor Nulling
    # =========================================================================

    def PerformNulling(self) -> tuple[bool, str]:
        """
        Calculate the FT sensor bias from multiple samples.

        Returns:
            Tuple containing:
                - Success state
                - Result message
        """

        if self.currentWrench is None:
            return (
                False,
                "No FT sensor data received yet. "
                "Cannot perform nulling.",
            )

        self.get_logger().info(
            "Starting nulling process: "
            f"collecting {self.numberOfSamples} samples..."
        )

        # ---------------------------------------------------------------------
        # Sample storage
        # ---------------------------------------------------------------------

        forceXSamples = []
        forceYSamples = []
        forceZSamples = []

        torqueXSamples = []
        torqueYSamples = []
        torqueZSamples = []

        # ---------------------------------------------------------------------
        # Collect samples
        # ---------------------------------------------------------------------

        for _ in range(self.numberOfSamples):

            if self.currentWrench is not None:

                forceXSamples.append(
                    self.currentWrench.wrench.force.x
                )

                forceYSamples.append(
                    self.currentWrench.wrench.force.y
                )

                forceZSamples.append(
                    self.currentWrench.wrench.force.z
                )

                torqueXSamples.append(
                    self.currentWrench.wrench.torque.x
                )

                torqueYSamples.append(
                    self.currentWrench.wrench.torque.y
                )

                torqueZSamples.append(
                    self.currentWrench.wrench.torque.z
                )

            time.sleep(self.sampleInterval)

        # ---------------------------------------------------------------------
        # Validate sample collection
        # ---------------------------------------------------------------------

        if len(forceXSamples) == 0:
            return (
                False,
                "Failed to collect any samples during nulling.",
            )

        # ---------------------------------------------------------------------
        # Calculate average bias
        # ---------------------------------------------------------------------

        self.forceBiasX = np.mean(forceXSamples)
        self.forceBiasY = np.mean(forceYSamples)
        self.forceBiasZ = np.mean(forceZSamples)

        self.torqueBiasX = np.mean(torqueXSamples)
        self.torqueBiasY = np.mean(torqueYSamples)
        self.torqueBiasZ = np.mean(torqueZSamples)

        # ---------------------------------------------------------------------
        # Log calculated bias
        # ---------------------------------------------------------------------

        self.get_logger().info(
            "Nulling complete. New bias values:"
        )

        self.get_logger().info(
            "  Force bias: "
            f"[{self.forceBiasX:.4f}, "
            f"{self.forceBiasY:.4f}, "
            f"{self.forceBiasZ:.4f}]"
        )

        self.get_logger().info(
            "  Torque bias: "
            f"[{self.torqueBiasX:.4f}, "
            f"{self.torqueBiasY:.4f}, "
            f"{self.torqueBiasZ:.4f}]"
        )

        return (
            True,
            "Nulling successful. "
            f"Bias: "
            f"F=[{self.forceBiasX:.3f}, "
            f"{self.forceBiasY:.3f}, "
            f"{self.forceBiasZ:.3f}], "
            f"T=[{self.torqueBiasX:.3f}, "
            f"{self.torqueBiasY:.3f}, "
            f"{self.torqueBiasZ:.3f}]",
        )


# =============================================================================
# MoveIt Initialization
# =============================================================================

def CreateMoveItInstance():
    """
    Create and configure the MoveItPy instance.

    Returns:
        Tuple containing:
            - MoveItPy instance
            - MoveIt planning component
    """

    if not IsMoveItAvailable:
        return None, None

    logger = get_logger("moveit_initialization")

    try:
        robotName = "ur5_robotiq"
        packageName = "ur5_robotiq_moveit_config"

        logger.info(
            "Initializing MoveItPy..."
        )

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
                    get_package_share_directory(
                        packageName
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

        # ---------------------------------------------------------------------
        # Create MoveIt parameter file
        # ---------------------------------------------------------------------

        parametersFile = create_params_file_from_dict(
            moveItConfig,
            "/**",
        )

        # ---------------------------------------------------------------------
        # Create MoveItPy instance
        # ---------------------------------------------------------------------

        moveItInstance = MoveItPy(
            node_name="moveit_py_nulling",
            launch_params_filepaths=[
                parametersFile
            ],
        )

        moveItArmComponent = (
            moveItInstance.get_planning_component(
                "arm"
            )
        )

        # ---------------------------------------------------------------------
        # Configure planning
        # ---------------------------------------------------------------------

        try:
            moveItArmComponent.set_planning_time(
                10.0
            )
        except Exception:
            # Some MoveItPy versions may not expose
            # set_planning_time on the planning component.
            pass

        logger.info(
            "MoveItPy instance created successfully."
        )

        return (
            moveItInstance,
            moveItArmComponent,
        )

    except Exception as exception:
        logger.warn(
            f"Failed to initialize MoveItPy: {exception}"
        )

        return None, None


# =============================================================================
# Application Entry Point
# =============================================================================

def Main(args=None):
    """Initialize ROS 2 and run the sensor nulling node."""

    # -------------------------------------------------------------------------
    # MoveIt initialization
    # -------------------------------------------------------------------------

    moveItInstance = None
    moveItArmComponent = None

    if IsMoveItAvailable:
        (
            moveItInstance,
            moveItArmComponent,
        ) = CreateMoveItInstance()

    else:
        logger = get_logger(
            "sensor_nulling"
        )

        logger.warn(
            "MoveItPy not available: "
            f"{MoveItImportError}"
        )

    # -------------------------------------------------------------------------
    # Initialize ROS 2
    # -------------------------------------------------------------------------

    if not rclpy.ok():
        rclpy.init(args=args)

    # -------------------------------------------------------------------------
    # Create node
    # -------------------------------------------------------------------------

    node = SensorNullingNode(
        moveItInstance=moveItInstance,
        moveItArmComponent=moveItArmComponent,
    )

    # -------------------------------------------------------------------------
    # Run executor
    # -------------------------------------------------------------------------

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
