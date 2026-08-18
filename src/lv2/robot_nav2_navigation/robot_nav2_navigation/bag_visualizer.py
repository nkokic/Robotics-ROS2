#!/usr/bin/env python3

"""
ROS 2 Bag Visualizer.

Loads a ROS 2 bag containing robot position and goal messages,
loads a warehouse map from a YAML/PGM pair, and creates a visualization
showing:

    - The warehouse map
    - The robot's actual trajectory
    - Goal waypoints recorded in the ROS 2 bag

The visualizer supports the following ROS 2 topics:

    /robot_position
        geometry_msgs/Pose
        Preferred source for the robot trajectory.

    /odom
        nav_msgs/Odometry
        Fallback source when /robot_position contains no messages.

    /goal_position
        geometry_msgs/Pose
        Source of the goal waypoints.
"""

import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message


class BagVisualizer:
    """
    Visualizes robot navigation data from a ROS 2 bag.

    The class is responsible for loading the warehouse map, extracting
    robot trajectory and waypoint data from the bag, and generating
    a visualization image.
    """

    Position = tuple[float, float]

    POSE_TOPIC = '/robot_position'
    ODOM_TOPIC = '/odom'
    GOAL_TOPIC = '/goal_position'

    def __init__(self, bagPath: str | Path, mapYamlPath: str | Path) -> None:
        """
        Initialize the bag visualizer.

        Args:
            bagPath: Path to the ROS 2 bag directory.
            mapYamlPath: Path to the map YAML file.
        """
        self.bagPath = Path(bagPath)
        self.mapYamlPath = Path(mapYamlPath)

        self.trajectory: list[BagVisualizer.Position] = []
        self.waypoints: list[BagVisualizer.Position] = []

        self.mapImage: np.ndarray | None = None
        self.mapInfo: dict = {}

    def loadMap(self) -> None:
        """Load the warehouse map from its YAML and PGM files."""
        print(f'Loading map from {self.mapYamlPath}...')

        self.mapInfo = self._loadMapConfiguration()

        mapImagePath = self._getMapImagePath()

        self.mapImage = cv2.imread(
            str(mapImagePath),
            cv2.IMREAD_GRAYSCALE
        )

        if self.mapImage is None:
            raise ValueError(
                f'Failed to load map image: {mapImagePath}'
            )

        # Invert the map:
        # black = obstacle
        # white = free space
        self.mapImage = 255 - self.mapImage

        print(f'Map loaded: {self.mapImage.shape}')
        print(
            f"Resolution: {self.mapInfo['resolution']} m/pixel"
        )
        print(f"Origin: {self.mapInfo['origin']}")

    def _loadMapConfiguration(self) -> dict:
        """Load and return the map configuration from the YAML file."""
        if not self.mapYamlPath.exists():
            raise FileNotFoundError(
                f'Map YAML file not found: {self.mapYamlPath}'
            )

        with self.mapYamlPath.open('r', encoding='utf-8') as file:
            mapInfo = yaml.safe_load(file)

        if not mapInfo:
            raise ValueError(
                f'Map YAML file is empty: {self.mapYamlPath}'
            )

        requiredFields = ('image', 'resolution', 'origin')

        for fieldName in requiredFields:
            if fieldName not in mapInfo:
                raise KeyError(
                    f"Required map field '{fieldName}' "
                    f'was not found in {self.mapYamlPath}'
                )

        return mapInfo

    def _getMapImagePath(self) -> Path:
        """Return the absolute path to the map image."""
        imagePath = Path(self.mapInfo['image'])

        if not imagePath.is_absolute():
            imagePath = self.mapYamlPath.parent / imagePath

        if not imagePath.exists():
            raise FileNotFoundError(
                f'Map image not found: {imagePath}'
            )

        return imagePath

    def extractTrajectoryFromBag(self) -> None:
        """
        Extract robot trajectory and goal waypoints from the ROS 2 bag.

        The /robot_position topic is preferred for the robot trajectory.
        If no /robot_position messages are found, /odom is used as a fallback.

        Goal waypoints are read from /goal_position and duplicate
        positions are removed.
        """
        print(f'📦 Reading bag: {self.bagPath}...')

        reader = self._createBagReader()
        topicTypes = self._getTopicTypes(reader)

        print(f'  Available topics: {list(topicTypes.keys())}')

        poseMessageType = self._getMessageType(
            topicTypes,
            self.POSE_TOPIC
        )

        odometryMessageType = self._getMessageType(
            topicTypes,
            self.ODOM_TOPIC
        )

        goalMessageType = self._getMessageType(
            topicTypes,
            self.GOAL_TOPIC
        )

        pozicijaCount = 0
        odometryCount = 0
        goalCount = 0

        seenWaypoints: set[BagVisualizer.Position] = set()

        while reader.has_next():
            topicName, serializedData, _ = reader.read_next()

            if topicName == self.POSE_TOPIC and poseMessageType:
                self._addPoseToTrajectory(
                    serializedData,
                    poseMessageType
                )
                pozicijaCount += 1

            elif topicName == self.GOAL_TOPIC and goalMessageType:
                self._addWaypoint(
                    serializedData,
                    goalMessageType,
                    seenWaypoints
                )
                goalCount += 1

            elif (
                topicName == self.ODOM_TOPIC
                and odometryMessageType
            ):
                # Keep reading /odom because we only know whether
                # /robot_position exists after processing the bag.
                odometryCount += 1

        # Use /odom only when /robot_position was not available.
        if pozicijaCount == 0 and odometryMessageType:
            self._extractOdometryTrajectory(
                self._createBagReader(),
                odometryMessageType
            )

        print(
            f'✓ Loaded {pozicijaCount} robot positions '
            f'(/robot_position)'
        )

        if pozicijaCount == 0:
            print(
                f'✓ Fallback: Loaded {len(self.trajectory)} '
                f'positions from /odom'
            )

        print(
            f'✓ Loaded {len(self.waypoints)} unique waypoints '
            f'(/goal_position from {goalCount} messages)'
        )

        print(
            f'Total: {len(self.trajectory)} trajectory points, '
            f'{len(self.waypoints)} waypoints'
        )

    def _createBagReader(self) -> SequentialReader:
        """Create and open a ROS 2 bag reader."""
        if not self.bagPath.exists():
            raise FileNotFoundError(
                f'ROS 2 bag not found: {self.bagPath}'
            )

        storageOptions = StorageOptions(
            uri=str(self.bagPath),
            storage_id='mcap'
        )

        converterOptions = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(
            storageOptions,
            converterOptions
        )

        return reader

    @staticmethod
    def _getTopicTypes(
        reader: SequentialReader
    ) -> dict[str, str]:
        """Return a mapping between topic names and message types."""
        topicTypes = reader.get_all_topics_and_types()

        return {
            topic.name: topic.type
            for topic in topicTypes
        }

    @staticmethod
    def _getMessageType(
        topicTypes: dict[str, str],
        topicName: str
    ):
        """Resolve a ROS message type for a topic."""
        if topicName not in topicTypes:
            return None

        return get_message(topicTypes[topicName])

    def _addPoseToTrajectory(
        self,
        serializedData,
        messageType
    ) -> None:
        """Deserialize a Pose message and add it to the trajectory."""
        message = deserialize_message(
            serializedData,
            messageType
        )

        self.trajectory.append(
            (
                message.position.x,
                message.position.y
            )
        )

    def _addWaypoint(
        self,
        serializedData,
        messageType,
        seenWaypoints: set[Position]
    ) -> None:
        """
        Deserialize a goal message and add a unique waypoint.

        Coordinates are rounded to three decimal places before
        duplicate detection.
        """
        message = deserialize_message(
            serializedData,
            messageType
        )

        waypoint = (
            round(message.position.x, 3),
            round(message.position.y, 3)
        )

        if waypoint in seenWaypoints:
            return

        self.waypoints.append(waypoint)
        seenWaypoints.add(waypoint)

    def _extractOdometryTrajectory(
        self,
        reader: SequentialReader,
        messageType
    ) -> None:
        """Extract the robot trajectory from /odom."""
        while reader.has_next():
            topicName, serializedData, _ = reader.read_next()

            if topicName != self.ODOM_TOPIC:
                continue

            message = deserialize_message(
                serializedData,
                messageType
            )

            self.trajectory.append(
                (
                    message.pose.pose.position.x,
                    message.pose.pose.position.y
                )
            )

    def worldToMap(
        self,
        x: float,
        y: float
    ) -> tuple[int, int]:
        """
        Convert world coordinates to map pixel coordinates.

        Args:
            x: World X coordinate in meters.
            y: World Y coordinate in meters.

        Returns:
            A tuple containing the map pixel X and Y coordinates.
        """
        if self.mapImage is None:
            raise RuntimeError(
                'Map must be loaded before converting coordinates.'
            )

        resolution = self.mapInfo['resolution']
        origin = self.mapInfo['origin']

        mapX = int(
            (x - origin[0]) / resolution
        )

        mapY = int(
            (y - origin[1]) / resolution
        )

        # Image Y coordinates increase downward while world/map
        # coordinates increase upward.
        mapY = self.mapImage.shape[0] - mapY

        return mapX, mapY

    def plotVisualization(
        self,
        outputFile: str | Path = 'trajectory_visualization.png'
    ) -> None:
        """
        Create and save the trajectory visualization.

        Args:
            outputFile: Path of the generated image.
        """
        if self.mapImage is None:
            raise RuntimeError(
                'Map must be loaded before creating visualization.'
            )

        print('Creating visualization...')

        outputPath = Path(outputFile)

        figure, axis = plt.subplots(
            figsize=(16, 12),
            dpi=150
        )

        try:
            self._plotMap(axis)
            self._plotTrajectory(axis)
            self._plotWaypoints(axis)

            axis.set_xlabel('X [m]', fontsize=14)
            axis.set_ylabel('Y [m]', fontsize=14)

            axis.set_title(
                'Robot Navigation - Depot Map\n'
                'Trajectory from ROS2 Bag + Waypoints',
                fontsize=16,
                fontweight='bold'
            )

            axis.legend(
                loc='upper right',
                fontsize=12
            )

            axis.grid(True, alpha=0.3)
            axis.set_aspect('equal')

            figure.tight_layout()
            figure.savefig(
                outputPath,
                bbox_inches='tight'
            )

            print(
                f'Visualization saved to: {outputPath}'
            )

            plt.show()

        finally:
            plt.close(figure)

    def _plotMap(self, axis) -> None:
        """Draw the warehouse map."""
        origin = self.mapInfo['origin']
        resolution = self.mapInfo['resolution']

        extent = [
            origin[0],
            origin[0] + self.mapImage.shape[1] * resolution,
            origin[1],
            origin[1] + self.mapImage.shape[0] * resolution
        ]

        axis.imshow(
            self.mapImage,
            cmap='gray',
            extent=extent,
            origin='upper',
            alpha=0.7
        )

    def _plotTrajectory(self, axis) -> None:
        """Draw the robot trajectory and start/end positions."""
        if not self.trajectory:
            print('⚠ No trajectory to display')
            return

        trajectoryX = [
            position[0]
            for position in self.trajectory
        ]

        trajectoryY = [
            position[1]
            for position in self.trajectory
        ]

        axis.plot(
            trajectoryX,
            trajectoryY,
            'b-',
            linewidth=2,
            alpha=0.6,
            label='Robot trajectory (from bag)'
        )

        # Start position.
        axis.plot(
            trajectoryX[0],
            trajectoryY[0],
            'go',
            markersize=15,
            label='Start',
            markeredgecolor='black',
            markeredgewidth=2
        )

        # End position.
        axis.plot(
            trajectoryX[-1],
            trajectoryY[-1],
            'ro',
            markersize=15,
            label='End',
            markeredgecolor='black',
            markeredgewidth=2
        )

    def _plotWaypoints(self, axis) -> None:
        """Draw goal waypoints and their numerical labels."""
        if not self.waypoints:
            print('⚠ No waypoints to display')
            return

        waypointX = [
            waypoint[0]
            for waypoint in self.waypoints
        ]

        waypointY = [
            waypoint[1]
            for waypoint in self.waypoints
        ]

        axis.scatter(
            waypointX,
            waypointY,
            c='red',
            s=200,
            marker='*',
            edgecolors='yellow',
            linewidths=2,
            label='Goal waypoints',
            zorder=5
        )

        for index, (x, y) in enumerate(self.waypoints, 1):
            axis.annotate(
                str(index),
                xy=(x, y),
                xytext=(5, 5),
                textcoords='offset points',
                fontsize=12,
                fontweight='bold',
                color='white',
                bbox={
                    'boxstyle': 'round,pad=0.3',
                    'facecolor': 'red',
                    'alpha': 0.7
                }
            )

    def run(
        self,
        outputFile: str | Path = 'trajectory_visualization.png'
    ) -> bool:
        """
        Run the complete visualization process.

        Args:
            outputFile: Path of the generated visualization.

        Returns:
            True if visualization completed successfully.
        """
        self.loadMap()
        self.extractTrajectoryFromBag()
        self.plotVisualization(outputFile)

        print('✓ Visualization completed successfully!')

        return True


def parseArguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Visualize robot trajectory from a ROS 2 bag.'
    )

    parser.add_argument(
        'bagPath',
        help='Path to the ROS 2 bag directory.'
    )

    parser.add_argument(
        '--map',
        dest='mapPath',
        default=(
            '~/Documents/ros2_lv/'
            'amr_lv_ws/src/lv2/'
            'robot_nav2_bringup/maps/depot.yaml'
        ),
        help='Path to the map YAML file.'
    )

    parser.add_argument(
        '--output',
        dest='outputPath',
        default='trajectory_visualization.png',
        help='Output image file name.'
    )

    return parser.parse_args()


def main() -> int:
    """Application entry point."""
    arguments = parseArguments()

    bagPath = Path(arguments.bagPath).expanduser()
    mapPath = Path(arguments.mapPath).expanduser()
    outputPath = Path(arguments.outputPath).expanduser()

    visualizer = BagVisualizer(
        bagPath=bagPath,
        mapYamlPath=mapPath
    )

    try:
        visualizer.run(outputPath)
        return 0

    except Exception as exception:
        print(f'✗ Error: {exception}')

        import traceback
        traceback.print_exc()

        return 1


if __name__ == '__main__':
    raise SystemExit(main())
