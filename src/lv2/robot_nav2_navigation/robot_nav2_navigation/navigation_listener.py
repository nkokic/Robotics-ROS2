import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class FrameListener(Node):
    """
    ROS 2 node that listens for a transform and publishes its position
    as a geometry_msgs/Pose message.

    The node looks up the transform from `map` to `base_link` and
    publishes the resulting pose on the `robot_position` topic.
    """

    def __init__(self) -> None:
        """Initialize the FrameListener node and its ROS 2 resources."""
        super().__init__('navigation_listener')

        # Frame whose position we want to track.
        self.targetFrame = 'base_link'

        # TF2 buffer and listener used to receive coordinate transforms.
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # Publisher for the calculated pose.
        self.posePublisher = self.create_publisher(
            Pose,
            'robot_position',
            10
        )

        # Execute onTimer every 100 ms.
        self.timer = self.create_timer(0.1, self.onTimer)

    def onTimer(self) -> None:
        """
        Look up the transform and publish the corresponding pose.

        The transform is calculated from `map` to `targetFrame`.
        If the transform is not currently available, the exception
        is logged and the next timer cycle will try again.
        """
        fromFrame = self.targetFrame
        toFrame = 'map'

        try:
            transform = self.tfBuffer.lookup_transform(
                toFrame,
                fromFrame,
                rclpy.time.Time()
            )

            pose = Pose()

            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z

            pose.orientation = transform.transform.rotation

            self.posePublisher.publish(pose)

        except TransformException as exception:
            self.get_logger().info(
                f'Could not transform {toFrame} to {fromFrame}: {exception}'
            )


def main(args=None) -> None:
    """
    Initialize ROS 2, start the FrameListener node, and spin it.

    The node is shut down cleanly when the application is interrupted.
    """
    rclpy.init(args=args)

    node = FrameListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
