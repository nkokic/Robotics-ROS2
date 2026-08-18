import math

from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class FrameListener(Node):

    def __init__(self):
        super().__init__('navigation_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = "base_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = self.create_publisher(Pose, "pozicija", 10)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        # Look up for the transformation between target_frame and turtle2 frames
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation

            self.tf_broadcaster.publish(pose)

        except TransformException as ex:
            self.get_logger().info( f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return


def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
