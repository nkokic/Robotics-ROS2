import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from geometry_msgs.msg import Pose2D

class TwistPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.start_time = time.time()
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Twist()
        t = time.time() - self.start_time

        # first derivatives
        xp = 4 * math.cos(t)
        yp = 4 * math.cos(2 * t)

        # second derivatives
        xpp = -4 * math.sin(t)
        ypp = -8 * math.sin(2 * t)

        # speed
        v = math.sqrt(xp**2 + yp**2)

        # angular velocity of tangent
        denom = xp**2 + yp**2
        if denom < 1e-6:
            omega = 0.0
        else:
            omega = (xp * ypp - yp * xpp) / denom

        msg.linear.x = v
        msg.angular.z = omega
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {str(msg)}")


def main(args=None):
    rclpy.init(args=args)
    time.sleep(0.5)
    pub = TwistPublisher()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
