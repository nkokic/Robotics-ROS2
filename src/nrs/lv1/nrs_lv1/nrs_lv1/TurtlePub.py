import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from nrs_lv1_interfaces.msg import Znamenitost
from turtlesim.msg import Pose


class ZnamenitostiNode(Node):
    def __init__(self):
        super().__init__("znamenitosti_node")

        # ---------------------
        # Deklaracija parametara
        # ---------------------
        self.declare_parameter("period", 1.0)     # sekunde
        self.declare_parameter("prefiks", "Polozaj")

        self.period = self.get_parameter("period").get_parameter_value().double_value
        self.prefiks = self.get_parameter("prefiks").get_parameter_value().string_value

        # Brojač za naziv znamenitosti
        self.counter = 1

        # ---------------------
        # Pretplata na turtlesim pozu
        # ---------------------
        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10
        )

        # ---------------------
        # Izdavač na /znamenitosti
        # ---------------------
        self.pub = self.create_publisher(
            Znamenitost,
            "/znamenitosti",
            10
        )

        # Timer za periodičko objavljivanje
        self.timer = self.create_timer(
            self.period,
            self.timer_callback
        )

        self.current_pose = Pose()  # inicijalna vrijednost
        self.get_logger().info("ZnamenitostiNode pokrenut.")

    def pose_callback(self, msg : Pose):
        """Spremamo zadnju poznatu pozu turtlesima."""
        self.current_pose = msg

    def timer_callback(self):
        """Periodički objavljujemo novu znamenitost."""
        znamenitost_msg = Znamenitost()

        # Prebacujemo turtlesim Pose → Pose2D
        znamenitost_msg.pose.x = self.current_pose.x
        znamenitost_msg.pose.y = self.current_pose.y
        znamenitost_msg.pose.theta = self.current_pose.theta

        znamenitost_msg.naziv = f"{self.prefiks}_{self.counter}"
        self.counter += 1

        self.pub.publish(znamenitost_msg)

        self.get_logger().info(
            f"Objavljeno: {znamenitost_msg.naziv} "
            f"({znamenitost_msg.pose.x:.2f}, {znamenitost_msg.pose.y:.2f}, {znamenitost_msg.pose.theta:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ZnamenitostiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
