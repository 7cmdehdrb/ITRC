import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.pub = self.create_publisher(
            Twist, "/md_robot_node/cmd_vel", qos_profile=qos_profile_system_default
        )

        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        msg = Twist()
        msg.linear.x = -0.0
        msg.angular.z = 0.0
        # msg.angular.z = -(2 * 3.14159) / 0.5  # 1 rotation per minute
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
