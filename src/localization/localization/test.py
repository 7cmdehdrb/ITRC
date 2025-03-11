# ROS2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, qos_profile_system_default

# Message
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from custom_msgs.msg import AprilTagMessage, AprilTagArrayMessage


# TF
from tf2_ros import *

# Python
import numpy as np


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.timer = self.create_timer(0.1, self.run)

        self.test_pub1 = self.create_publisher(
            AprilTagMessage, self.get_name() + "/tag", qos_profile_system_default
        )
        self.test_pub2 = self.create_publisher(
            AprilTagArrayMessage, self.get_name() + "/tags", qos_profile_system_default
        )

    def run(self):
        tag_msg = AprilTagMessage()
        tag_msg.id = 0
        tag_msg.pose.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id="camera1_link"
        )
        tag_msg.pose.pose.pose.position = Point(x=1.0, y=2.0, z=3.0)
        tag_msg.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.test_pub1.publish(tag_msg)

        # msg = AprilTagArrayMessage()
        # for i in range(1, 5):
        #     i = float(i)
        #     tag = AprilTagMessage()
        #     tag.id = int(i)
        #     tag.header = Header(
        #         stamp=self.get_clock().now().to_msg(), frame_id="camera1_link"
        #     )
        #     tag.pose.pose.position = Point(x=i, y=i, z=i)
        #     tag.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        #     msg.tags.append(tag)

        # self.test_pub2.publish(msg)


def main():
    rclpy.init(args=None)

    node = TestNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
