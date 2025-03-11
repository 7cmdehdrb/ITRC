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
from custom_msgs.msg import SerialFeedback

# TF
from tf2_ros import *

# Python
import sys
import os
import json
import numpy as np
from base_package.transformation import Transformation
from ament_index_python.packages import get_package_share_directory


class WheelOdometryNode(Node):
    class WheelOdometry:
        def __init__(self):
            self.state = np.array([0.0, 0.0, 0.0])  # x, y, theta
            self.control_vector = np.array(
                [0.0, 0.0]
            )  # linear_velocity, angular_velocity

            self.model = np.array(
                [[np.cos(self.state[2]), 0], [np.sin(self.state[2]), 0], [0, 1]]
            )

        def update(self, control_vector: np.array, dt: float):
            """
            Update the state of the robot.
            control_vector: [linear_velocity, angular_velocity]
            dt: time step
            """
            self.model = np.array(
                [[np.cos(self.state[2]), 0], [np.sin(self.state[2]), 0], [0, 1]]
            )

            self.control_vector = control_vector
            self.state += self.model @ control_vector * dt

        def to_Transform(self, header: Header) -> Transform:
            euler_orientation = [0.0, 0.0, self.state[2]]
            qx, qy, qz, qw = Transformation.euler_to_quaternion(*euler_orientation)

            return TransformStamped(
                header=header,
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(
                        x=self.state[0],
                        y=self.state[1],
                        z=0.0,
                    ),
                    rotation=Quaternion(
                        x=qx,
                        y=qy,
                        z=qz,
                        w=qw,
                    ),
                ),
            )

        def to_Odometry(self, header: Header) -> Odometry:
            euler_orientation = [0.0, 0.0, self.state[2]]
            qx, qy, qz, qw = Transformation.euler_to_quaternion(*euler_orientation)

            # vx = self.control_vector[0] * np.cos(self.state[2])
            # vy = self.control_vector[0] * np.sin(self.state[2])
            vth = self.control_vector[1]

            odom = Odometry(
                header=header,
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self.state[0],
                            y=self.state[1],
                            z=0.0,
                        ),
                        orientation=Quaternion(
                            x=qx,
                            y=qy,
                            z=qz,
                            w=qw,
                        ),
                    ),
                    covariance=[0.0] * 36,
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=self.control_vector[0], y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=vth),
                    ),
                    covariance=[0.0] * 36,
                ),
            )

            return odom

        def to_Marker(self, header: Header) -> Marker:
            z_scale = 0.6

            euler_matrix = np.array([0.0, 0.0, self.state[2]])
            quat = Transformation.euler_to_quaternion(*euler_matrix)
            quat_keys = ["x", "y", "z", "w"]

            msg = Marker(
                header=header,
                ns="wheel_odometry",
                id=999,
                type=Marker.CUBE,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=self.state[0], y=self.state[1], z=z_scale / 2.0),
                    orientation=Quaternion(**dict(zip(quat_keys, quat))),
                ),
                scale=Vector3(x=0.8, y=0.45, z=z_scale),
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            )

            return msg

    def __init__(self):
        super().__init__("wheel_odometry_node")

        try:
            package_name = "robot_control"
            package_path = get_package_share_directory(package_name)
            resource_path = os.path.join(
                package_path, os.pardir, "ament_index", "resource_index", "packages"
            )

            with open(os.path.join(resource_path, "serial_data.json"), "r") as f:
                self.serial_data = json.load(f)

        except Exception as ex:
            self.get_logger().error(f"Failed to get package path: {ex}")

        self.wheel_radius = self.serial_data["model_description"]["wheel_radius"]
        self.wheel_length = self.serial_data["model_description"]["wheel_length"]
        self.reduction = self.serial_data["model_description"]["reduction"]

        self.buffer = Buffer(node=self, cache_time=Duration(seconds=0.1))
        self.tf_listener = TransformListener(
            buffer=self.buffer, node=self, qos=qos_profile_system_default
        )
        self.tf_broadcaster = TransformBroadcaster(
            node=self, qos=qos_profile_system_default
        )

        self.odometry = WheelOdometryNode.WheelOdometry()

        self.feedback_sub = self.create_subscription(
            SerialFeedback,
            "/robot_control/feedback",
            self.feedback_callback,
            qos_profile=qos_profile_system_default,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_name() + "/odom",
            qos_profile=qos_profile_system_default,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            self.get_name() + "/marker",
            qos_profile=qos_profile_system_default,
        )

        self.hz = 30
        self.timer = self.create_timer(1.0 / self.hz, self.publish_wheel_odometry)

    def publish_wheel_odometry(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")

        odom_msg = self.odometry.to_Odometry(header=header)
        transform_msg = self.odometry.to_Transform(header=header)
        marker = self.odometry.to_Marker(header=header)

        self.odom_pub.publish(odom_msg)
        self.tf_broadcaster.sendTransform(transform_msg)
        self.marker_pub.publish(marker)

    def feedback_callback(self, msg: SerialFeedback):
        """
        Calculate linear and angular velocity from motor feedback
        """
        dt = msg.interval_time  # seconds
        left_rpm = msg.motor_left_rpm
        right_rpm = msg.motor_right_rpm

        left_omega = (left_rpm / 60.0) * 2 * np.pi
        right_omega = (right_rpm / 60.0) * 2 * np.pi

        left_velocity = left_omega * self.wheel_radius
        right_velocity = right_omega * self.wheel_radius

        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / self.wheel_length

        self.odometry.update(np.array([linear_velocity, angular_velocity]), dt)


def main():
    rclpy.init(args=None)

    node = WheelOdometryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
