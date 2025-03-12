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
import numpy as np
from base_package.transformation import Transformation
from path_planner.qp_planner_client import State, QPPlannerClient
from localization.pure_pursuit_controller import (
    DifferentialState,
    TargetCourse,
    proportional_control,
    pure_pursuit_steer_control,
)
import json
import os
import sys
from ament_index_python.packages import get_package_share_directory


class FakeLocalization(Node):
    def __init__(self):
        super().__init__("fake_localization_node")

        # TF
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)

        # Path Planner
        self.qp_client = QPPlannerClient(node=self)

        # ROS2
        self.odom_sub = self.create_subscription(
            Odometry,
            "/wheel_odometry_node/odom",
            callback=self.odom_callback,
            qos_profile=qos_profile_system_default,
        )
        self.serial_feedback_pub = self.create_publisher(
            SerialFeedback,
            "/robot_control/feedback",
            qos_profile=qos_profile_system_default,
        )
        self.odom = Odometry()

        # Path Tracking
        try:
            package_name = "robot_control"
            package_path = get_package_share_directory(package_name)
            resource_path = os.path.join(
                package_path, os.pardir, "ament_index", "resource_index", "packages"
            )

            with open(os.path.join(resource_path, "serial_data.json"), "r") as f:
                self.model_description = json.load(f)["model_description"]

        except Exception as ex:
            self.get_logger().error(f"Failed to get package path: {ex}")

        self.differential_state = DifferentialState(
            model_description=self.model_description
        )

        # Path Tracking Variables
        self.path = None
        self.target_course = None
        self.target_speed = 1.0
        self.target_ind = 0
        self.last_currrent_ind = 0
        self.Lfc = 1.0

        self.hz = 15
        self.timer = self.create_timer(float(1.0 / self.hz), self.run)

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def update_path(self, end_state: State) -> Path:
        # >>> STEP 1: Request Path <<<
        self.get_logger().info("Requesting Path...")

        # Request Path
        start_state = State(
            position=self.odom.pose.pose.position,
            orientation=self.odom.pose.pose.orientation,
            velocity=0.0,
            acceleration=0.0,
        )

        self.qp_client.request_path(start_state=start_state, end_state=end_state)

        return True

    @staticmethod
    def parse_path_to_target_course(path: Path) -> TargetCourse:
        cx = []
        cy = []

        for p in path.poses:
            p: PoseStamped

            x = p.pose.position.x
            y = p.pose.position.y

            cx.append(x)
            cy.append(y)

        return TargetCourse(cx=cx, cy=cy)

    def reset(self):
        self.get_logger().info("Goal Reachedl. Resetting...")

        self.path = None
        self.target_course = None
        self.target_speed = 1.0
        self.target_ind = 0
        self.last_currrent_ind = 0
        self.Lfc = 1.0

        self.qp_client.future = None

    def run(self):
        self.differential_state.update(self.odom)

        # >>> STEP 2: Update Path <<<
        if self.path is None:
            # Initial Path Request
            if self.qp_client.future is None:
                self.differential_state.yaw
                euler_matrix = np.array(
                    [
                        0.0,
                        0.0,
                        self.differential_state.yaw
                        + np.deg2rad(np.random.randint(-90, 90)),
                    ]
                )
                quaternion_matrix = Transformation.euler_to_quaternion(*euler_matrix)
                quaternion_key = ["x", "y", "z", "w"]

                end_state = State(
                    position=Point(
                        x=self.differential_state.x + float(np.random.randint(5, 10)),
                        y=self.differential_state.x - float(np.random.randint(5, 10)),
                        z=0.0,
                    ),
                    orientation=Quaternion(
                        **dict(zip(quaternion_key, quaternion_matrix))
                    ),
                    velocity=2.0,
                    acceleration=0.0,
                )

                self.update_path(end_state=end_state)

                return True

            # Check if the previous request is done
            elif self.qp_client.future.done():
                self.get_logger().info("Get response!")

                self.qp_client.publish_path()

                self.path = (
                    self.qp_client.path if self.qp_client.path is not None else None
                )
                self.target_course = FakeLocalization.parse_path_to_target_course(
                    self.path
                )
                self.target_ind, _ = self.target_course.search_target_index(
                    self.differential_state, Lfc=self.Lfc, k=0.1
                )

                return True

            # Previous request is not finished yet
            else:
                self.get_logger().warn("Previous request is not finished yet.")
                return False

        # >>> STEP 3: Track the Path <<<
        dt = float(1.0 / self.hz)

        # Calc control input
        alpha, self.target_ind = pure_pursuit_steer_control(
            self.differential_state,
            self.target_course,
            self.target_ind,
            Lfc=self.Lfc,
            k=0.1,
        )

        current_idx = self.target_course.search_current_index(self.differential_state)

        print(
            f"Current Index: {current_idx}, Last Cureent Index: {self.last_currrent_ind} Target Index: {self.target_ind}"
        )

        if current_idx < self.last_currrent_ind:
            self.get_logger().warn("Path tracking is not working properly.")
            self.reset()
            return True

        self.last_currrent_ind = current_idx

        if self.target_ind == len(self.target_course.cx) - 1:

            if len(self.target_course.cx) - 1 - current_idx < 3:
                self.get_logger().info("Goal Reached!")
                self.reset()
                return True

        ai = proportional_control(self.target_speed, self.differential_state.v, Kp=1.0)

        v = self.differential_state.v + ai * dt  # update velocity

        # >>> STEP 4: Publish Control Input <<<
        wheel_left, wheel_right = DifferentialState.differential_drive_rpm(
            yaw_rate=alpha,
            speed=v,
            wheel_base=self.model_description["wheel_length"],
            wheel_radius=self.model_description["wheel_radius"],
        )

        feedback = SerialFeedback()
        feedback.interval_time = dt
        feedback.motor_left_rpm = int(wheel_left)
        feedback.motor_right_rpm = int(wheel_right)

        self.serial_feedback_pub.publish(feedback)


def main():
    rclpy.init(args=None)

    node = FakeLocalization()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
