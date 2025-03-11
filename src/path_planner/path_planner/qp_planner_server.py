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
from custom_msgs.srv import QuinticPolynomials

# TF
from tf2_ros import *

# Python
import numpy as np
from base_package.transformation import Transformation
from path_planner.quintic_polynomials_planner import quintic_polynomials_planner


class QPPlannerServer(Node):
    def __init__(self):
        super().__init__("quintic_polynomials_planner_server_node")

        self.max_accel = 1.0  # max accel [m/ss]
        self.max_jerk = 0.5  # max jerk [m/sss]
        self.dt = 0.1  # time tick [s]
        self.frame_id = "odom"

        # ROS Service
        self.srv = self.create_service(
            QuinticPolynomials,
            "quintic_polynomials_service",
            self.handle_quintic_polynomials,
        )

        self.get_logger().info("Quintic polynomials planner server has been started.")

    def handle_quintic_polynomials(
        self, request: QuinticPolynomials.Request, response: QuinticPolynomials.Response
    ):
        self.get_logger().info("Received request for quintic polynomials.")

        start_quat = np.array(
            [
                request.start_orientation.x,
                request.start_orientation.y,
                request.start_orientation.z,
                request.start_orientation.w,
            ]
        )
        _, _, syaw = Transformation.quaternion_to_euler(start_quat)

        end_quat = np.array(
            [
                request.end_orientation.x,
                request.end_orientation.y,
                request.end_orientation.z,
                request.end_orientation.w,
            ]
        )
        _, _, eyaw = Transformation.quaternion_to_euler(end_quat)

        request_data = [
            request.start_position.x,
            request.start_position.y,
            syaw,
            request.start_velocity,
            request.start_acceleration,
            request.end_position.x,
            request.end_position.y,
            eyaw,
            request.end_velocity,
            request.end_acceleration,
            self.max_accel,
            self.max_jerk,
            self.dt,
        ]

        path = quintic_polynomials_planner(*request_data)
        path_msg = self.parse_to_Path(*path)

        response.path = path_msg

        self.get_logger().info("Quintic polynomials planner has been completed.")

        return response

    def parse_to_Path(self, time, x, y, yaw, v, a, j):
        path = Path()

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        path.header = header
        path.poses = []

        quaternion_key = ["x", "y", "z", "w"]

        for time, x, y, yaw, v, a, j in zip(time, x, y, yaw, v, a, j):
            pose = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=x, y=y, z=0.0),
                    orientation=Quaternion(
                        **dict(
                            zip(
                                quaternion_key,
                                Transformation.euler_to_quaternion(0, 0, yaw),
                            )
                        )
                    ),
                ),
            )
            path.poses.append(pose)

        return path


def main():
    rclpy.init(args=None)

    node = QPPlannerServer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
