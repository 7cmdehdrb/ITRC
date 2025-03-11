# ROS2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, qos_profile_system_default
from rclpy.task import Future


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


class State(object):
    def __init__(
        self,
        position: Point,
        orientation: Quaternion,
        velocity: float,
        acceleration: float,
    ):
        """
        :param position: Point
        :param orientation: Quaternion
        :param velocity: float
        :param acceleration: float
        """
        self.position = position
        self.orientation = orientation
        self.velocity = velocity
        self.acceleration = acceleration

    def to_dict(self, start: bool = False):
        header = "start" if start else "end"

        return {
            f"{header}_position": self.position,
            f"{header}_orientation": self.orientation,
            f"{header}_velocity": self.velocity,
            f"{header}_acceleration": self.acceleration,
        }


class QPPlannerClient(object):
    """
    Client class for Quintic Polynomials Planner
    Use QPPlannerClient.request_path() to update the path
    """

    def __init__(self, node: Node):
        self.node = node

        # ROS
        self.qp_client = self.node.create_client(
            QuinticPolynomials, "quintic_polynomials_service"
        )
        self.path_publisher = self.node.create_publisher(
            Path,
            self.node.get_name() + "/quintic_polynomials_path",
            qos_profile=qos_profile_system_default,
        )

        while not self.qp_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Service not available, waiting again...")

    def request_path(self, start_state: State, end_state: State):
        # Parse to dictionary
        integrated_dict = {
            **start_state.to_dict(start=True),
            **end_state.to_dict(start=False),
        }

        # Send request
        request = QuinticPolynomials.Request(**integrated_dict)
        self.send_qp_request(request)

    def send_qp_request(self, request: QuinticPolynomials.Request):
        self.qp_client.call_async(request)
        future: Future = self.qp_client.call_async(request)
        future.add_done_callback(self.qp_response_callback)

    def qp_response_callback(self, future: Future):
        response: QuinticPolynomials.Response = future.result()

        path = response.path

        self.node.get_logger().info("Publishing path...")

        self.path = path

        for _ in range(10):
            self.path_publisher.publish(path)


def main():
    rclpy.init(args=None)

    node = Node("quintic_polynomials_planner_client_node")
    client = QPPlannerClient(node=node)

    euler_matrix = np.array([0.0, 0.0, np.deg2rad(90.0)])
    quat = Transformation.euler_to_quaternion(*euler_matrix)
    quat_keys = ["x", "y", "z", "w"]

    client.request_path(
        start_state=State(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            velocity=0.0,
            acceleration=0.0,
        ),
        end_state=State(
            position=Point(x=10.0, y=10.0, z=0.0),
            orientation=Quaternion(**dict(zip(quat_keys, quat))),
            velocity=1.0,
            acceleration=0.0,
        ),
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
