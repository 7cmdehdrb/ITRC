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
from custom_msgs.msg import AprilTag, AprilTagArray, AprilTagStamped

# TF
from tf2_ros import *

# Python
import os
import sys
import json
import numpy as np
import cv2
import cv_bridge
import pupil_apriltags
from ament_index_python.packages import get_package_share_directory
from base_package.transformation import Transformation


class ApriltagDetector(Node):
    def __init__(self):
        super().__init__("apriltag_detector_node")

        # Load parameters
        try:
            package_name = "localization"
            package_path = get_package_share_directory(package_name)
            resource_path = os.path.join(
                package_path, os.pardir, "ament_index", "resource_index", "packages"
            )

            with open(os.path.join(resource_path, "april_parameter.json"), "r") as f:
                self.params = json.load(f)

        except Exception as ex:
            self.get_logger().error(f"Failed to get package path: {ex}")

        # TF
        self.tfBuffer = Buffer(node=self, cache_time=Duration(seconds=0.1))
        self.tfListener = TransformListener(
            buffer=self.tfBuffer, node=self, qos=qos_profile_system_default
        )

        # ROS2 Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera1/color/image_raw",
            self.image_callback,
            qos_profile_system_default,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera1/color/camera_info",
            self.camera_info_callback,
            qos_profile_system_default,
        )

        self.image_pub = self.create_publisher(
            Image, self.get_name() + "/image", qos_profile_system_default
        )
        self.tag_pub = self.create_publisher(
            AprilTagArray, self.get_name() + "/tags", qos_profile_system_default
        )
        self.tag_marker_pub = self.create_publisher(
            MarkerArray, self.get_name() + "/tags/marker", qos_profile_system_default
        )

        # ROS2 messages initialization
        self.tag_array_msg = None
        self.bounding_image = None

        # CV Bridge
        self.bridge = cv_bridge.CvBridge()

        # Initialize apriltag detector
        self.tag_detector = pupil_apriltags.Detector(families="tag36h11")

        self.camera_params = [0, 0, 0, 0]
        self.frame_id = "camera_link"

        self.hz = 10
        self.timer = self.create_timer(float(1 / self.hz), self.run)

    def run(self):
        if self.tag_array_msg is not None:
            # 1. Publish AprilTagArray message
            self.tag_pub.publish(self.tag_array_msg)

            # 2. Publish AprilTagArray message as MarkerArray
            marker_msg = ApriltagDetector.parse_apriltag_to_marker(
                self.tag_array_msg, self.params["tag_size"]
            )
            self.tag_marker_pub.publish(marker_msg)

        if self.bounding_image is not None:
            # 3. Publish bounding image
            self.image_pub.publish(self.bounding_image)

    def camera_info_callback(self, msg: CameraInfo):
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        self.camera_params = [fx, fy, cx, cy]
        self.frame_id = msg.header.frame_id

    def image_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # https://chaitanyantr.github.io/apriltag.html => When "tag_size" is 0.038, Total Size, mm is 41, Tag size, mm is 32.8
        detections: list = self.tag_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.params["tag_size"],
        )

        tag_array_msg = AprilTagArray(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        )

        # Append all detected tags
        for detection in detections:
            detection: pupil_apriltags.Detection

            # 1. Get tag id
            tag_id = int(detection.tag_id)

            # 2-1. Calculate transform matrix in REALSENSE coordinate
            translation_matrix = detection.pose_t.flatten()

            rotation_matrix = detection.pose_R
            quaternion_matrix = Transformation.rotation_matrix_to_quaternion(
                rotation_matrix
            )

            point_keys = ["x", "y", "z"]
            orientation_keys = ["x", "y", "z", "w"]

            # 3. Parse to ROS message
            tag_msg = AprilTag(
                id=tag_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(**dict(zip(point_keys, translation_matrix))),
                        orientation=Quaternion(
                            **dict(zip(orientation_keys, quaternion_matrix))
                        ),
                    ),
                    covariance=[0.0] * 36,
                ),
            )

            # 4. Append to tag array message
            tag_array_msg.tags.append(tag_msg)

            # 5. Draw bounding box
            bbox = detection.corners
            bbox_xs = [bbox[i][0] for i in range(4)]
            bbox_ys = [bbox[i][1] for i in range(4)]

            b1 = (int(min(bbox_xs) - 10), int(min(bbox_ys) - 10))
            b2 = (int(max(bbox_xs) + 10), int(max(bbox_ys) + 10))

            cv2.rectangle(image, b1, b2, (0, 255, 0), 5)

            # 6. Draw tag id
            cv2.putText(
                image,
                str(tag_id),
                (b1[0], b1[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # Update tag array message
        self.tag_array_msg = tag_array_msg

        # Update image message
        self.bounding_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

    @staticmethod
    def parse_apriltag_to_marker(tags: AprilTagArray, tag_size: float) -> Marker:
        marker_array = MarkerArray()

        for tag in tags.tags:
            tag: AprilTag

            marker = Marker()
            marker.id = tag.id
            marker.ns = str(tag.id)
            marker.header = tags.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = tag.pose.pose
            marker.scale = Vector3(x=tag_size, y=tag_size, z=0.001)
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            # marker.lifetime = 0.1

            marker_array.markers.append(marker)

        return marker_array


def main():
    rclpy.init(args=None)

    node = ApriltagDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
