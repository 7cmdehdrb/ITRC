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

# TF
from tf2_ros import *

# Python
import os
import sys
import json
import numpy as np
import serial
import time

# Custom
from custom_msgs.msg import SerialFeedback


class MDRobotNode(Node):
    def __init__(self):
        super().__init__("md_robot_node")

        resource_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "resource"
        )
        # 시리얼 포트 설정
        with open(os.path.join(resource_path, "serial_data.json"), "r") as f:
            self.serial_data = json.load(f)

        # >>> Parameters >>>
        self.serial_port = self.serial_data["serial_port"]
        self.baudrate = self.serial_data["serial_baudrate"]

        model_description = self.serial_data["model"]

        self.wheel_radius = model_description["wheel_radius"]
        self.wheel_length = model_description["wheel_radius"]
        self.reduction = model_description["wheel_radius"]
        self.maxrpm = model_description["wheel_radius"]
        # <<< Parameters End <<<

        self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # ROS2 구독 및 발행 설정
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_name + "/cmd_vel",
            self.cmd_vel_callback,
            qos_profile=qos_profile_system_default,
        )
        self.feedback_pub = self.create_publisher(
            SerialFeedback,
            self.get_name + "/serial_feedback",
            qos_profile=qos_profile_system_default,
        )

        # 데이터 송수신 루프
        self.last_time = self.get_clock().now()
        self.hz = 10
        self.timer = self.create_timer(float(1.0 / self.hz), self.read_serial_data)

    def cmd_vel_callback(self, msg: Twist):
        """
        Subscribe cmd_vel as Twist message and send motor command to serial port.
        Only linear.x and angular.z are used.
        """
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        rpm_left, rpm_right = self.compute_rpm(linear_speed, angular_speed)

        self.send_motor_command(rpm_left, rpm_right)

    def compute_rpm(self, linear: float, angular: float):
        """
        Input linear and angular speed and compute rpm for left and right wheels.
        Solve differential drive kinematics.
        """
        left_speed = linear - (angular * self.wheel_length / 2)
        right_speed = linear + (angular * self.wheel_length / 2)

        rpm_left = int(left_speed * 9.5492743 / self.wheel_radius * self.reduction)
        rpm_right = int(right_speed * 9.5492743 / self.wheel_radius * self.reduction)

        return rpm_left, rpm_right

    def send_motor_command(self, rpm_left, rpm_right):
        """
        Convert rpm to byte array and send to serial port.
        """
        send_data = bytearray(6)
        send_data[0] = 0xAA  # 헤더
        send_data[1] = rpm_left & 0xFF
        send_data[2] = (rpm_left >> 8) & 0xFF
        send_data[3] = rpm_right & 0xFF
        send_data[4] = (rpm_right >> 8) & 0xFF
        send_data[5] = 0xBB  # 종료 바이트

        # 시리얼 포트로 데이터 전송
        self.serial_conn.write(send_data)

    def read_serial_data(self):
        """
        Read serial data with self.hz frequency.
        """
        if self.serial_conn.in_waiting:
            data = self.serial_conn.read(32)  # 최대 32바이트 읽기

            if data and data[0] == 0xAA and data[-1] == 0xBB:  # 패킷 검증
                rpm_left = int.from_bytes(
                    data[1:3], byteorder="little", signed=True
                )  # 왼쪽 바퀴 RPM
                rpm_right = int.from_bytes(
                    data[3:5], byteorder="little", signed=True
                )  # 오른쪽 바퀴 RPM

                # ROS 메시지 생성 및 발행
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.last_time = current_time

                feedback_msg = SerialFeedback(
                    interval_time=dt,
                    motor_left_rpm=rpm_left,
                    motor_right_rpm=rpm_right,
                )
                self.feedback_pub.publish(feedback_msg)

            else:
                self.get_logger().warn("Invalid packet format")

    def destroy_node(self):
        self.serial_conn.close()
        super().destroy_node()


def main():
    rclpy.init(args=None)

    node = MDRobotNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
