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
import os
import sys
import json
import numpy as np
import serial
import time
from enum import Enum

# Custom
from ament_index_python.packages import get_package_share_directory

class MDRobotNode(Node):
    class MDProtocol(object):
        class PacketType(Enum):
            RMID = 183
            TMID = 184
            ID = 1
            PID = 207 # PID_PNT_VEL_CMD

        def __init__(self):
            pass

        @staticmethod
        def from_byte(data: bytearray):
            state_param = data[2]
            start_stop_signal = (state_param >> 0) & 0x01
            run_brake_signal = (state_param >> 1) & 0x01
            sync_mode_slave_status = (state_param >> 2) & 0x01
            sync_mode_comm_fail = (state_param >> 3) & 0x01
            overload_alarm = (state_param >> 4) & 0x01
            undervoltage_alarm = (state_param >> 5) & 0x01

            return {
                "start_stop_signal": start_stop_signal,
                "run_brake_signal": run_brake_signal,
                "sync_mode_slave_status": sync_mode_slave_status,
                "sync_mode_comm_fail": sync_mode_comm_fail,
                "overload_alarm": overload_alarm,
                "undervoltage_alarm": undervoltage_alarm,
            }

        @staticmethod
        def from_bytes(data: bytearray):
            """
            d0, d1 : wheel 1 RPM
            d2 : wheel 1 state param
            d3, d4, d5 d6 : wheel 1 encoder

            d7, d8 : wheel 2 RPM
            d9 : wheel 2 state param
            d10, d11, d12, d13 : wheel 2 encoder

            State Parameter (1byte)

            BIT0: START/STOP 신호 반전 여부 (1: HIGH → ON, 0: LOW → ON)
            BIT1: RUN/BRAKE 신호 반전 여부
            BIT2: 동기제어(SYNC) 모드에서 슬레이브 제어기 상태 (1: FAIL)
            BIT3: 동기제어에서 슬레이브 제어기와의 통신 실패 여부 (1: FAIL)
            BIT4: 과부하 알람 발생 여부 (FAULT 발생)
            BIT5: 저전압 알람 발생 여부 (저전압 감지됨)
            """

            # print(data[-1])
            # return SerialFeedback()
            wheel1_rpm = int.from_bytes(data[0:2], byteorder="little", signed=True)
            wheel1_encoder = int.from_bytes(data[3:7], byteorder="little", signed=True)
            # wheel1_state = MDRobotNode.MDProtocol.from_byte(data[2])

            wheel2_rpm = int.from_bytes(data[7:9], byteorder="little", signed=True)
            wheel2_encoder = int.from_bytes(
                data[10:14], byteorder="little", signed=True
            )
            # wheel2_state = MDRobotNode.MDProtocol.from_byte(data[9])

            wheel1_rpm = np.clip(wheel1_rpm, 0, 10)
            wheel2_rpm = np.clip(wheel2_rpm, 0, 10)

            msg = SerialFeedback(
                motor_left_rpm=int(wheel1_rpm),
                moter_left_position=int(wheel1_encoder),
                motor_right_rpm=int(wheel2_rpm),
                moter_right_position=int(wheel2_encoder),
            )

            return msg

        def create_checksum(self, packet: bytearray):
            byChkSend = sum(packet[:-1]) & 0xFF
            return (~byChkSend + 1) & 0xFF

        def create_control_packet(self, left_rpm: int, right_rpm: int):
            data_num = 7
            data_length = 6 + data_num  # HEADER + data_num

            # >>> DATA >>>
            data_packet = bytearray(data_num)

            data_packet[0] = 1
            data_packet[1:3] = left_rpm.to_bytes(2, byteorder="little", signed=True)

            # wheel 2
            data_packet[3] = 1
            data_packet[4:6] = right_rpm.to_bytes(2, byteorder="little", signed=True)

            # return data
            data_packet[6] = 1  # PID_PNT_MONITOR
            # <<< DATA <<<
            
            packet = bytearray(data_length)

            packet[0] = MDRobotNode.MDProtocol.PacketType.RMID.value
            packet[1] = MDRobotNode.MDProtocol.PacketType.TMID.value
            packet[2] = MDRobotNode.MDProtocol.PacketType.ID.value
            packet[3] = MDRobotNode.MDProtocol.PacketType.PID.value  # PID_PNT_VEL_CMD
            packet[4] = data_num
            packet[5 : 5 + data_num] = data_packet
            packet[12] = self.create_checksum(packet)

            return packet

        def parse_feedback_packet(self, packet: bytearray):
            # rmid = packet[0] # 183
            # tmid = packet[1]
            # id = packet[2]
            # pid = packet[3] # 216
            # data_num = packet[4]  # 14
            # checksum = packet[-1]

            data_packet = packet[5:-1]
            serial_feedback = MDRobotNode.MDProtocol.from_bytes(data_packet)

            return serial_feedback

    def __init__(self):
        super().__init__("md_robot_node")

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
            
        # resource_path = os.path.join(
        #     os.path.dirname(os.path.dirname(__file__)), "resource"
        # )
        # # 시리얼 포트 설정
        # with open(os.path.join(resource_path, "serial_data.json"), "r") as f:
        #     self.serial_data = json.load(f)


        # >>> Parameters >>>
        self.serial_port = self.serial_data["serial_port"]
        self.baudrate = self.serial_data["baud_rate"]

        model_description = self.serial_data["model_description"]

        self.wheel_radius = model_description["wheel_radius"]
        self.wheel_length = model_description["wheel_radius"]
        self.reduction = model_description["wheel_radius"]
        self.maxrpm = model_description["wheel_radius"]
        # <<< Parameters End <<<

        self.protocol = MDRobotNode.MDProtocol()

        self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # ROS2 구독 및 발행 설정
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_name() + "/cmd_vel",
            self.cmd_vel_callback,
            qos_profile=qos_profile_system_default,
        )
        self.feedback_pub = self.create_publisher(
            SerialFeedback,
            self.get_name() + "/serial_feedback",
            qos_profile=qos_profile_system_default,
        )

        self.response_data = bytearray()

        # 데이터 송수신 루프
        self.last_time = self.get_clock().now()
        self.hz = 30
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
        
        # 각 바퀴의 선속도 계산
        right_speed = linear + (angular * self.wheel_length / 2)
        left_speed = linear - (angular * self.wheel_length / 2)
        
        right_omega = right_speed / self.wheel_radius
        left_omega = left_speed / self.wheel_radius

        right_rpm = right_omega * 60 / (2 * np.pi)
        left_rpm = left_omega * 60 / (2 * np.pi)

        # rpm_right = int(right_speed * 9.5492743 / self.wheel_radius * self.reduction)
        # rpm_left = int(left_speed * 9.5492743 / self.wheel_radius * self.reduction)

        return left_rpm, right_rpm

    def send_motor_command(self, rpm_left, rpm_right):
        """
        Convert rpm to byte array and send to serial port.
        """
        send_data = self.protocol.create_control_packet(rpm_left, rpm_right)

        # self.get_logger().info("Send motor command: {}".format(send_data))

        # 시리얼 포트로 데이터 전송
        self.serial_conn.write(send_data)

    def read_serial_data(self):
        """
        Read serial data with self.hz frequency.
        """
        if self.serial_conn.in_waiting:
            data = self.serial_conn.read(20)  # 20 bytes

            try:
                serial_feedback = self.protocol.parse_feedback_packet(data)
                
                # Calculate interval time
                current_time = self.get_clock().now()
                delta_time = current_time - self.last_time
                serial_feedback.interval_time = delta_time.nanoseconds / 1e9
                self.last_time = current_time
                
                self.feedback_pub.publish(serial_feedback)
            except Exception as ex:
                self.get_logger().error("Error parsing serial data: {}".format(ex))

        else:
            self.get_logger().warn("Waiting for serial data")

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
