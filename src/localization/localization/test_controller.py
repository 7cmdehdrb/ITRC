import keyboard
import serial
import threading
import time
import math
from enum import Enum


class Twist(object):
    def __init__(self, linear: float = 0.0, angular: float = 0.0):
        self.linear = linear
        self.angular = angular

    def update(self, linear: float, angular: float):
        self.linear += linear
        self.angular += angular

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0

    def get_data(self):
        return self.linear, self.angular


class MDProtocol(object):
    def __init__(self):
        pass

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
        wheel2_encoder = int.from_bytes(data[10:14], byteorder="little", signed=True)
        # wheel2_state = MDRobotNode.MDProtocol.from_byte(data[9])

        return wheel1_rpm, wheel1_encoder, wheel2_rpm, wheel2_encoder

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

        packet[0] = 183
        packet[1] = 184
        packet[2] = 1
        packet[3] = 207  # PID_PNT_VEL_CMD
        packet[4] = data_num
        packet[5 : 5 + data_num] = data_packet
        packet[12] = self.create_checksum(packet)

        return packet

    def parse_feedback_packet(self, packet: bytearray):
        data_packet = packet[5:-1]
        serial_feedback = MDProtocol.from_bytes(data_packet)

        return serial_feedback


class KeyboardController(object):
    def __init__(self):
        self.key = None

        self.twist = Twist()

        self.thead1 = threading.Thread(target=self.detect_keyboard)
        self.thead1.start()

    def detect_keyboard(self):
        while True:
            keyboard.on_release_key("up", callback=self.detect_key, suppress=True)
            keyboard.on_release_key("down", callback=self.detect_key, suppress=True)
            keyboard.on_release_key("left", callback=self.detect_key, suppress=True)
            keyboard.on_release_key("right", callback=self.detect_key, suppress=True)
            keyboard.on_release_key("e", callback=self.detect_key, suppress=True)

            keyboard.wait()  # 프로그램 종료 방지

    def detect_key(self, e: keyboard.KeyboardEvent):
        key = e.name

        if key == "up":
            self.twist.update(0.1, 0.0)
        elif key == "down":
            self.twist.update(-0.1, 0.0)
        elif key == "left":
            self.twist.update(0.0, -0.1)
        elif key == "right":
            self.twist.update(0.0, 0.1)
        elif key == "e":
            self.twist.stop()


class SerialTest(object):
    def __init__(self):
        self.keyboard_control = KeyboardController()

        self.wheel_length = 0.64
        self.wheel_radius = 0.122
        self.reduction = 30

        self.protocol = MDProtocol()

        self.serial_port = "/dev/ttyUSB0"
        self.baudrate = 19200

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        except Exception as ex:
            print(f"Failed to open serial port: {ex}")
            self.serial_conn = None

        self.run()

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

        right_rpm = right_omega * 60 / (2 * math.pi)
        left_rpm = left_omega * 60 / (2 * math.pi)

        # rpm_right = int(right_speed * 9.5492743 / self.wheel_radius * self.reduction)
        # rpm_left = int(left_speed * 9.5492743 / self.wheel_radius * self.reduction)

        return int(left_rpm), int(right_rpm)

    def send_motor_command(self, rpm_left, rpm_right):
        """
        Convert rpm to byte array and send to serial port.
        """
        send_data = self.protocol.create_control_packet(rpm_left, rpm_right)

        # 시리얼 포트로 데이터 전송
        if self.serial_conn is not None:
            self.serial_conn.write(send_data)

    def run(self):
        while True:

            # >>> STEP 1. Read Keyboard Input <<<
            linear, angular = self.keyboard_control.twist.get_data()

            # >>> STEP 2. Compute RPM and Send Packets <<<
            rpm_left, rpm_right = self.compute_rpm(linear, angular)
            print(f"Left RPM: {rpm_left}, Right RPM: {rpm_right}")
            self.send_motor_command(rpm_left, rpm_right)

            # >>> STEP 3. Read Serial Data <<<
            if self.serial_conn is not None:
                if self.serial_conn.in_waiting:
                    data = self.serial_conn.read(20)

                    try:
                        wheel1_rpm, wheel1_encoder, wheel2_rpm, wheel2_encoder = (
                            self.protocol.from_bytes(data)
                        )

                        print(
                            f"Wheel1 RPM: {wheel1_rpm}, Wheel1 Encoder: {wheel1_encoder}"
                        )
                        print(
                            f"Wheel2 RPM: {wheel2_rpm}, Wheel2 Encoder: {wheel2_encoder}"
                        )
                    except Exception as ex:
                        print(f"Exception: {ex}")

                else:
                    print("Waiting for serial data")

            time.sleep(0.05)


if __name__ == "__main__":
    ser = SerialTest()
