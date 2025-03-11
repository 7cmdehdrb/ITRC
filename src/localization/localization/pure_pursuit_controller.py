"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""

import numpy as np
import math

from nav_msgs.msg import Odometry

from base_package.transformation import Transformation


class DifferentialState:
    def __init__(self, model_description: dict):
        self.wheel_length = model_description["wheel_length"]
        self.wheel_radius = model_description["wheel_radius"]

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def update(self, odometry: Odometry):
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y

        quat = np.array(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        _, _, yaw = Transformation.quaternion_to_euler(quat)

        self.yaw = yaw
        self.v = odometry.twist.twist.linear.x

    @staticmethod
    def differential_drive_rpm(yaw_rate, speed, wheel_base=0.5, wheel_radius=0.1):
        """
        Differential Drive 로봇에서 왼쪽, 오른쪽 바퀴의 RPM을 계산하는 함수.

        Parameters:
        yaw_rate (float): 원하는 회전율 (rad/s)
        speed (float): 원하는 전진 속도 (m/s)
        wheel_base (float): 두 바퀴 사이의 거리 (m). 기본값: 0.5 m.
        wheel_radius (float): 바퀴의 반지름 (m). 기본값: 0.1 m.

        Returns:
        tuple: (left_wheel_RPM, right_wheel_RPM)
        """
        # 왼쪽, 오른쪽 바퀴의 선형 속도 (m/s)
        # v = (v_left + v_right) / 2,  yaw_rate = (v_right - v_left) / wheel_base
        v_left = speed - (yaw_rate * wheel_base / 2.0)
        v_right = speed + (yaw_rate * wheel_base / 2.0)

        # 선형 속도를 바퀴의 회전 속도 (rad/s)로 변환: omega = v / wheel_radius
        omega_left = v_left / wheel_radius
        omega_right = v_right / wheel_radius

        # rad/s를 RPM으로 변환: RPM = (omega [rad/s] * 60) / (2*pi)
        rpm_left = omega_left * (60 / (2 * math.pi))
        rpm_right = omega_right * (60 / (2 * math.pi))

        return rpm_left, rpm_right


def proportional_control(target, current, Kp=1.0):
    a = Kp * (target - current)
    return a


class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state: DifferentialState, Lfc=2.0, k=0.1):
        """
        :param state: current state
        :param Lfc: look-ahead distance
        :param k: proportional gain
        :return: index, look ahead target point index
        """
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                if ind >= len(self.cx) - 1:
                    break

                distance_next_index = state.calc_distance(
                    self.cx[ind + 1], self.cy[ind + 1]
                )
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(
    state: DifferentialState, trajectory: TargetCourse, pind: int, Lfc=2.0, k=0.1
):
    ind, _ = trajectory.search_target_index(state, Lfc=Lfc, k=k)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
        return 0.0, -1

    # 회전이 필요한 각도(α)
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    return alpha, ind
