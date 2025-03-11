import numpy as np
import matplotlib.pyplot as plt


class PurePursuit:
    def __init__(self, path, lookahead_distance, wheel_base):
        self.path = np.array(path)  # 경로 좌표 리스트 (x, y)
        self.lookahead_distance = lookahead_distance  # 추적할 거리
        self.wheel_base = wheel_base  # 차동구동 로봇의 바퀴 간 거리

    def find_lookahead_point(self, position):
        """현재 위치에서 가장 가까운 Lookahead Point를 찾는 함수"""
        for i in range(len(self.path) - 1):
            start, end = self.path[i], self.path[i + 1]
            # start ~ end를 선분으로 보고, Lookahead Distance를 만족하는 점 찾기
            vec = end - start
            length = np.linalg.norm(vec)
            if length == 0:
                continue
            unit_vec = vec / length
            proj = np.dot(position - start, unit_vec)
            closest_point = start + unit_vec * np.clip(proj, 0, length)
            if np.linalg.norm(closest_point - position) > self.lookahead_distance:
                return end  # Lookahead Distance를 만족하는 점 선택
        return self.path[-1]  # 마지막 점 반환 (종료 시)

    def compute_steering(self, position, theta):
        """현재 위치에서 Lookahead Point를 따라가기 위한 회전율 계산"""
        lookahead_point = self.find_lookahead_point(position)
        dx, dy = lookahead_point - position
        angle_to_target = np.arctan2(dy, dx)
        alpha = angle_to_target - theta  # 방향 오차
        curvature = 2 * np.sin(alpha) / self.lookahead_distance
        return curvature

    def compute_wheel_speeds(self, v, curvature):
        """주어진 속도 v에서 차동 구동 로봇의 바퀴 속도 계산"""
        omega = v * curvature  # 각속도
        v_r = v + (self.wheel_base / 2) * omega  # 오른쪽 바퀴 속도
        v_l = v - (self.wheel_base / 2) * omega  # 왼쪽 바퀴 속도
        return v_l, v_r


def simulate():
    # 샘플 경로 설정 (waypoints)
    path = np.array([[0, 0], [2, 1], [4, 2], [6, 0], [8, -1], [10, 0]])
    lookahead_distance = 1.0
    wheel_base = 0.5  # 바퀴 간 거리

    controller = PurePursuit(path, lookahead_distance, wheel_base)

    # 초기 상태 설정
    position = np.array([0, -0.5])
    theta = 0  # 초기 방향
    v = 1.0  # 선속도 (m/s)
    dt = 0.1  # 시간 간격

    positions = [position]

    for _ in range(100):  # 시뮬레이션 반복
        curvature = controller.compute_steering(position, theta)
        v_l, v_r = controller.compute_wheel_speeds(v, curvature)

        # 차동 구동 모델의 이동 업데이트
        omega = (v_r - v_l) / wheel_base  # 각속도
        theta += omega * dt  # 방향 업데이트
        position += np.array([v * np.cos(theta), v * np.sin(theta)]) * dt
        positions.append(position.copy())

        if np.linalg.norm(position - path[-1]) < 0.5:  # 목표 도착 여부 확인
            break

    positions = np.array(positions)

    # 결과 시각화
    plt.plot(path[:, 0], path[:, 1], "ro-", label="Path")  # 목표 경로
    plt.plot(
        positions[:, 0], positions[:, 1], "b-", label="Tracked Path"
    )  # 로봇 이동 경로
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Pure Pursuit Path Tracking")
    plt.grid()
    plt.show()


simulate()
