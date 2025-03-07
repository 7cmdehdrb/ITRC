import numpy as np
from scipy.spatial.transform import Rotation as R


class Transformation:
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.array:
        """
        Convert euler angles to quaternion.
        """
        return R.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_quat()

    @staticmethod
    def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.array:
        """
        Convert euler angles to rotation matrix.
        """
        return R.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_matrix()

    @staticmethod
    def quaternion_to_euler(quaternion: np.array) -> np.array:
        """
        Convert quaternion to euler angles.
        """
        return R.from_quat(quaternion).as_euler("xyz", degrees=True)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion: np.array) -> np.array:
        """
        Convert quaternion to rotation matrix.
        """
        return R.from_quat(quaternion).as_matrix()

    @staticmethod
    def rotation_matrix_to_quaternion(rotation_matrix: np.array) -> np.array:
        """
        Convert rotation matrix to quaternion.
        """
        return R.from_matrix(rotation_matrix).as_quat()

    @staticmethod
    def rotation_matrix_to_euler(rotation_matrix: np.array) -> np.array:
        """
        Convert rotation matrix to euler angles.
        """
        return R.from_matrix(rotation_matrix).as_euler("xyz", degrees=True)

    @staticmethod
    def transform_realsense_to_ros(transform_matrix: np.ndarray) -> np.array:
        """
        Realsense 좌표계를 ROS 좌표계로 변환합니다.

        Realsense 좌표계:
            X축: 이미지의 가로 방향 (오른쪽으로 증가).
            Y축: 이미지의 세로 방향 (아래쪽으로 증가).
            Z축: 카메라 렌즈가 바라보는 방향 (깊이 방향).

        ROS 좌표계:
            X축: 앞으로 나아가는 방향.
            Y축: 왼쪽으로 이동하는 방향.
            Z축: 위로 이동하는 방향.

        Args:
            transform_matrix (np.ndarray): 4x4 변환 행렬.

        Returns:
            np.ndarray: 변환된 4x4 변환 행렬 (ROS 좌표계 기준).
        """
        if transform_matrix.shape != (4, 4):
            raise ValueError("Input transformation matrix must be a 4x4 matrix.")

        # Realsense에서 ROS로 좌표계를 변환하는 회전 행렬
        realsense_to_ros_rotation = np.array(
            [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]  # X -> Z  # Y -> -X  # Z -> -Y
        )

        # 변환 행렬의 분해
        rotation = transform_matrix[:3, :3]  # 3x3 회전 행렬
        translation = transform_matrix[:3, 3]  # 3x1 평행 이동 벡터

        # 좌표계 변환
        rotation_ros = realsense_to_ros_rotation @ rotation
        translation_ros = realsense_to_ros_rotation @ translation

        # 새로운 변환 행렬 구성
        transform_matrix_ros = np.eye(4)
        transform_matrix_ros[:3, :3] = rotation_ros
        transform_matrix_ros[:3, 3] = translation_ros

        return transform_matrix_ros
