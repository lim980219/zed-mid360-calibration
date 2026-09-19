import numpy as np
from scipy.spatial.transform import Rotation


def mount_transform(forward, left, down, pitch_down_deg=10.0, yaw_left_deg=0.0, roll_deg=0.0):
    """T_lidar_camera: camera optical (right/down/front) -> lidar FLU."""
    optical_to_body = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("ZYX", [yaw_left_deg, pitch_down_deg, roll_deg], degrees=True).as_matrix() @ optical_to_body
    result[:3, 3] = [forward, left, -down]
    return result


def pose_vector(matrix):
    return [*matrix[:3, 3].tolist(), *Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()]


def from_pose(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("Backend pose must contain 7 finite values: x y z qx qy qz qw")
    if abs(np.linalg.norm(values[3:]) - 1) > 0.01:
        raise ValueError("Backend returned a non-unit quaternion")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def project(points, transform, intrinsic, width, height):
    camera = points[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > 0.1)
    ids = np.flatnonzero(valid)
    pixels = camera[valid] @ intrinsic.T
    pixels = pixels[:, :2] / pixels[:, 2:3]
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return pixels[inside], camera[ids[inside], 2], ids[inside]
