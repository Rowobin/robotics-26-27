"""Helpers for fk_check - PROVIDED, DO NOT EDIT.

Everything the checker needs that is not part of your work: turning the
references (a tf transform, the controller's reported pose) into 4x4 matrices,
and measuring the distance between two poses. Your forward_kinematics.py does
not use, and may not import, anything from here.
"""

import numpy as np


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def xyzrpy_to_matrix(x, y, z, roll, pitch, yaw) -> np.ndarray:
    """4x4 from position + X-Y-Z fixed angles, R = Rz(yaw) Ry(pitch) Rx(roll).

    The Lite 6 controller's convention (xArm Developer Manual: roll/pitch/yaw
    rotate in turn about the X/Y/Z axes of the fixed reference frame).
    """
    T = np.eye(4)
    T[:3, :3] = _rz(yaw) @ _ry(pitch) @ _rx(roll)
    T[:3, 3] = (x, y, z)
    return T


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    """3x3 rotation from a unit quaternion (ROS order x, y, z, w)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def transform_stamped_to_matrix(ts) -> np.ndarray:
    """4x4 from a geometry_msgs/TransformStamped."""
    t, r = ts.transform.translation, ts.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(r.x, r.y, r.z, r.w)
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def pose_stamped_to_matrix(ps) -> np.ndarray:
    """4x4 from a geometry_msgs/PoseStamped."""
    p, o = ps.pose.position, ps.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(o.x, o.y, o.z, o.w)
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def controller_eef_pose(pose, tcp_offset) -> np.ndarray:
    """^0_6 T from an xarm_msgs/RobotMsg.

    RobotMsg.pose is the TCP pose [x, y, z (mm), roll, pitch, yaw (rad)]; the
    TCP sits at RobotMsg.offset from the end-effector frame {6}, expressed in
    that frame. Removing the offset gives the pose of {6} itself, which is what
    forward_kinematics returns.
    """
    p = [float(v) for v in pose]
    o = [float(v) for v in tcp_offset]
    T_tcp = xyzrpy_to_matrix(p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0, p[3], p[4], p[5])
    T_off = xyzrpy_to_matrix(o[0] / 1000.0, o[1] / 1000.0, o[2] / 1000.0, o[3], o[4], o[5])
    return T_tcp @ np.linalg.inv(T_off)


def pose_errors(T_c, T_m):
    """(e_p in metres, e_o in radians) between two 4x4 poses.

    e_p = || p_c - p_m ||,   e_o = angle of R_c^T R_m.
    """
    T_c, T_m = np.asarray(T_c, float), np.asarray(T_m, float)
    e_p = float(np.linalg.norm(T_c[:3, 3] - T_m[:3, 3]))
    R = T_c[:3, :3].T @ T_m[:3, :3]
    # Same angle as arccos((tr R - 1) / 2), computed as atan2(sin, cos) so it
    # stays accurate for very small errors.
    s = np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
    c = (np.trace(R) - 1.0) / 2.0
    return e_p, float(np.arctan2(s, c))


def is_homogeneous(T, tol=1e-9) -> bool:
    """True if T is a 4x4 rigid-body transform."""
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return False
    R = T[:3, :3]
    return (np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=tol)
            and np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            and abs(np.linalg.det(R) - 1.0) < 1e-6)
