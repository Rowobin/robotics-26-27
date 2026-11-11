"""LW2 public tests - PROVIDED, DO NOT EDIT.

    python3 -m pytest ~/dev_ws/src/lite6_kinematics/test -v

These check structure, units and internal consistency - not whether your
forward kinematics is right: they use it, and cannot tell. That is what
ik_check (Gazebo, then the arm) is for. They need your Labwork 1
forward_kinematics() to work. The node test needs ROS, so it runs inside the
container and is skipped elsewhere.
"""

import ast
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from lite6_kinematics import forward_kinematics as fk  # noqa: E402
from lite6_kinematics import inverse_kinematics as ik  # noqa: E402

SOURCE = os.path.join(os.path.dirname(HERE), 'lite6_kinematics', 'inverse_kinematics.py')
ALLOWED_IMPORTS = {'numpy', 'math', 'typing', '__future__', 'rclpy', 'sensor_msgs', 'geometry_msgs'}
ALLOWED_MODULES = {'lite6_kinematics.forward_kinematics'}

LOW = np.array([-3.11, -2.61, -0.06, -3.11, -2.16, -3.11])
HIGH = np.array([3.11, 2.61, 3.11, 3.11, 2.16, 3.11])
RNG = np.random.default_rng(2627)
SAMPLES = RNG.uniform(LOW, HIGH, size=(30, 6))
START_OFFSETS = RNG.uniform(-0.2, 0.2, size=(30, 6))
TOL = 1e-6


def FK(q):
    try:
        return np.asarray(fk.forward_kinematics(q), dtype=float)
    except NotImplementedError:
        pytest.fail('forward_kinematics() is not implemented: finish Labwork 1 first', pytrace=False)


def _rot(axis, angle):
    u = np.asarray(axis, dtype=float)
    u = u / np.linalg.norm(u)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _pose(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def _errors(T_c, T_d):
    """(position distance in m, orientation angle in rad) between two poses."""
    R = T_c[:3, :3].T @ T_d[:3, :3]
    s = np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
    return float(np.linalg.norm(T_c[:3, 3] - T_d[:3, 3])), float(np.arctan2(s, (np.trace(R) - 1.0) / 2.0))


# --- rules --------------------------------------------------------------------
def test_only_allowed_modules_are_imported():
    tree = ast.parse(open(SOURCE).read())
    bad = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name not in ALLOWED_MODULES and a.name.split('.')[0] not in ALLOWED_IMPORTS:
                    bad.add(a.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ''
            if module in ALLOWED_MODULES:
                continue
            if module == 'lite6_kinematics' and all(a.name == 'forward_kinematics' for a in node.names):
                continue
            if module.split('.')[0] not in ALLOWED_IMPORTS:
                bad.add(module)
        elif isinstance(node, ast.Call) and getattr(node.func, 'id', '') in ('__import__', 'exec', 'eval', 'open'):
            bad.add(node.func.id)
    assert not bad, f'not allowed in inverse_kinematics.py: {sorted(bad)}'


# --- provided code (these pass from day one) ------------------------------------------
# A regression test of the code you were given, not of your work: if one of
# these fails, something provided has been edited by mistake - restore it.
def test_provided_constants():
    assert (ik.H, ik.TOL_POS, ik.TOL_ORI, ik.MAX_ITER, ik.MAX_STEP) == (1e-6, 1e-6, 1e-6, 100, 0.2)
    assert ik.JOINT_LIMITS.shape == (6, 2) and np.all(ik.JOINT_LIMITS[:, 0] < ik.JOINT_LIMITS[:, 1])
    assert np.all(ik.JOINT_LIMITS[:, 0] <= LOW) and np.all(ik.JOINT_LIMITS[:, 1] >= HIGH)


@pytest.mark.parametrize('axis,angle', [
    ([1, 0, 0], 0.0), ([0, 1, 0], 1e-9), ([0, 0, 1], 0.5), ([1, 2, 3], 2.0),
    ([-1, 0.5, 2], math.pi - 1e-3), ([1, 1, 0], math.pi - 1e-9)])
def test_rotation_vector(axis, angle):
    u = np.asarray(axis, dtype=float)
    u /= np.linalg.norm(u)
    phi = np.asarray(ik.rotation_vector(_rot(u, angle)), dtype=float)
    assert phi.shape == (3,)
    assert np.allclose(phi, angle * u, atol=1e-7)


def test_rotation_vector_at_pi():
    # theta = pi: the axis is only defined up to its sign, the rotation is not.
    for axis in ([1, 0, 0], [0, 0, 1], [1, 1, 1], [-2, 1, 0.5]):
        R = _rot(axis, math.pi)
        phi = np.asarray(ik.rotation_vector(R), dtype=float)
        assert abs(np.linalg.norm(phi) - math.pi) < 1e-9
        assert np.allclose(_rot(phi, np.linalg.norm(phi)), R, atol=1e-9)


def test_transform_from_pose_normalises_and_orders_the_quaternion():
    from types import SimpleNamespace as NS
    R = _rot([1, 2, -1], 0.8)
    q = fk.quaternion_from_matrix(R)
    msg = NS(pose=NS(position=NS(x=0.1, y=-0.2, z=0.3),
                     orientation=NS(x=2 * q[0], y=2 * q[1], z=2 * q[2], w=2 * q[3])))
    T = np.asarray(ik.transform_from_pose(msg), dtype=float)
    assert np.allclose(T, _pose(R, [0.1, -0.2, 0.3]), atol=1e-12)


# --- pose_error ---------------------------------------------------------------------
def test_pose_error_of_equal_poses_is_zero():
    for k in range(5):
        T = _pose(_rot(RNG.normal(size=3), RNG.uniform(0, 3)), RNG.uniform(-0.5, 0.5, 3))
        e = np.asarray(ik.pose_error(T, T), dtype=float)
        assert e.shape == (6,) and np.allclose(e, 0.0, atol=1e-12)


def test_pose_error_of_a_pure_translation():
    R = _rot([1, -2, 0.5], 1.1)
    T_c = _pose(R, [0.1, 0.2, 0.3])
    T_d = _pose(R, [0.4, -0.1, 0.35])
    e = np.asarray(ik.pose_error(T_c, T_d), dtype=float)
    assert np.allclose(e, [0.3, -0.3, 0.05, 0.0, 0.0, 0.0], atol=1e-12), \
        'e = [p_d - p_c, 0, 0, 0] when the orientations are equal'


def test_pose_error_of_a_pure_rotation():
    # R_d = Rot(u, theta) R_c: the error is theta * u, expressed in {0}.
    R_c = _rot([1, -2, 0.5], 1.1)
    p = [0.1, 0.2, 0.3]
    u = np.array([0.0, 0.6, 0.8])
    T_c, T_d = _pose(R_c, p), _pose(_rot(u, 0.7) @ R_c, p)
    e = np.asarray(ik.pose_error(T_c, T_d), dtype=float)
    assert np.allclose(e[:3], 0.0, atol=1e-12)
    assert np.allclose(e[3:], 0.7 * u, atol=1e-9), \
        'the rotation part is phi(R_d R_c^T): the rotation that takes T_c to T_d, in {0}'
    assert np.allclose(np.asarray(ik.pose_error(T_d, T_c), dtype=float)[3:], -0.7 * u, atol=1e-9)


# --- jacobian ------------------------------------------------------------------------
def test_jacobian_shape():
    for q in SAMPLES[:3]:
        J = np.asarray(ik.jacobian(q), dtype=float)
        assert J.shape == (6, 6) and np.all(np.isfinite(J))


def test_jacobian_default_increment():
    q = SAMPLES[4]
    assert np.allclose(ik.jacobian(q), ik.jacobian(q, ik.H), atol=0.0)


def test_jacobian_predicts_a_small_motion():
    # J dq is the pose error the motion dq produces, to first order.
    for q, offset in zip(SAMPLES[:5], START_OFFSETS[:5]):
        dq = 1e-5 * offset / np.max(np.abs(offset))
        J = np.asarray(ik.jacobian(q), dtype=float)
        e = np.asarray(ik.pose_error(FK(q), FK(q + dq)), dtype=float)
        assert np.allclose(J @ dq, e, atol=1e-8), 'column i of J is the error caused by joint i, per radian'


# --- inverse_kinematics ----------------------------------------------------------------
def test_ik_reaches_the_pose_of_a_known_joint_vector():
    # The start is a little way from the joint vector the pose came from: the
    # method has to find a solution, not the original. 30 poses, all must work.
    for q_true, offset in zip(SAMPLES, START_OFFSETS):
        T_d = FK(q_true)
        q_start = np.clip(q_true + offset, LOW, HIGH)
        q, status, n_iter = ik.inverse_kinematics(T_d, q_start)
        assert status == 'converged', f'status {status} for the pose of {np.round(q_true, 3)}'
        q = np.asarray(q, dtype=float)
        assert q.shape == (6,) and 0 <= n_iter <= 100
        e_p, e_o = _errors(FK(q), T_d)
        assert e_p < 1.01e-6 and e_o < 1.01e-6, f'e_p {e_p:.2e} m, e_o {e_o:.2e} rad after "converged"'
        assert np.all(q >= ik.JOINT_LIMITS[:, 0]) and np.all(q <= ik.JOINT_LIMITS[:, 1])


def test_ik_from_the_solution_stops_at_once():
    q_true = SAMPLES[5]
    q, status, n_iter = ik.inverse_kinematics(FK(q_true), q_true)
    assert status == 'converged' and n_iter == 0 and np.allclose(q, q_true)


def test_ik_accepts_an_increment():
    q_true, offset = SAMPLES[6], START_OFFSETS[6]
    q, status, _ = ik.inverse_kinematics(FK(q_true), np.clip(q_true + offset, LOW, HIGH), 1e-8)
    assert status == 'converged'


def test_ik_takes_the_equivalent_angle_inside_the_range():
    # joint6 is 2 pi away from a value inside its range: same pose, so the
    # method stops at once and shifts the angle back by 2 pi.
    q_true = SAMPLES[7].copy()
    q_true[5] = 0.5
    q_start = q_true.copy()
    q_start[5] += 2 * math.pi
    q, status, n_iter = ik.inverse_kinematics(FK(q_true), q_start)
    assert status == 'converged' and n_iter == 0
    assert abs(q[5] - 0.5) < 1e-9, 'an angle outside its range is shifted by 2 pi if that brings it inside'


def test_ik_reports_a_solution_outside_the_joint_range():
    # joint2 at 2.7 rad is outside its range and no multiple of 2 pi brings it in.
    q_out = SAMPLES[8].copy()
    q_out[1] = 2.7
    q, status, _ = ik.inverse_kinematics(FK(q_out), q_out)
    assert status == 'out_of_range'


def test_ik_passes_the_increment_to_the_jacobian(monkeypatch):
    seen = []
    real = ik.jacobian

    def spy(q, h=None):
        seen.append(ik.H if h is None else h)
        return real(q, h)

    monkeypatch.setattr(ik, 'jacobian', spy)
    q_true, offset = SAMPLES[6], START_OFFSETS[6]
    q_start = np.clip(q_true + offset, LOW, HIGH)
    ik.inverse_kinematics(FK(q_true), q_start, 1e-8)
    assert seen and all(h == 1e-8 for h in seen), 'inverse_kinematics must pass h on to jacobian()'
    seen.clear()
    ik.inverse_kinematics(FK(q_true), q_start)
    assert seen and all(h == ik.H for h in seen), 'the default h is H'


def test_ik_limits_the_step_by_scaling_it(monkeypatch):
    # With J = I the step is the error itself: 0.5 rad on joint 1 is cut to
    # MAX_STEP, and the others are cut by the same factor (the direction is kept).
    e = np.array([0.5, 0.25, 0.1, 0.0, 0.0, 0.0])
    monkeypatch.setattr(ik, 'pose_error', lambda T_c, T_d: e)
    monkeypatch.setattr(ik, 'jacobian', lambda q, h=None: np.eye(6))
    monkeypatch.setattr(ik, 'MAX_ITER', 1)
    q0 = SAMPLES[12]
    q, status, n_iter = ik.inverse_kinematics(np.eye(4), q0)
    assert status == 'max_iterations' and n_iter == 1
    assert np.allclose(np.asarray(q, dtype=float) - q0, [0.2, 0.1, 0.04, 0.0, 0.0, 0.0], atol=1e-12), \
        'if the largest change exceeds MAX_STEP, multiply the whole step by MAX_STEP / largest'


def test_ik_reports_a_singular_jacobian(monkeypatch):
    monkeypatch.setattr(ik, 'jacobian', lambda q, h=None: np.zeros((6, 6)))
    q, status, n_iter = ik.inverse_kinematics(FK(SAMPLES[13]), SAMPLES[14])
    assert status == 'singular' and n_iter == 0
    assert np.allclose(q, SAMPLES[14])


def test_ik_does_not_converge_for_a_pose_out_of_reach():
    T_d = _pose(FK(SAMPLES[9])[:3, :3], [2.0, 0.0, 0.3])        # the arm reaches 0.44 m
    for q_start in (np.zeros(6), SAMPLES[10]):
        q, status, n_iter = ik.inverse_kinematics(T_d, q_start)
        assert status in ('max_iterations', 'singular', 'out_of_range')
        assert status != 'converged' and 0 <= n_iter <= 100
        assert np.asarray(q, dtype=float).shape == (6,)


# --- the node (needs ROS) -----------------------------------------------------------------
def test_node_publishes_the_solution_for_the_target(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    prefix = f'/lw2_test_{os.getpid()}'
    q_true = SAMPLES[11]
    q_start = np.clip(q_true + START_OFFSETS[11], LOW, HIGH)
    order = [3, 0, 5, 1, 4, 2]                  # shuffled, plus a joint that is not ours
    state = JointState()
    state.name = [f'joint{i + 1}' for i in order] + ['gripper']
    state.position = [float(q_start[i]) for i in order] + [0.5]

    def target(stamp, T):
        msg = PoseStamped()
        msg.header.frame_id = 'link_base'
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
        x, y, z, w = fk.quaternion_from_matrix(T[:3, :3])
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        return msg

    starts = []                                 # what the node solves from
    real_solver = ik.inverse_kinematics

    def spy(T_d, q_from, *args, **kwargs):
        starts.append(np.asarray(q_from, dtype=float).copy())
        return real_solver(T_d, q_from, *args, **kwargs)

    monkeypatch.setattr(ik, 'inverse_kinematics', spy)

    T_ok = FK(q_true)
    T_far = _pose(T_ok[:3, :3], [2.0, 0.0, 0.3])
    good = target((123, 456789), T_ok)
    far = target((124, 1), T_far)

    rclpy.init(args=[])
    got = []
    try:
        node = ik.IkNode(target_topic=prefix + '/target', joint_states_topic=prefix + '/joint_states',
                         solution_topic=prefix + '/solution')
        helper = Node('lw2_test_helper')
        helper.create_subscription(JointState, prefix + '/solution', got.append, 10)
        states = helper.create_publisher(JointState, prefix + '/joint_states', 10)
        targets = helper.create_publisher(PoseStamped, prefix + '/target', 10)

        def spin_for(seconds, stop=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not stop():
                states.publish(state)
                rclpy.spin_once(node, timeout_sec=0.02)
                rclpy.spin_once(helper, timeout_sec=0.02)

        end = time.monotonic() + 10.0
        while not got and time.monotonic() < end:       # until discovery and the first joint state are done
            targets.publish(good)
            spin_for(0.1, lambda: bool(got))
        n_good = len(got)
        for _ in range(10):                             # an out-of-reach pose: no answer
            targets.publish(far)
            spin_for(0.2)
        node.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()

    assert n_good, 'the node published nothing on solution_topic'
    stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in got}
    assert (124, 1) not in stamps, 'the node published a solution for a pose out of reach'
    assert stamps == {(123, 456789)}, 'header.stamp must be the stamp of the target'
    msg = got[0]
    index = {n: i for i, n in enumerate(msg.name)}
    assert all(f'joint{i}' in index for i in range(1, 7)), 'name must hold joint1 ... joint6'
    q = np.array([msg.position[index[f'joint{i}']] for i in range(1, 7)])
    e_p, e_o = _errors(FK(q), T_ok)
    assert e_p < 1.01e-6 and e_o < 1.01e-6, 'the solution must reach the desired pose'
    assert starts and all(np.allclose(st, q_start, atol=1e-12) for st in starts), \
        'the node must solve from the latest joint angles, picked by name from the JointState'
