"""fk_check - LW1 forward-kinematics checker. PROVIDED, DO NOT EDIT.

Moves the Lite 6 through the poses in config/lw1_poses.yaml and, at each one,
compares the pose that YOUR fk_node publishes on /lite6_fk/pose with
independent references:

    tf          link_base -> link_eef, from robot_state_publisher   (sim, real)
    controller  the pose the arm's own controller reports,          (real only)
                /ufactory/robot_states, TCP offset removed

The pose is matched to the joint angles it was computed from by its
header.stamp, which must be the stamp of the JointState your node received.

Terminal 1:   lab sim                         |  lab real <robot_ip>
Terminal 2:   ros2 run lite6_kinematics fk_node
Terminal 3:   ros2 run lite6_kinematics fk_check --ros-args -p robot:=sim
              ros2 run lite6_kinematics fk_check --ros-args -p robot:=real

Results go to ~/shared/lw1_fk_<robot>.csv (= ./shared/ on your machine).

With -p once:=true it does not move the arm: it compares once, at whatever
configuration the arm is in, and appends to ~/shared/lw1_fk_<robot>_once.csv.
"""

import csv
import math
import os
import sys
import time
from collections import OrderedDict

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from lite6_kinematics.checking import (controller_eef_pose, is_homogeneous,
                                       pose_errors, pose_stamped_to_matrix,
                                       transform_stamped_to_matrix)
from lite6_kinematics.forward_kinematics import pose_from_transform

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class FkCheck(Node):

    def __init__(self):
        super().__init__('fk_check')
        self.declare_parameter('robot', 'sim')
        self.declare_parameter('once', False)
        self.declare_parameter('output_dir', '~/shared')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('eef_frame', 'link_eef')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('pose_topic', '/lite6_fk/pose')
        self.declare_parameter('robot_states_topic', '/ufactory/robot_states')
        self.declare_parameter('action', '/lite6_traj_controller/follow_joint_trajectory')
        self.declare_parameter('max_speed', 0.35)      # rad/s, slowest joint sets the time
        self.declare_parameter('settle_s', 1.5)        # wait after each motion

        self.robot = self.get_parameter('robot').value
        if self.robot not in ('sim', 'real'):
            raise ValueError("parameter robot must be 'sim' or 'real'")
        self.once = bool(self.get_parameter('once').value)
        self.base = self.get_parameter('base_frame').value
        self.eef = self.get_parameter('eef_frame').value

        self._q = None
        self._robot_msg = None
        self._pose = None
        self._states = OrderedDict()        # recent joint states, by header.stamp
        # Deep queue: joint states can arrive at hundreds of Hz, and the one that
        # matches your pose by stamp must not be dropped.
        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value,
                                 self._on_joint_state, 1000)
        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value,
                                 self._on_pose, 10)
        if self.robot == 'real':
            from xarm_msgs.msg import RobotMsg
            self.create_subscription(RobotMsg, self.get_parameter('robot_states_topic').value,
                                     self._on_robot_msg, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action = ActionClient(self, FollowJointTrajectory,
                                   self.get_parameter('action').value)

    # --- callbacks: store only ----------------------------------------------
    def _on_joint_state(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if all(n in index for n in JOINT_NAMES):
            self._q = np.array([msg.position[index[n]] for n in JOINT_NAMES])
            self._states[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = self._q
            while len(self._states) > 500:
                self._states.popitem(last=False)

    def _on_pose(self, msg):
        self._pose = msg

    def _on_robot_msg(self, msg):
        self._robot_msg = msg

    # --- plumbing --------------------------------------------------------------
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, condition, timeout, what):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                raise RuntimeError(f'timed out waiting for {what}')
            rclpy.spin_once(self, timeout_sec=0.05)

    def lookup_tf(self):
        ts = self.tf_buffer.lookup_transform(self.base, self.eef, Time())
        return transform_stamped_to_matrix(ts)

    # --- motion ---------------------------------------------------------------
    def move_to(self, q_target):
        q_now = self._q.copy()
        span = float(np.max(np.abs(q_target - q_now)))
        duration = max(3.0, span / float(self.get_parameter('max_speed').value))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q_target]
        point.velocities = [0.0] * 6
        sec = int(duration)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))
        goal.trajectory.points = [point]

        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not rclpy.ok():
            raise KeyboardInterrupt     # Ctrl+C while waiting: quit, do not report a timeout
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('trajectory goal rejected by the controller')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 15.0)
        if not rclpy.ok():
            raise KeyboardInterrupt
        if not result_future.done():
            raise RuntimeError('trajectory did not finish in time')
        code = result_future.result().result.error_code
        if code != 0:
            # Measured anyway: the comparison uses the joint angles the arm
            # actually reached, not the ones requested.
            self.get_logger().warn(f'controller reported error_code={code}; measuring the reached pose')

    # --- measurement ----------------------------------------------------------
    def matched_pose(self):
        """The latest pose from your node, and the joint angles it was computed from."""
        self.spin_for(0.3)                  # let the latest messages in
        topic = self.get_parameter('pose_topic').value
        end = time.monotonic() + 3.0
        while True:
            msg = self._pose
            if msg.header.frame_id != self.base:
                raise RuntimeError(f"the pose on {topic} has header.frame_id '{msg.header.frame_id}', "
                                   f"expected '{self.base}'")
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if key in self._states:
                return self._states[key].copy(), msg
            if time.monotonic() > end:
                raise RuntimeError(f'the pose on {topic} has a header.stamp that is not the stamp of a '
                                   'recent /joint_states message: copy msg.header.stamp from the JointState '
                                   'into the PoseStamped, and keep fk_node running')
            rclpy.spin_once(self, timeout_sec=0.05)

    def measure(self, name):
        q, pose_msg = self.matched_pose()
        T_student = pose_stamped_to_matrix(pose_msg)
        if not is_homogeneous(T_student):
            raise RuntimeError('the pose from your node is not a valid rigid-body pose '
                               '(is the quaternion normalised, in the order x, y, z, w?)')
        pose = pose_from_transform(T_student)
        row = {'pose': name, 'robot': self.robot, 'time': time.strftime('%Y-%m-%dT%H:%M:%S')}
        row.update({f'q{i + 1}': q[i] for i in range(6)})
        row.update(_flat('student', T_student))
        T_tf = self.lookup_tf()
        row.update(_flat('tf', T_tf))
        e_p, e_o = pose_errors(T_student, T_tf)
        row['ep_tf_mm'], row['eo_tf_deg'] = e_p * 1e3, math.degrees(e_o)
        if self.robot == 'real':
            msg = self._robot_msg
            T_ctrl = controller_eef_pose(msg.pose, msg.offset)
            row.update(_flat('ctrl', T_ctrl))
            row['tcp_offset'] = ' '.join(f'{float(v):.4f}' for v in msg.offset)
            e_p, e_o = pose_errors(T_student, T_ctrl)
            row['ep_ctrl_mm'], row['eo_ctrl_deg'] = e_p * 1e3, math.degrees(e_o)
        self._print_row(row, pose)
        return row

    def _print_row(self, row, pose):
        xyz = ' '.join(f'{1e3 * v:8.2f}' for v in pose[:3])
        rpy = ' '.join(f'{math.degrees(v):8.2f}' for v in pose[3:])
        line = (f"{row['pose']:<12} your pose [mm] {xyz}  [deg] {rpy}  |  "
                f"vs tf: {row['ep_tf_mm']:7.3f} mm {row['eo_tf_deg']:7.4f} deg")
        if 'ep_ctrl_mm' in row:
            line += f"  |  vs controller: {row['ep_ctrl_mm']:7.3f} mm {row['eo_ctrl_deg']:7.4f} deg"
        print(line, flush=True)

    # --- the procedure ----------------------------------------------------------
    def run(self):
        self.wait_until(lambda: self._q is not None, 15.0,
                        f'/joint_states with {JOINT_NAMES} - is lab {self.robot} running?')
        # Before anything moves: is your node publishing, and is the pose usable?
        # (The arm never leaves where it is if it is not.)
        self.wait_until(lambda: self._pose is not None, 15.0,
                        f"a pose on {self.get_parameter('pose_topic').value} - "
                        'is your node running? (ros2 run lite6_kinematics fk_node)')
        self.matched_pose()
        self.wait_until(lambda: self.tf_buffer.can_transform(self.base, self.eef, Time()), 15.0,
                        f'tf {self.base} -> {self.eef}')
        if self.robot == 'real':
            self.wait_until(lambda: self._robot_msg is not None, 15.0, '/ufactory/robot_states')
            if any(abs(float(v)) > 1e-6 for v in self._robot_msg.offset):
                print('note: the controller has a TCP offset set '
                      f'({[round(float(v), 4) for v in self._robot_msg.offset]}); '
                      'it is removed before comparing.', flush=True)

        rows = []
        if self.once:
            self.spin_for(0.5)
            rows.append(self.measure('once'))
        else:
            poses = _load_poses()
            self.wait_until(lambda: self.action.server_is_ready(), 15.0,
                            'the trajectory controller action server')
            for p in poses:
                if self.robot == 'real':
                    input(f"\nNext: {p['name']} {p['q_deg']} deg. "
                          'Workspace clear, hand on the e-stop? Enter to move, Ctrl+C to quit. '
                          '(Ctrl+C does not stop a motion in progress: the e-stop does.) ')
                self.move_to(np.radians(np.asarray(p['q_deg'], dtype=float)))
                self.spin_for(float(self.get_parameter('settle_s').value))
                rows.append(self.measure(p['name']))
        path = self._write(rows)
        _summary(rows)
        print(f'\nwritten: {path}', flush=True)

    def _write(self, rows):
        out = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out, exist_ok=True)
        suffix = '_once' if self.once else ''
        path = os.path.join(out, f'lw1_fk_{self.robot}{suffix}.csv')
        fields = list(rows[0].keys())
        append = self.once and os.path.exists(path)
        with open(path, 'a' if append else 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            if not append:
                w.writeheader()
            w.writerows(rows)
        return path


def _flat(prefix, T):
    """x, y, z and the nine rotation entries: unambiguous, no angle convention."""
    d = {f'{prefix}_x': T[0, 3], f'{prefix}_y': T[1, 3], f'{prefix}_z': T[2, 3]}
    for i in range(3):
        for j in range(3):
            d[f'{prefix}_r{i + 1}{j + 1}'] = T[i, j]
    return d


def _load_poses():
    path = os.path.join(get_package_share_directory('lite6_kinematics'), 'config', 'lw1_poses.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['poses']


def _summary(rows):
    print('\nsummary', flush=True)
    for key, label in (('tf', 'vs tf (model)     '), ('ctrl', 'vs controller     ')):
        if f'ep_{key}_mm' not in rows[0]:
            continue
        ep = np.array([r[f'ep_{key}_mm'] for r in rows])
        eo = np.array([r[f'eo_{key}_deg'] for r in rows])
        print(f'  {label} e_p max {ep.max():.3f} mm, mean {ep.mean():.3f} mm  |  '
              f'e_o max {eo.max():.4f} deg', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = FkCheck()
    status = 0
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Quitting does not cancel a goal already sent: the arm finishes that
        # motion. Only the emergency stop halts it.
        print('\nstopped. A motion already in progress is not cancelled: '
              'it runs to its end; on the real arm only the emergency stop halts it.',
              file=sys.stderr)
        status = 130
    except RuntimeError as e:
        print(f'\nfk_check: {e}', file=sys.stderr)
        status = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
