"""ik_check - LW2 inverse-kinematics checker. PROVIDED, DO NOT EDIT.

For each desired pose in config/lw2_targets.yaml it:

    1. commands the arm, through the trajectory controller, to the starting
       configuration (S0 unless -p start:=Sn) and records the joint angles it
       actually reached;
    2. publishes the desired pose on /lite6_ik/target and waits for YOUR node's
       answer on /lite6_ik/solution (5 s);
    3. if a solution arrives, commands the arm to it and compares the pose it
       reaches with the desired pose. The references are independent of your
       code:

           tf          link_base -> link_eef, from robot_state_publisher  (sim, real)
           controller  the pose the arm's own controller reports,         (real only)
                       /ufactory/robot_states, TCP offset removed

    4. if no solution arrives, records the pose as refused and does not move
       the arm.

The solution is matched to the target by its header.stamp, which must be the
stamp of the PoseStamped your node answered.

Terminal 1:   lab sim                         |  lab real <robot_ip>
Terminal 2:   ros2 run lite6_kinematics ik_node
Terminal 3:   ros2 run lite6_kinematics ik_check --ros-args -p robot:=sim
              ros2 run lite6_kinematics ik_check --ros-args -p robot:=real

Results go to ~/shared/lw2_ik_<robot>.csv (= ./shared/ on your machine).

    -p target:=A      only that target
    -p start:=S1      start from S1 instead of S0
                      (with either of the two, the file is lw2_ik_<robot>_<target>_<start>.csv)
    -p once:=true     the desired pose is the pose the end-effector has now (read
                      from tf); the arm goes to the start, your node solves, the
                      arm executes the solution. Appends to lw2_ik_<robot>_once.csv.
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

from lite6_kinematics.checking import (controller_eef_pose, pose_errors,
                                       transform_stamped_to_matrix, xyzrpy_to_matrix)
from lite6_kinematics.forward_kinematics import pose_from_transform, quaternion_from_matrix

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Joint ranges of the xarm_description model (rad). A solution outside them is
# never executed.
LOWER = np.array([-math.pi * 0.99, -2.61799, -0.061087, -math.pi * 0.99, -2.1642, -math.pi * 0.99])
UPPER = np.array([math.pi * 0.99, 2.61799, math.pi * 0.99, math.pi * 0.99, 2.1642, math.pi * 0.99])


class IkCheck(Node):

    def __init__(self):
        super().__init__('ik_check')
        self.declare_parameter('robot', 'sim')
        self.declare_parameter('once', False)
        self.declare_parameter('target', '')
        self.declare_parameter('start', 'S0')
        self.declare_parameter('output_dir', '~/shared')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('eef_frame', 'link_eef')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_topic', '/lite6_ik/target')
        self.declare_parameter('solution_topic', '/lite6_ik/solution')
        self.declare_parameter('robot_states_topic', '/ufactory/robot_states')
        self.declare_parameter('action', '/lite6_traj_controller/follow_joint_trajectory')
        self.declare_parameter('max_speed', 0.35)      # rad/s, slowest joint sets the time
        self.declare_parameter('settle_s', 1.5)        # wait after each motion
        self.declare_parameter('wait_s', 5.0)          # wait for the solution

        self.robot = self.get_parameter('robot').value
        if self.robot not in ('sim', 'real'):
            raise ValueError("parameter robot must be 'sim' or 'real'")
        self.once = bool(self.get_parameter('once').value)
        self.only = self.get_parameter('target').value
        self.start_name = self.get_parameter('start').value
        self.base = self.get_parameter('base_frame').value
        self.eef = self.get_parameter('eef_frame').value
        self.target_topic = self.get_parameter('target_topic').value

        self._q = None
        self._robot_msg = None
        self._solutions = OrderedDict()     # recent solutions, by header.stamp
        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value,
                                 self._on_joint_state, 1000)
        self.create_subscription(JointState, self.get_parameter('solution_topic').value,
                                 self._on_solution, 10)
        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
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

    def _on_solution(self, msg):
        self._solutions[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        while len(self._solutions) > 50:
            self._solutions.popitem(last=False)

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

    def confirm(self, text):
        """On the real arm, nothing moves without the operator's Enter."""
        if self.robot == 'real':
            input(f'\n{text} Workspace clear, hand on the e-stop? Enter to move, Ctrl+C to quit. '
                  '(Ctrl+C does not stop a motion in progress: the e-stop does.) ')

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
            # Measured anyway: the comparison uses the pose the arm actually reached.
            self.get_logger().warn(f'controller reported error_code={code}; measuring the reached pose')
        self.spin_for(float(self.get_parameter('settle_s').value))

    # --- the exchange with your node ------------------------------------------------
    def send_target(self, T_d):
        """Publish T_d and wait for the solution with the same stamp, or None."""
        msg = PoseStamped()
        msg.header.frame_id = self.base
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T_d[:3, 3])
        x, y, z, w = quaternion_from_matrix(T_d[:3, :3])
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        t0 = time.monotonic()
        self.target_pub.publish(msg)
        end = t0 + float(self.get_parameter('wait_s').value)
        while key not in self._solutions and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        solved_s = time.monotonic() - t0
        return self._solutions.get(key), solved_s

    # --- the procedure ----------------------------------------------------------
    def run(self):
        targets, starts = _load_config()
        if self.start_name not in starts:
            raise RuntimeError(f"unknown start '{self.start_name}'; the starts are {sorted(starts)}")
        q_start = np.radians(np.asarray(starts[self.start_name], dtype=float))

        self.wait_until(lambda: self._q is not None, 15.0,
                        f'/joint_states with {JOINT_NAMES} - is lab {self.robot} running?')
        # Before anything moves: is your node listening?
        # (The arm never leaves where it is if it is not.)
        self.wait_until(lambda: self.count_subscribers(self.target_topic) > 0, 15.0,
                        f'a node listening on {self.target_topic} - '
                        'is your node running? (ros2 run lite6_kinematics ik_node)')
        self.wait_until(lambda: self.tf_buffer.can_transform(self.base, self.eef, Time()), 15.0,
                        f'tf {self.base} -> {self.eef}')
        if self.robot == 'real':
            self.wait_until(lambda: self._robot_msg is not None, 15.0, '/ufactory/robot_states')
            if any(abs(float(v)) > 1e-6 for v in self._robot_msg.offset):
                print('note: the controller has a TCP offset set '
                      f'({[round(float(v), 4) for v in self._robot_msg.offset]}); '
                      'it is removed before comparing.', flush=True)
        self.wait_until(lambda: self.action.server_is_ready(), 15.0,
                        'the trajectory controller action server')

        if self.once:
            self.spin_for(0.5)
            targets = [{'name': 'once', 'T': self.lookup_tf()}]
        elif self.only:
            targets = [t for t in targets if t['name'] == self.only]
            if not targets:
                raise RuntimeError(f"unknown target '{self.only}'")

        rows = []
        for t in targets:
            rows.append(self.check(t['name'], t['T'], q_start))
        path = self._write(rows)
        _summary(rows)
        print(f'\nwritten: {path}', flush=True)

    def check(self, name, T_d, q_start):
        self.confirm(f"Next: {name} - move to {self.start_name} "
                     f"{[round(math.degrees(v), 1) for v in q_start]} deg.")
        self.move_to(q_start)
        self.spin_for(0.3)
        q_reached = self._q.copy()
        row = {'pose': name, 'robot': self.robot, 'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
               'start': self.start_name}
        row.update({f'q_start{i + 1}': q_reached[i] for i in range(6)})
        row.update(_flat('target', T_d))

        sol, solved_s = self.send_target(T_d)
        row['solve_s'] = solved_s
        if sol is None:
            row['result'] = 'refused'
            self._print_row(row, T_d)
            return row
        index = {n: i for i, n in enumerate(sol.name)}
        if not all(n in index for n in JOINT_NAMES):
            row['result'] = 'rejected'
            self._print_row(row, T_d, f'the solution lacks one of {JOINT_NAMES}: not executed')
            return row
        q_sol = np.array([sol.position[index[n]] for n in JOINT_NAMES], dtype=float)
        row.update({f'q_sol{i + 1}': q_sol[i] for i in range(6)})
        if not (np.all(np.isfinite(q_sol)) and np.all(q_sol >= LOWER) and np.all(q_sol <= UPPER)):
            row['result'] = 'rejected'
            self._print_row(row, T_d, 'the solution is not finite or is outside the joint ranges: '
                                      'not executed')
            return row

        self.confirm(f"{name} - move to the solution "
                     f"{[round(math.degrees(v), 1) for v in q_sol]} deg.")
        self.move_to(q_sol)
        self.spin_for(0.3)
        row['result'] = 'reached'
        row.update({f'q_final{i + 1}': self._q[i] for i in range(6)})
        T_tf = self.lookup_tf()
        row.update(_flat('tf', T_tf))
        e_p, e_o = pose_errors(T_tf, T_d)
        row['ep_tf_mm'], row['eo_tf_deg'] = e_p * 1e3, math.degrees(e_o)
        if self.robot == 'real':
            msg = self._robot_msg
            T_ctrl = controller_eef_pose(msg.pose, msg.offset)
            row.update(_flat('ctrl', T_ctrl))
            row['tcp_offset'] = ' '.join(f'{float(v):.4f}' for v in msg.offset)
            e_p, e_o = pose_errors(T_ctrl, T_d)
            row['ep_ctrl_mm'], row['eo_ctrl_deg'] = e_p * 1e3, math.degrees(e_o)
        self._print_row(row, T_d)
        return row

    def _print_row(self, row, T_d, note=''):
        pose = pose_from_transform(T_d)
        # + 0.0: a rounding residue such as -1e-17 prints as 0.0, not -0.0
        xyz = ' '.join(f'{round(1e3 * v, 1) + 0.0:7.1f}' for v in pose[:3])
        rpy = ' '.join(f'{round(math.degrees(v), 1) + 0.0:7.1f}' for v in pose[3:])
        line = f"{row['pose']:<6} target [mm] {xyz}  [deg] {rpy}  |  {row['result']:<8}"
        if row['result'] == 'refused':
            line += f"  no solution in {row['solve_s']:.1f} s: the arm was not moved"
        elif row['result'] == 'reached':
            line += (f"  vs tf: {row['ep_tf_mm']:7.3f} mm {row['eo_tf_deg']:7.4f} deg")
            if 'ep_ctrl_mm' in row:
                line += f"  |  vs controller: {row['ep_ctrl_mm']:7.3f} mm {row['eo_ctrl_deg']:7.4f} deg"
        elif note:
            line += f'  {note}'
        print(line, flush=True)

    def _write(self, rows):
        out = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out, exist_ok=True)
        if self.once:
            suffix = '_once'
        elif self.only or self.start_name != 'S0':
            suffix = f"_{self.only or 'all'}_{self.start_name}"
        else:
            suffix = ''
        path = os.path.join(out, f'lw2_ik_{self.robot}{suffix}.csv')
        append = self.once and os.path.exists(path)
        with open(path, 'a' if append else 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=_fields(self.robot), restval='')
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


def _fields(robot):
    """The columns of the CSV file, in order; refused poses leave the later ones empty."""
    def flat(prefix):
        return ([f'{prefix}_{c}' for c in 'xyz']
                + [f'{prefix}_r{i}{j}' for i in (1, 2, 3) for j in (1, 2, 3)])
    fields = ['pose', 'robot', 'time', 'start', 'result']
    fields += [f'q_start{i}' for i in range(1, 7)] + flat('target')
    fields += ['solve_s'] + [f'q_sol{i}' for i in range(1, 7)] + [f'q_final{i}' for i in range(1, 7)]
    fields += flat('tf') + ['ep_tf_mm', 'eo_tf_deg']
    if robot == 'real':
        fields += flat('ctrl') + ['tcp_offset', 'ep_ctrl_mm', 'eo_ctrl_deg']
    return fields


def _load_config():
    """(targets, starts) from config/lw2_targets.yaml; each target has its 4x4 'T'."""
    path = os.path.join(get_package_share_directory('lite6_kinematics'), 'config', 'lw2_targets.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    targets = []
    for t in cfg['targets']:
        x, y, z = (v / 1000.0 for v in t['xyz_mm'])
        roll, pitch, yaw = (math.radians(v) for v in t['rpy_deg'])
        targets.append({'name': t['name'], 'T': xyzrpy_to_matrix(x, y, z, roll, pitch, yaw)})
    return targets, cfg['starts']


def _summary(rows):
    print('\nsummary', flush=True)
    reached = [r for r in rows if r['result'] == 'reached']
    print(f"  reached {len(reached)}, refused {sum(r['result'] == 'refused' for r in rows)}, "
          f"rejected {sum(r['result'] == 'rejected' for r in rows)} of {len(rows)}", flush=True)
    for key, label in (('tf', 'vs tf (model)     '), ('ctrl', 'vs controller     ')):
        if not reached or f'ep_{key}_mm' not in reached[0]:
            continue
        ep = np.array([r[f'ep_{key}_mm'] for r in reached])
        eo = np.array([r[f'eo_{key}_deg'] for r in reached])
        print(f'  {label} e_p max {ep.max():.3f} mm, mean {ep.mean():.3f} mm  |  '
              f'e_o max {eo.max():.4f} deg', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = IkCheck()
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
        print(f'\nik_check: {e}', file=sys.stderr)
        status = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
