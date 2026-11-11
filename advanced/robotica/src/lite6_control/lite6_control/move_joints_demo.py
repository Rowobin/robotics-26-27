"""Complete, self-contained example: read joint states, command joint positions.

Everything is in this one file on purpose. `lite6_client.py` is the same logic
factored out for reuse, but if you are learning the API, read this top to bottom
first - there is nothing hidden.

What it demonstrates
--------------------
* subscribing to ``/ufactory/joint_states``  (continuous, the ROS-native way)
* calling ``/ufactory/get_servo_angle``      (on demand, straight from the box)
* the enable sequence: motion_enable -> set_mode(0) -> set_state(0)
* commanding absolute joint positions with ``/ufactory/set_servo_angle``
* checking ``ret`` on every call, because ROS success != robot success
* measuring the tracking error: where you asked for vs where it went

Run it
------
    # driver must own the arm (NOT the MoveIt realmove stack):
    lab driver 192.168.1.181

    ros2 run lite6_control move_joints_demo --dry-run     # moves nothing
    ros2 run lite6_control move_joints_demo               # 3 waypoints
    ros2 run lite6_control move_joints_demo --amplitude 0.1 --speed 0.15

SAFETY
------
This moves a real arm. Emergency stop within reach, nobody in the workspace.
Every waypoint is a small offset from wherever the arm is when you start, so it
never makes a large unplanned move - but check the space around it anyway.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import GetFloat32List, MoveJoint, SetInt16, SetInt16ById

JOINT_COUNT = 6
MODE_POSITION = 0

# set_state() and the REPORTED state use different value spaces:
#   pass to set_state():  0 = motion, 3 = pause, 4 = stop
#   robot_states reports: 1 = in motion, 2 = sleeping (idle+ready),
#                         3 = suspended, 4 = stopping
# You send 0; it then reports 2. A reported 0 never happens.
SET_STATE_MOTION = 0
READY_STATES = (1, 2)


class RobotRefused(RuntimeError):
    """The arm answered, but said no."""


class JointMover(Node):
    """Reads joint states and commands joint positions on one Lite 6."""

    def __init__(self, namespace: str = 'ufactory'):
        super().__init__('move_joints_demo')
        ns = namespace.strip('/')
        self.ns = ns

        # ---- reading: a subscription, updated continuously in the background.
        self._joint_state: Optional[JointState] = None
        self._robot_state: Optional[RobotMsg] = None
        self.create_subscription(JointState, f'/{ns}/joint_states',
                                 self._on_joint_state, 10)
        self.create_subscription(RobotMsg, f'/{ns}/robot_states',
                                 self._on_robot_state, 10)

        # ---- commanding: service clients. Creating one is cheap and does not
        #      connect to anything yet; wait_for_service() does that.
        self.cli_enable = self.create_client(SetInt16ById, f'/{ns}/motion_enable')
        self.cli_mode = self.create_client(SetInt16, f'/{ns}/set_mode')
        self.cli_state = self.create_client(SetInt16, f'/{ns}/set_state')
        self.cli_move = self.create_client(MoveJoint, f'/{ns}/set_servo_angle')
        self.cli_get = self.create_client(GetFloat32List, f'/{ns}/get_servo_angle')

    # ------------------------------------------------------------------ #
    # callbacks - these just store the newest message and return quickly.
    # Never do slow work or blocking service calls in here.
    # ------------------------------------------------------------------ #

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_robot_state(self, msg: RobotMsg) -> None:
        self._robot_state = msg

    # ------------------------------------------------------------------ #
    # the blocking call helper
    # ------------------------------------------------------------------ #

    def call(self, client, request, label: str, timeout: float = 30.0):
        """Send a request, wait for the reply, raise unless ret == 0.

        Called from main(), never from a callback: it spins the node to wait
        for the response, and spinning from inside a callback deadlocks.
        """
        if not client.wait_for_service(timeout_sec=5.0):
            raise RobotRefused(
                f'{label}: service not available. Is the driver running? '
                f'  lab driver <robot_ip>')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RobotRefused(f'{label}: no reply within {timeout:.0f}s')

        response = future.result()
        # THIS is the check people forget. The service call above succeeded at
        # the ROS level no matter what the arm thought of it.
        if getattr(response, 'ret', 0) != 0:
            raise RobotRefused(
                f'{label}: robot returned ret={response.ret} '
                f'({getattr(response, "message", "")!r})')
        return response

    # ------------------------------------------------------------------ #
    # reading
    # ------------------------------------------------------------------ #

    def wait_for_data(self, timeout: float = 5.0) -> None:
        """Spin until both subscriptions have produced a message."""
        deadline = time.monotonic() + timeout
        while (self._joint_state is None or self._robot_state is None):
            if time.monotonic() > deadline:
                missing = []
                if self._joint_state is None:
                    missing.append(f'/{self.ns}/joint_states')
                if self._robot_state is None:
                    missing.append(f'/{self.ns}/robot_states')
                raise RobotRefused(f'nothing published on {", ".join(missing)}')
            rclpy.spin_once(self, timeout_sec=0.1)

    def joints_from_topic(self) -> List[float]:
        """Latest joint angles as broadcast on the topic. Radians."""
        self.wait_for_data()
        return list(self._joint_state.position[:JOINT_COUNT])

    def joints_from_service(self) -> List[float]:
        """Joint angles fetched on demand from the control box. Radians."""
        response = self.call(self.cli_get, GetFloat32List.Request(), 'get_servo_angle')
        return [float(a) for a in response.datas[:JOINT_COUNT]]

    def state_line(self) -> str:
        self.wait_for_data()
        s = self._robot_state
        return f'mode={s.mode} state={s.state} err={s.err} warn={s.warn}'

    # ------------------------------------------------------------------ #
    # commanding
    # ------------------------------------------------------------------ #

    def enable(self) -> None:
        """The three calls, in this order, every session and after every error.

        id=8 means "all joints". mode 0 is position control - the mode
        set_servo_angle needs. If MoveIt/ros2_control is running it will be
        holding the arm in mode 1 and fighting you.
        """
        self.call(self.cli_enable, SetInt16ById.Request(id=8, data=1), 'motion_enable')
        self.call(self.cli_mode, SetInt16.Request(data=MODE_POSITION), 'set_mode')
        self.call(self.cli_state, SetInt16.Request(data=SET_STATE_MOTION), 'set_state')

    def check_ready(self) -> None:
        """Fail now, with a reason, instead of silently not moving later.

        Drops the cached RobotMsg first: the control box reports its state on a
        timer, so the message already in hand was published before enable()
        landed and still describes the old mode.
        """
        self._robot_state = None
        self.wait_for_data()
        s = self._robot_state
        if s.err != 0:
            raise RobotRefused(f'arm is in error state err={s.err}; clear it in '
                               f'UFACTORY Studio or call /{self.ns}/clean_error')
        if s.mode != MODE_POSITION:
            raise RobotRefused(
                f'arm is in mode {s.mode}, set_servo_angle needs mode {MODE_POSITION}. '
                f'Mode 1 means ros2_control/MoveIt still owns the arm - stop the '
                f'realmove launch and use `lab driver <ip>`.')
        if s.state not in READY_STATES:
            raise RobotRefused(
                f'arm reports state {s.state} (suspended or stopping); ready '
                f'states are {READY_STATES}. Check for an e-stop or a collision '
                f'fault in UFACTORY Studio.')

    def move_to(self, angles: List[float], speed: float = 0.2,
                acc: float = 5.0) -> None:
        """Command all six joints to absolute positions, in radians.

        wait=True makes the service return only once the motion has finished,
        which is what lets you write this as a straight-line sequence.
        """
        if len(angles) != JOINT_COUNT:
            raise ValueError(f'need {JOINT_COUNT} angles, got {len(angles)}')

        request = MoveJoint.Request()
        request.angles = [float(a) for a in angles]
        request.speed = float(speed)      # rad/s
        request.acc = float(acc)          # rad/s^2
        request.mvtime = 0.0
        request.wait = True               # block until the arm gets there
        request.timeout = -1.0            # no timeout on the motion itself
        request.radius = -1.0             # -1 = stop at the point, no blending
        request.relative = False          # absolute, not an offset
        self.call(self.cli_move, request, 'set_servo_angle')


# ---------------------------------------------------------------------- #
# the actual "algorithm"
# ---------------------------------------------------------------------- #

def format_joints(label: str, angles: List[float]) -> str:
    rad = ' '.join(f'{a:8.4f}' for a in angles)
    deg = ' '.join(f'{math.degrees(a):7.2f}' for a in angles)
    return f'  {label:<24} rad [{rad} ]\n  {"":<24} deg [{deg} ]'


def build_waypoints(start: List[float], amplitude: float) -> List[List[float]]:
    """Three waypoints, each a small offset from wherever the arm is now.

    Offsets rather than absolute angles: the sequence is then safe from any
    starting configuration, and cannot swing the arm across the workspace
    because someone left it somewhere unexpected.
    """
    return [
        [start[0] + amplitude, start[1], start[2], start[3], start[4], start[5]],
        [start[0] + amplitude, start[1] - amplitude, start[2], start[3], start[4], start[5]],
        [start[0], start[1], start[2], start[3], start[4], start[5] + amplitude],
    ]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='move_joints_demo',
        description='Read joint states and command joint positions on a Lite 6.')
    parser.add_argument('--amplitude', type=float, default=0.15,
                        help='waypoint offset in radians (default 0.15 = 8.6 deg)')
    parser.add_argument('--speed', type=float, default=0.2, help='rad/s')
    parser.add_argument('--acc', type=float, default=5.0, help='rad/s^2')
    parser.add_argument('--namespace', default='ufactory')
    parser.add_argument('--dry-run', action='store_true',
                        help='read and print everything, command nothing')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if argv is None else argv)[1:])

    rclpy.init()
    arm = JointMover(namespace=args.namespace)

    try:
        # ---- 1. read ---------------------------------------------------- #
        print('waiting for the driver to publish...')
        arm.wait_for_data()
        print(f'arm state: {arm.state_line()}')

        from_topic = arm.joints_from_topic()
        from_service = arm.joints_from_service()
        print(format_joints('from joint_states topic', from_topic))
        print(format_joints('from get_servo_angle', from_service))

        # The two should agree to within the publish interval. If they diverge
        # by more than a fraction of a degree while the arm is still, something
        # else is commanding it.
        spread = max(abs(a - b) for a, b in zip(from_topic, from_service))
        print(f'  largest disagreement between the two: {spread:.6f} rad')

        start = from_service
        waypoints = build_waypoints(start, args.amplitude)

        print(f'\n{len(waypoints)} waypoints, amplitude {args.amplitude:+.4f} rad, '
              f'speed {args.speed} rad/s')
        for i, wp in enumerate(waypoints, 1):
            print(format_joints(f'waypoint {i}', wp))

        if args.dry_run:
            print('\n--dry-run: nothing was commanded.')
            return 0

        # ---- 2. enable -------------------------------------------------- #
        print('\nenabling...')
        arm.enable()
        arm.check_ready()
        print(f'arm state: {arm.state_line()}')

        # ---- 3. move, and measure what actually happened ---------------- #
        for i, target in enumerate(waypoints, 1):
            print(f'\n--- waypoint {i}/{len(waypoints)} ---')
            arm.move_to(target, speed=args.speed, acc=args.acc)

            reached = arm.joints_from_service()
            errors = [r - t for r, t in zip(reached, target)]
            worst = max(range(JOINT_COUNT), key=lambda j: abs(errors[j]))

            print(format_joints('commanded', target))
            print(format_joints('reached', reached))
            print(f'  largest error: joint{worst + 1} off by {errors[worst]:+.5f} rad '
                  f'({math.degrees(errors[worst]):+.4f} deg)')

        # ---- 4. put it back -------------------------------------------- #
        print('\nreturning to the starting configuration...')
        arm.move_to(start, speed=args.speed, acc=args.acc)
        final = arm.joints_from_service()
        drift = max(abs(f - s) for f, s in zip(final, start))
        print(format_joints('back at', final))
        print(f'  net drift over the whole sequence: {drift:.6f} rad '
              f'({math.degrees(drift):.4f} deg)')
        print(f'\ndone. arm state: {arm.state_line()}')
        return 0

    except RobotRefused as exc:
        print(f'\nrobot refused: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\ninterrupted - the arm finishes the motion already in progress',
              file=sys.stderr)
        return 130
    finally:
        arm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
