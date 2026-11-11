"""A thin, readable wrapper over the ``/ufactory`` services of xarm_ros2.

Why this file exists
--------------------
The raw service API is fiddly in three specific ways, and every one of them
costs you an afternoon the first time:

1. **Angles are in radians.**  The UFACTORY SDK documentation is mostly written
   in degrees; the ROS 2 services are not.  Verified against the robot:
   ``/ufactory/get_servo_angle`` returns exactly what ``/ufactory/joint_states``
   publishes.

2. **A service call that "succeeds" tells you nothing.**  Every xarm service
   reports its own outcome in ``ret``.  ROS will happily hand you a response
   with ``ret=9`` and no exception.  This wrapper raises on any non-zero
   ``ret``, so a refused command is loud instead of silent.

3. **Mode and state gate everything.**  ``set_servo_angle`` needs mode 0
   (position control).  If MoveIt / ros2_control is running it holds the arm in
   mode 1 (servo streaming) and your commands are ignored.
   :meth:`Lite6Client.require_position_mode` checks for exactly that.

4. **set_state() and the reported state are different value spaces.**  You pass
   0 to set_state() to make the arm ready; it then *reports* state 2
   (sleeping/idle) or 1 (in motion).  A reported 0 never happens.  See the
   constants below.

Blocking by design
------------------
Every method here blocks until the robot answers, so you write your algorithm
top to bottom like a script.  That is the right shape for a lab exercise and it
keeps the control flow obvious.

It is also why you must **not** call these methods from inside a subscription
or timer callback: the call spins the node waiting for a response, but you are
already inside the spin, and the node deadlocks.  Call them from ``main()``.
If you need callbacks *and* service calls, that is a MultiThreadedExecutor with
a ReentrantCallbackGroup, and a different file.
"""

from __future__ import annotations

import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import (
    Call,
    GetFloat32List,
    MoveJoint,
    SetInt16,
    SetInt16ById,
)

# The Lite 6 is a 6R arm. The API is shared with the 7-DOF xArm7, so several
# services return 7 values; the 7th is always 0 here.
JOINT_COUNT = 6

# Modes. Same value space whether you set it or read it back.
#   (xarm_sdk/cxx/doc/xarm_cplus_api.md, set_mode)
MODE_POSITION = 0        # set_servo_angle, set_position - what you want
MODE_SERVO = 1           # what ros2_control / MoveIt puts the arm in
MODE_JOINT_TEACH = 2
MODE_JOINT_VELOCITY = 4
MODE_CARTESIAN_VELOCITY = 5

# States. READ THIS: set_state() and the reported state use DIFFERENT value
# spaces, and confusing them is a genuinely nasty trap.
#
#   what you PASS to set_state():   0 = motion   3 = pause   4 = stop
#   what robot_states REPORTS:      1 = in motion   2 = sleeping
#                                   3 = suspended   4 = stopping
#
# So you send 0 to make the arm ready, and it then reports 2 (sleeping, i.e.
# ready and idle) or 1 (currently moving). A reported 0 is not a thing.
SET_STATE_MOTION = 0
SET_STATE_PAUSE = 3
SET_STATE_STOP = 4

REPORTED_IN_MOTION = 1
REPORTED_SLEEPING = 2    # idle and ready - the normal resting state
REPORTED_SUSPENDED = 3
REPORTED_STOPPING = 4

#: Reported states in which the arm will accept a motion command.
READY_STATES = (REPORTED_IN_MOTION, REPORTED_SLEEPING)


class Lite6Error(RuntimeError):
    """The robot refused a command, or never answered."""


class Lite6Client(Node):
    """Blocking client for one Lite 6 arm.

    Typical use::

        rclpy.init()
        arm = Lite6Client()
        try:
            arm.enable()
            arm.jog(joint=0, delta=0.1)
            print(arm.get_angles())
        finally:
            arm.destroy_node()
            rclpy.shutdown()
    """

    def __init__(self, namespace: str = '', node_name: str = 'lite6_client'):
        super().__init__(node_name)
        self.ns = namespace

        self._joint_state: Optional[JointState] = None
        self._robot_state: Optional[RobotMsg] = None

        self.create_subscription(
            JointState, f'{self.ns}/joint_states', self._on_joint_state, 10)
        self.create_subscription(
            RobotMsg, f'{self.ns}/robot_states', self._on_robot_state, 10)

        self._srv_cache = {}   # NOTE: not `_clients` - rclpy.Node uses that name internally

    # -- plumbing ---------------------------------------------------------- #

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_robot_state(self, msg: RobotMsg) -> None:
        self._robot_state = msg

    def _call(self, name, srv_type, request, timeout: float = 30.0):
        """Call /<ns>/<name> and raise unless the robot reports ret == 0."""
        client = self._srv_cache.get(name)
        if client is None:
            client = self.create_client(srv_type, f'{self.ns}/{name}')
            self._srv_cache[name] = client

        if not client.wait_for_service(timeout_sec=5.0):
            raise Lite6Error(
                f'service {self.ns}/{name} is not there. Is the driver running? '
                f'Try:  lab driver <robot_ip>')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done():
            raise Lite6Error(f'{self.ns}/{name} did not answer within {timeout:.0f}s')

        response = future.result()
        ret = getattr(response, 'ret', 0)
        if ret != 0:
            raise Lite6Error(
                f'{self.ns}/{name} refused the command: ret={ret} '
                f'({getattr(response, "message", "")!r}). '
                f'Check mode/state with `ros2 topic echo {self.ns}/robot_states --once`, '
                f'and the arm itself in UFACTORY Studio.')
        return response

    # -- reading ----------------------------------------------------------- #

    def wait_for_joint_state(self, timeout: float = 5.0) -> JointState:
        """Block until a JointState arrives (the driver publishes continuously)."""
        return self._wait_for('_joint_state', f'{self.ns}/joint_states', timeout)

    def wait_for_robot_state(self, timeout: float = 5.0) -> RobotMsg:
        """Block until a RobotMsg arrives. This is where mode/state/err live."""
        return self._wait_for('_robot_state', f'{self.ns}/robot_states', timeout)

    def _wait_for(self, attr: str, topic: str, timeout: float):
        deadline = time.monotonic() + timeout
        while getattr(self, attr) is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        value = getattr(self, attr)
        if value is None:
            raise Lite6Error(f'nothing published on {topic} within {timeout:.0f}s')
        return value

    def get_angles(self) -> List[float]:
        """Current joint angles in radians, straight from the control box."""
        response = self._call('get_servo_angle', GetFloat32List, GetFloat32List.Request())
        return [float(a) for a in response.datas[:JOINT_COUNT]]

    def describe_state(self) -> str:
        """One line you can print when something is not moving."""
        s = self.wait_for_robot_state()
        return (f'mode={s.mode} state={s.state} err={s.err} warn={s.warn} '
                f'brakes={s.mt_brake:#08b} enabled={s.mt_able:#08b}')

    # -- gating ------------------------------------------------------------ #

    def fresh_robot_state(self, timeout: float = 5.0) -> RobotMsg:
        """Discard the cached RobotMsg and wait for one published after now.

        Needed straight after enable(): the control box reports its state on a
        timer, so the message sitting in the cache was published *before* the
        mode/state change landed and still describes the old situation.
        """
        self._robot_state = None
        return self._wait_for('_robot_state', f'{self.ns}/robot_states', timeout)

    def require_position_mode(self, timeout: float = 5.0) -> None:
        """Fail early, with the actual reason, if joint commands cannot work.

        This is the failure that looks like nothing happening at all: the
        service returns, the arm does not move, and there is no error anywhere.
        """
        deadline = time.monotonic() + timeout
        while True:
            s = self.fresh_robot_state()
            if s.err == 0 and s.mode == MODE_POSITION and s.state in READY_STATES:
                return
            if time.monotonic() >= deadline:
                break

        if s.err != 0:
            raise Lite6Error(
                f'the arm is in error state (err={s.err}). Clear it with '
                f'clean_error() or in UFACTORY Studio, then enable() again.')
        if s.mode != MODE_POSITION:
            extra = ''
            if s.mode == MODE_SERVO:
                extra = (' That is the mode ros2_control uses, so MoveIt is '
                         'probably still running. Stop the realmove launch '
                         '(`lab kill`) and use `lab driver <ip>` instead.')
            raise Lite6Error(
                f'the arm is in mode {s.mode}, but set_servo_angle needs mode '
                f'{MODE_POSITION} (position control).{extra}')
        raise Lite6Error(
            f'the arm reports state {s.state} (suspended or stopping), so it '
            f'will not accept motion. Ready states are {READY_STATES} '
            f'(1 = in motion, 2 = sleeping). Call enable(), and check for an '
            f'emergency stop or a collision fault in UFACTORY Studio.')

    # -- commanding -------------------------------------------------------- #

    def motion_enable(self, enable: bool = True) -> None:
        """id=8 means "all joints" on the Lite 6."""
        self._call('motion_enable', SetInt16ById,
                   SetInt16ById.Request(id=8, data=1 if enable else 0))

    def set_mode(self, mode: int) -> None:
        self._call('set_mode', SetInt16, SetInt16.Request(data=int(mode)))

    def set_state(self, state: int) -> None:
        self._call('set_state', SetInt16, SetInt16.Request(data=int(state)))

    def clean_error(self) -> None:
        self._call('clean_error', Call, Call.Request())

    def enable(self) -> None:
        """The three calls, in the order the robot expects.

        Needed every session, and again after every error or emergency stop.
        """
        self.motion_enable(True)
        self.set_mode(MODE_POSITION)
        self.set_state(SET_STATE_MOTION)

    def move_joints(self, angles: Sequence[float], speed: float = 0.2,
                    acc: float = 5.0, wait: bool = True,
                    relative: bool = False) -> None:
        """Command all six joints.

        :param angles:   six values, radians. Absolute, or deltas if *relative*.
        :param speed:    rad/s. 0.2 is already clearly visible motion.
        :param acc:      rad/s^2.
        :param wait:     block until the motion finishes.
        :param relative: treat *angles* as offsets from where the arm is now.
        """
        if len(angles) != JOINT_COUNT:
            raise ValueError(f'expected {JOINT_COUNT} angles, got {len(angles)}')

        request = MoveJoint.Request()
        request.angles = [float(a) for a in angles]
        request.speed = float(speed)
        request.acc = float(acc)
        request.mvtime = 0.0
        request.wait = bool(wait)
        request.timeout = -1.0
        request.radius = -1.0
        request.relative = bool(relative)
        self._call('set_servo_angle', MoveJoint, request)

    def jog(self, joint: int, delta: float, speed: float = 0.2,
            acc: float = 5.0, wait: bool = True) -> None:
        """Move ONE joint by *delta* radians, leaving the others alone.

        :param joint: 0-based index, so joint1 on the arm is ``joint=0``.
        """
        if not 0 <= joint < JOINT_COUNT:
            raise ValueError(f'joint must be 0..{JOINT_COUNT - 1}, got {joint}')
        offsets = [0.0] * JOINT_COUNT
        offsets[joint] = float(delta)
        self.move_joints(offsets, speed=speed, acc=acc, wait=wait, relative=True)

    def go_home(self, speed: float = 0.2) -> None:
        """All joints to zero. Check the path is clear first."""
        self.move_joints([0.0] * JOINT_COUNT, speed=speed, wait=True)
