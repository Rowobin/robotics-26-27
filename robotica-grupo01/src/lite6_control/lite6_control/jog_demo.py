"""An actual algorithm: exercise each joint in turn and measure what happened.

This is the shape most of your lab code will take - a sequential procedure that
commands the arm and reads it back - so it is worth understanding rather than
just running.

What it does, for each joint you select:

    1. read the joint angles from the control box
    2. jog that one joint by +delta, wait for the motion to finish
    3. read the angles again
    4. jog it back by -delta
    5. report commanded vs measured

Step 5 is the interesting one. The arm does not land exactly where you asked;
the residual is small, systematic, and real. That gap is the same one you will
see between RViz and Gazebo in Test 7, and between Gazebo and the hardware
later on.

    ros2 run lite6_control jog_demo --dry-run        # prints, moves nothing
    ros2 run lite6_control jog_demo                  # all six joints, 0.15 rad
    ros2 run lite6_control jog_demo --joints 0 1 --delta 0.2 --speed 0.15

SAFETY: this moves a real arm. Emergency stop within reach, nobody inside the
workspace. Start with --dry-run, then a single joint, then widen.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List

import rclpy
from rclpy.utilities import remove_ros_args

from lite6_control.lite6_client import JOINT_COUNT, Lite6Client, Lite6Error


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='jog_demo', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--joints', type=int, nargs='+', default=list(range(JOINT_COUNT)),
                        metavar='N', help='0-based joint indices (default: all six)')
    parser.add_argument('--delta', type=float, default=0.15,
                        help='jog size in radians (default: 0.15 = 8.6 deg)')
    parser.add_argument('--speed', type=float, default=0.2, help='rad/s')
    parser.add_argument('--acc', type=float, default=5.0, help='rad/s^2')
    parser.add_argument('--namespace', default='/ufactory',
                        help='the Lite 6 uses /ufactory, not /xarm')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the plan and the arm state, command nothing')
    return parser.parse_args(argv)


def report(label: str, angles: List[float]) -> None:
    rad = ' '.join(f'{a:8.4f}' for a in angles)
    deg = ' '.join(f'{math.degrees(a):7.2f}' for a in angles)
    print(f'  {label:<22} rad [{rad} ]')
    print(f'  {"":<22} deg [{deg} ]')


def exercise_joint(arm: Lite6Client, joint: int, delta: float,
                   speed: float, acc: float) -> None:
    print(f'\njoint{joint + 1}  (index {joint}), delta = {delta:+.4f} rad '
          f'({math.degrees(delta):+.2f} deg)')

    before = arm.get_angles()
    report('before', before)

    arm.jog(joint=joint, delta=delta, speed=speed, acc=acc, wait=True)
    after = arm.get_angles()
    report('after +delta', after)

    commanded = before[joint] + delta
    measured = after[joint]
    residual = measured - commanded
    print(f'  commanded {commanded:+.4f} rad, measured {measured:+.4f} rad, '
          f'residual {residual:+.5f} rad ({math.degrees(residual):+.3f} deg)')

    # Put it back where we found it.
    arm.jog(joint=joint, delta=-delta, speed=speed, acc=acc, wait=True)
    returned = arm.get_angles()
    drift = returned[joint] - before[joint]
    print(f'  returned, net drift over the round trip {drift:+.5f} rad')


def main(argv=None) -> int:
    argv = sys.argv if argv is None else argv
    args = parse_args(remove_ros_args(argv)[1:])

    for j in args.joints:
        if not 0 <= j < JOINT_COUNT:
            print(f'joint index {j} is out of range 0..{JOINT_COUNT - 1}', file=sys.stderr)
            return 2

    rclpy.init()
    arm = Lite6Client(namespace=args.namespace)
    try:
        print(f'arm state: {arm.describe_state()}')
        print(f'joints to exercise: {args.joints}')
        print(f'delta {args.delta:+.4f} rad, speed {args.speed} rad/s, acc {args.acc} rad/s^2')

        if args.dry_run:
            report('current angles', arm.get_angles())
            print('\n--dry-run: nothing was commanded.')
            return 0

        # Fail here, loudly, rather than silently not moving.
        arm.enable()
        arm.require_position_mode()

        for joint in args.joints:
            exercise_joint(arm, joint, args.delta, args.speed, args.acc)

        print(f'\ndone. final state: {arm.describe_state()}')
        return 0

    except Lite6Error as exc:
        print(f'\nrobot refused: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\ninterrupted - the arm finishes the motion it already started',
              file=sys.stderr)
        return 130
    finally:
        arm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
