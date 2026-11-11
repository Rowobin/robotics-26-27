"""
Recover a UFACTORY Lite 6 stuck at a joint limit (error C23), and re-zero a
joint whose zero point has shifted.

The controller refuses to move while a joint is at or past its software
limit, so the web interface and ROS cannot bring it back. This script gives a
small menu: release the brake on a joint you choose and turn it by hand,
move or nudge a single joint slowly, clear errors and enable, go to the
initial position, or set a joint's zero at its current position.

Requires:  pip install xarm-python-sdk
Run:       python tools/recover_joint_limit.py [ROBOT_IP]   (from the repo root)

Before running: stop any ROS driver connected to the robot, hold the arm when
a brake is released, and keep the emergency stop within reach.
"""
import sys
import time

from xarm.wrapper import XArmAPI

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.189"

# Lite 6 joint limits in degrees (xarm_description/config/lite6/joint_limits).
LIMITS_DEG = {
    1: (-360.0, 360.0),
    2: (-150.0, 150.0),
    3: (-3.5, 300.0),
    4: (-360.0, 360.0),
    5: (-124.0, 124.0),
    6: (-360.0, 360.0),
}
NEAR_DEG = 5.0       # within this of a limit counts as "at the limit"
JOINT_SPEED = 10.0   # deg/s for the recovery moves
JOINT_ACC = 100.0    # deg/s^2
MAX_NUDGE_DEG = 5.0  # largest single step for the nudge option

# Releasing these brakes lets the arm fall under gravity.
GRAVITY_JOINTS = {2, 3}

MENU = """
  1-6  release the brake of that joint and turn it by hand
  m    move one joint alone, slowly, to an angle
  r    nudge one joint by small steps (fine alignment)
  e    clear errors and enable
  h    go to the initial position (all joints 0)
  z    set a joint's zero at its current position (calibration)
  s    show joints and status again
  q    quit
"""


def ask(prompt):
    return input(prompt).strip().lower() in ("y", "yes")


def read_joints(arm):
    code, angles = arm.get_servo_angle(is_radian=False)
    if code != 0:
        print(f"  could not read joint angles (code {code}); showing last report")
        return list(arm.angles[:6])
    return angles[:6]


def show(arm):
    for i, a in enumerate(read_joints(arm), start=1):
        lo, hi = LIMITS_DEG[i]
        if not lo <= a <= hi:
            flag = "  <-- OUT OF RANGE"
        elif a > hi - NEAR_DEG or a < lo + NEAR_DEG:
            flag = "  <-- AT LIMIT"
        else:
            flag = ""
        print(f"  J{i}: {a:9.3f} deg   limit [{lo:7.1f}, {hi:7.1f}]{flag}")
    time.sleep(0.3)
    print(f"  error code: {arm.error_code}   warn code: {arm.warn_code}   state: {arm.state}")


def release_joint(arm, j):
    print(f"\nReleasing the J{j} brake. J{j} will turn freely; hold the arm.")
    if j in GRAVITY_JOINTS:
        print(f"!! J{j} carries the weight of the arm: it WILL drop when released. "
              "Support the upper arm and forearm with both hands.")
        if input(f"Type RELEASE J{j} to continue: ").strip().upper() != f"RELEASE J{j}":
            print("  cancelled, brake left on.")
            return
    elif not ask(f"Holding the arm? Release J{j} now? [y/N] "):
        print("  cancelled, brake left on.")
        return

    arm.clean_error()
    arm.clean_warn()
    # Same order as the controller's own G-code D12: disable the joint, then release it.
    print(f"  motion_enable(False, J{j}) -> {arm.motion_enable(enable=False, servo_id=j)}")
    print(f"  set_servo_detach({j}) -> {arm.set_servo_detach(servo_id=j)}")
    print(f"  Turn J{j} by hand well back inside its limit. The readout may stay "
          "frozen while the controller is in error; judge the position by eye.")
    try:
        while True:
            reply = input("  [Enter] show readout   [d] done, re-engage brake: ").strip().lower()
            if reply == "d":
                break
            code, polled = arm.get_servo_angle(is_radian=False)
            print(f"  J{j}: polled {polled[j - 1] if code == 0 else 'n/a'}   "
                  f"reported {arm.angles[j - 1]:.2f}   (informative only)")
    finally:
        print(f"  set_servo_attach({j}) -> {arm.set_servo_attach(servo_id=j)}  (brake back on)")


def enable(arm):
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1.0)
    show(arm)


def move_joint(arm):
    try:
        j = int(input("  Joint (1-6): "))
        angle = float(input(f"  Target angle for J{j} in degrees: "))
    except ValueError:
        print("  not a number, cancelled.")
        return
    if j not in LIMITS_DEG:
        print("  no such joint.")
        return
    lo, hi = LIMITS_DEG[j]
    if not lo + NEAR_DEG <= angle <= hi - NEAR_DEG:
        print(f"  {angle} is at or past the J{j} limit [{lo}, {hi}], cancelled.")
        return
    if not ask(f"  Move J{j} alone to {angle:.1f} deg at {JOINT_SPEED:.0f} deg/s? [y/N] "):
        return
    code = arm.set_servo_angle(servo_id=j, angle=angle, speed=JOINT_SPEED,
                               mvacc=JOINT_ACC, wait=True)
    print(f"  set_servo_angle -> {code}")
    show(arm)


def ask_joint():
    try:
        j = int(input("  Joint (1-6): "))
    except ValueError:
        j = 0
    if j not in LIMITS_DEG:
        print("  no such joint.")
        return None
    return j


def nudge_joint(arm):
    j = ask_joint()
    if j is None:
        return
    print(f"  Enter a step in degrees (e.g. 1, -0.5, max {MAX_NUDGE_DEG:.0f}); "
          "empty line to stop.")
    while True:
        text = input(f"  J{j} step: ").strip()
        if not text:
            break
        try:
            step = float(text)
        except ValueError:
            print("  not a number.")
            continue
        if abs(step) > MAX_NUDGE_DEG:
            print(f"  step larger than {MAX_NUDGE_DEG:.0f} deg, ignored.")
            continue
        code = arm.set_servo_angle(servo_id=j, angle=step, relative=True,
                                   speed=JOINT_SPEED, mvacc=JOINT_ACC, wait=True)
        print(f"  set_servo_angle -> {code}   J{j} now reads "
              f"{read_joints(arm)[j - 1]:.3f} deg")
        if code != 0:
            show(arm)
            break


def zero_joint(arm):
    """Make the joint's current physical position read 0.

    Uses the same sequence as the controller's own G-code commands D12/D13:
    clear errors, disable and release the joint, set the zero, enable it.
    UFACTORY marks set_servo_zero as a debugging call, so this asks twice.
    """
    j = ask_joint()
    if j is None:
        return
    angles = read_joints(arm)
    print(f"\n  J{j} reads {angles[j - 1]:.3f} deg. After this it will read 0 at its "
          "CURRENT physical position.")
    others = [i for i in range(1, 7) if i != j and abs(angles[i - 1]) > 0.5]
    if others:
        print("  Note: " + ", ".join(f"J{i}={angles[i - 1]:.1f}" for i in others) +
              " are not at 0, so the arm is not in the initial pose; make sure your "
              "reference (level, marks) does not depend on them.")
    print("  Recommended: go to the initial position (h), use r to nudge the joint "
          "until it is exactly aligned (phone level on the gripper/flange), then z.")
    print("  Do NOT run this on a joint whose zero is correct. Use with care: "
          "UFACTORY marks this call as a debugging function.")
    if input(f"  Type ZERO J{j} to continue: ").strip().upper() != f"ZERO J{j}":
        print("  cancelled, nothing changed.")
        return

    arm.clean_error()
    arm.clean_warn()
    print(f"  motion_enable(False, J{j}) -> {arm.motion_enable(enable=False, servo_id=j)}")
    print(f"  set_servo_detach({j}) -> {arm.set_servo_detach(servo_id=j)}")
    print(f"  J{j} is released. Hold it exactly at the aligned position "
          "(check the level again).")
    if input(f"  Press Enter to store this as the J{j} zero, or type c to cancel: "
             ).strip().lower() == "c":
        print(f"  cancelled. set_servo_attach({j}) -> {arm.set_servo_attach(servo_id=j)}")
        print(f"  motion_enable(True, J{j}) -> {arm.motion_enable(enable=True, servo_id=j)}")
        return
    print(f"  set_servo_zero({j}) -> {arm.arm.set_servo_zero(servo_id=j)}")
    print(f"  motion_enable(True, J{j}) -> {arm.motion_enable(enable=True, servo_id=j)}")
    time.sleep(1.0)
    show(arm)
    print(f"\n  Now power-cycle the control box, then run this again: e, then h, and "
          f"check the initial pose. J{j} should read about 0 where you set it.")


def go_home(arm):
    if not ask("  Move all joints to 0 at low speed? Check the path is clear. [y/N] "):
        return
    code = arm.move_gohome(speed=JOINT_SPEED, mvacc=JOINT_ACC, wait=True)
    print(f"  move_gohome -> {code}")
    show(arm)


def main():
    print(f"Connecting to {ROBOT_IP} ...")
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    try:
        show(arm)
        while True:
            print(MENU)
            choice = input("Choice: ").strip().lower()
            if choice in {"1", "2", "3", "4", "5", "6"}:
                release_joint(arm, int(choice))
            elif choice == "m":
                move_joint(arm)
            elif choice == "e":
                enable(arm)
            elif choice == "h":
                go_home(arm)
            elif choice == "r":
                nudge_joint(arm)
            elif choice == "z":
                zero_joint(arm)
            elif choice == "s":
                show(arm)
            elif choice == "q":
                break
            else:
                print("  unknown choice.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
