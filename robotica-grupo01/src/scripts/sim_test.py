from src.lite6 import session

from xarm.wrapper import XArmAPI

arm = session.safe_session()
print(f"Position:{arm.get_position(is_radian=False)}")
print(f"Joint Angles:{arm.get_servo_angle(is_radian=False)}")

pose_a = [-114, 21, 72, 0, 51, -114]
print(f"arm.get_forward_kinematics(Pose A):\n{arm.get_forward_kinematics(pose_a)}")

arm.disconnect()