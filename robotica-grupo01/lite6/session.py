import yaml
from xarm.wrapper import XArmAPI

def safe_session():
    with open('config/robot.yaml', 'r') as f:
        config = yaml.full_load(f)

    # Connect
    arm = XArmAPI(config.get('ip'))

    # Clear warnings and errors
    arm.clean_warn()
    arm.clean_error()

    # Enable motion and set position-control mode/state
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    # Reduced-mode caps: 150 mm/s TCP, 54 deg/s joints
    arm.set_reduced_max_tcp_speed(config.get('max_joint_speed'))
    arm.set_reduced_max_joint_speed(config.get('max_tcp_speed'), is_radian=False)

    # TCP fence: [x_max, x_min, y_max, y_min, z_max, z_min] in mm
    arm.set_reduced_tcp_boundary([300, 70, 200, -200, 400, 10])
    arm.set_fence_mode(True)
    arm.set_reduced_mode(True)

    return arm