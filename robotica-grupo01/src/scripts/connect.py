from lite6 import session

from xarm.wrapper import XArmAPI

arm = session.safe_session()
arm.disconnect()
