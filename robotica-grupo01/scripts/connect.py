# uv run python -m scripts.connect

from lite6 import session

from xarm.wrapper import XArmAPI

print("hello, robot!")
arm = session.safe_session()
arm.disconnect()
