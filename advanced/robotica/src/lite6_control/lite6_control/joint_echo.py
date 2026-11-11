"""The smallest useful ROS 2 node: subscribe to the arm and print what it says.

Read this one first. It is the standard rclpy shape - create a node, subscribe,
spin - and it commands nothing, so you can run it against the real arm with no
risk at all.

    ros2 run lite6_control joint_echo
    ros2 run lite6_control joint_echo --ros-args -p rate_hz:=1.0
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointEcho(Node):

    def __init__(self):
        super().__init__('joint_echo')

        # Parameters are how you configure a node without editing it.
        #self.declare_parameter('namespace', '/ufactory')
        self.declare_parameter('namespace', '')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('degrees', False)

        namespace = self.get_parameter('namespace').value.strip('/')
        self._degrees = self.get_parameter('degrees').value
        period = 1.0 / max(self.get_parameter('rate_hz').value, 0.1)

        self._latest = None
        self.create_subscription(
            JointState, f'{namespace}/joint_states', self._on_joint_state, 10)

        # Print on a timer rather than on every message: the driver publishes
        # far faster than you can read.
        self.create_timer(period, self._print)
        self.get_logger().info(f'listening on {namespace}/joint_states')

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest = msg

    def _print(self) -> None:
        if self._latest is None:
            self.get_logger().warn('no joint states yet - is the driver running?')
            return
        positions = self._latest.position
        if self._degrees:
            values = ' '.join(f'{math.degrees(p):8.2f}' for p in positions)
            unit = 'deg'
        else:
            values = ' '.join(f'{p:8.4f}' for p in positions)
            unit = 'rad'
        self.get_logger().info(f'[{unit}] {values}')


def main(args=None):
    rclpy.init(args=args)
    node = JointEcho()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C, or the process being told to stop from outside.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
