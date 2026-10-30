"""Move the Lite 6 to a fixed joint position, through ros2_control.

Works in Gazebo (lab sim) and on the real arm (lab real <ip>), unchanged.

    ros2 run lite6_control move_joints_traj_demo
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Where to go: joint1..joint6, in radians.
TARGET = [0.5, 0.2, 0.6, 0.0, 0.4, 0.0]

# How long the move takes, in seconds.
MOVE_TIME = 3


def main():
    rclpy.init()
    node = Node('move_joints_traj_demo')

    # The trajectory controller listens on this action.
    client = ActionClient(node, FollowJointTrajectory,
                          '/lite6_traj_controller/follow_joint_trajectory')
    print('waiting for the controller...')
    client.wait_for_server()

    # One point: be at TARGET, at rest, MOVE_TIME seconds from now.
    point = JointTrajectoryPoint()
    point.positions = TARGET
    point.velocities = [0.0] * 6
    point.time_from_start = Duration(sec=MOVE_TIME)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3',
                                   'joint4', 'joint5', 'joint6']
    goal.trajectory.points = [point]

    # Send the goal, and wait until the controller accepts it.
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        print('goal rejected')
        return

    # Wait until the motion is finished.
    print('moving...')
    future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, future)

    # 0 means the arm got there; anything else means it stopped early.
    print(f'done, error_code = {future.result().result.error_code}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
