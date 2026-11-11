# xarm_gazebo: the lab's copy

This is a copy of `xarm_gazebo` from `xarm_ros2` (branch `jazzy`, as built into the image), kept
here so that it shadows the image's package once it is built (`lab build`). Everything is upstream
except two things:

| File | Change |
|---|---|
| `launch/_robot_beside_table_gazebo.launch.py` | the Gazebo spawn yaw of the robot, `-Y 1.571` → `-Y -1.571` |
| `worlds/table_gz.world` | gravity, `0 0 0` → `0 0 -9.81` |

**The yaw.** Upstream spawns the Lite 6 on the edge of the table, facing off it. With the yaw changed
it faces −y, onto the table, where `lite6_checkers` puts the checkerboard (`-0.2, -0.74`: 0.24 m in
front of the base axis). Both `lab sim` and `lab sim-lite` use this launch file. Only the `gz` branch
of the launch file is changed; the `ignition` and classic Gazebo branches are not used here.

**The gravity.** Upstream turned it off in 2022 (`humble: change gravity from [0 0 -9.81] to
[0 0 0]`), presumably so that the arm does not sag. With it off, nothing rests on the board: a piece
pushed would keep sliding, because friction needs a weight to act on. With it on, the position
controller of the arm still holds its pose: 0.1 mrad of drift after ten idle seconds, and 0.2 mrad
of error at the end of a move. If you ever see the arm sag in your own controller, this is the
first thing to suspect.

To go back to upstream, delete this folder and run `lab build`; to see what differs from the image:

```bash
diff -r /opt/xarm_ws/src/xarm_ros2/xarm_gazebo ~/dev_ws/src/xarm_gazebo
```
