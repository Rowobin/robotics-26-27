# `lite6_control`

Worked example nodes for the UFACTORY Lite 6 — Robotics (02000537), 2026/2027.

## Two interfaces, and why it matters

There are two ways to command this arm, and they are **not** interchangeable:

| | `ros2_control` | driver API |
|---|---|---|
| interface | `/lite6_traj_controller/follow_joint_trajectory` | `/ufactory/set_servo_angle` |
| in Gazebo | **yes** (`lab sim`) | no |
| on hardware | **yes** (`lab real <ip>`) | yes (`lab driver <ip>`) |
| level | trajectories, through a controller | direct, one command at a time |

Only the first runs in simulation, so it is the only one where "test it in Gazebo
before the real arm" is possible. **Write your lab code against it.**

## The files

| | Works with | |
|---|---|---|
| `joint_echo.py` | anything | the minimal node — subscribe, spin, print. Commands nothing |
| `move_joints_demo.py` | `lab driver` | self-contained driver-API example, nothing hidden in a helper |
| `lite6_client.py` | `lab driver` | the reusable `Lite6Client` class to import |
| `jog_demo.py` | `lab driver` | one joint at a time, built on `Lite6Client` |
| `move_joints_traj_demo.py` | `lab sim`, `lab real` | the simplest move: one fixed joint target over `ros2_control` — no MoveIt, no driver API |

## Running

Every node here except `joint_echo` and `move_joints_traj_demo` uses the driver
API, so terminal 1 is:

```bash
lab driver 192.168.1.xxx
```

Terminal 2:

```bash
ros2 run lite6_control move_joints_demo --dry-run
```

```bash
ros2 run lite6_control move_joints_demo
```

`joint_echo` works against anything that publishes `/joint_states`, including
`lab sim` and `lab real`.

## Building

```bash
lab build
```

**Usually unnecessary.** `--symlink-install` links the package rather than copying
it, so editing a `.py` takes effect on the next `ros2 run`. Rebuild only for a new
entry point, a changed `package.xml`/`setup.py`, or a new package — and re-source
(or open a new shell) only for a brand-new package.

## Writing your own

**For the simulation pipeline, start from `move_joints_traj_demo.py`** — every
other node except `joint_echo` speaks the driver API, which Gazebo does not
provide. It runs unchanged in Gazebo and on the arm:

```bash
lab sim
```

```bash
ros2 run lite6_control move_joints_traj_demo --dry-run
```

It writes against `ros2_control`:

```
/lite6_traj_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory
/joint_states                                    sensor_msgs/msg/JointState
```

Send a `FollowJointTrajectory` goal with `joint_names` `joint1`..`joint6` and
`JointTrajectoryPoint`s whose `time_from_start` is measured from the start of the
whole trajectory, not from the previous point. Check `error_code` on the result:
0 is success, anything else means the arm stopped early.

Map `/joint_states` by name, not by index — the ordering is not promised, and the
message may carry joints you did not ask about.

Make waypoints relative to the current pose rather than absolute — the node is then
safe from any starting configuration and cannot swing the arm across the workspace
because someone left it somewhere unexpected.

Add your node to `entry_points` in `setup.py`, declare `control_msgs` and
`trajectory_msgs` in `package.xml`, then `lab build` once.

## Four things that will cost you an afternoon

**Angles are radians.** The UFACTORY docs are mostly degrees; these interfaces are
not. Verified on the robot: `get_servo_angle` returns exactly what
`/ufactory/joint_states` publishes.

**`ret` is the real result** (driver API). A service call can return perfectly
happily with `ret=9` having done nothing. `Lite6Client` raises on any non-zero
`ret`; if you call the services directly, check it yourself. The action interface
has `error_code` on its result, for the same reason.

**`set_state()` and the reported state are different value spaces.** You *pass* 0
to make the arm ready; it then *reports* 2 (sleeping = idle and ready) or 1 (in
motion). A reported 0 never happens. Constants for both are in `lite6_client.py`.

**Don't make blocking calls inside callbacks.** Every `Lite6Client` method spins
the node waiting for a reply — call one from a subscription or timer callback and
you deadlock, because you are already inside the spin. Write your algorithm as a
straight-line procedure in `main()`. If you truly need both, that is a
`MultiThreadedExecutor` with a `ReentrantCallbackGroup`.

## Safety

On hardware this moves a real arm. Emergency stop within reach, nobody in the
workspace, `--dry-run` first. Nothing runs on the real robot that has not run in
Gazebo first.
