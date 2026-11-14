# `lite6_kinematics` — Labworks 1 and 2

Forward and inverse kinematics of the UFACTORY Lite 6, as ROS 2 nodes — Robotics
(02000537), 2026/2027. The handouts are the specification; this file is the map of the
package. Labwork 1 comes first; **[Labwork 2 starts here](#labwork-2--inverse-kinematics)**.

## The files

Every file of the package. In each labwork you implement one, and you do not touch the
rest.

| File | | Purpose |
|---|---|---|
| `lite6_kinematics/forward_kinematics.py` | **LW1: to implement** | `dh_table`, `mdh_transform`, `forward_kinematics` and the node `FkNode`. The only file you edit in LW1. It also holds `pose_from_transform`, `quaternion_from_matrix` and `main`, which are **provided** — do not edit them. LW2 uses it: it stays as you delivered it |
| `lite6_kinematics/inverse_kinematics.py` | **LW2: to implement** | `pose_error`, `jacobian`, `inverse_kinematics` and the node `IkNode`. The only file you edit in LW2. It also holds `rotation_vector`, `transform_from_pose`, `JOINT_LIMITS`, the constants of Table 1 and `main`, which are **provided** — do not edit them |
| `lite6_kinematics/fk_check.py` | provided | LW1 checker node: moves the arm through the ten poses and compares the pose your node publishes with tf and, on the real arm, with the controller |
| `lite6_kinematics/ik_check.py` | provided | LW2 checker node: moves the arm to a starting configuration, sends your node a desired pose, executes the solution it answers with, and compares the pose reached with the desired one |
| `lite6_kinematics/ik_study.py` | provided | LW2 convergence studies on 500 poses: `convergence`, `nudge`, `zero`. No Gazebo, no arm |
| `lite6_kinematics/checking.py` | provided | the checkers' helpers: reference poses as 4×4 matrices, the error metrics e_p and e_o |
| `lite6_kinematics/__init__.py` | provided | empty; makes the folder a Python package |
| `config/lw1_poses.yaml` | provided | the ten joint configurations `fk_check` visits |
| `config/lw2_targets.yaml` | provided | the starting configurations S0–S4 and the desired poses A–D and U that `ik_check` uses |
| `test/test_forward_kinematics.py` | provided | the LW1 unit tests; the node test needs ROS, so it runs inside the container |
| `test/test_inverse_kinematics.py` | provided | the LW2 unit tests; same |
| `package.xml` | provided | the ROS 2 package description and its dependencies |
| `setup.py` | provided | installs the package and registers the `fk_node`, `fk_check`, `ik_node`, `ik_check` and `ik_study` commands |
| `setup.cfg` | provided | where `ros2 run` finds the installed commands |
| `resource/lite6_kinematics` | provided | empty marker that registers the package with ROS 2 |
| `README.md` | provided | this file |

The provided files are compared against the originals at grading. Edit them and
your results do not count.

`forward_kinematics.py` may import `numpy` and `math` for the maths, and `rclpy`,
`sensor_msgs` and `geometry_msgs` for the node (already imported in the template),
nothing else. `inverse_kinematics.py` may import the same, and `forward_kinematics`
from your own `forward_kinematics.py` (already imported in the template).

# Labwork 1 — forward kinematics

## Conventions

Craig's modified DH. Frame {0} = `link_base`, frame {6} = `link_eef` (the
end-effector frame). Joint angles in **radians**, exactly as on `/joint_states`.
Lengths in **metres**.

Your node `fk_node` reads `/joint_states` and publishes the pose of `link_eef`
in `link_base` on `/lite6_fk/pose` (`geometry_msgs/PoseStamped`, position in
metres, orientation as a quaternion). Its `header.stamp` is the stamp of the
joint state it was computed from.

## Before you start: the container

Every command below is typed **inside the container**. Start it once, from the
repository root:

```bash
./lab.sh up
```

Each terminal the steps ask for is a new terminal window in which you run:

```bash
./lab.sh shell
```

The prompt changes to `ros@lab0:~/dev_ws$`. Only there do `lab`, `ros2` and
`python3 -m pytest` exist: in a plain terminal on your machine they are not found.

## 1. Tests

```bash
python3 -m pytest ~/dev_ws/src/lite6_kinematics/test -v
```

All green before you go further. They check structure and units, and that your
node publishes the pose of the joint states it receives — not whether your table
is right; that is step 2. The `pose_from_transform` and `quaternion_from_matrix`
tests pass from the start: they guard the provided code, not your work. The
maths tests need no ROS; the node test does, so it runs inside the container.

## 2. Gazebo

Once, to register the package and its `fk_node` and `fk_check` commands:

```bash
lab build
```

Then open a new shell (a brand-new package needs re-sourcing).

**Terminal 1:**

```bash
lab sim-lite
```

This starts Gazebo and the robot controllers only, without MoveIt. To look at
the robot while it runs, open another terminal and run `lab rviz`.

**Terminal 2** — your node:

```bash
ros2 run lite6_kinematics fk_node
```

Look at what it publishes:

```bash
ros2 topic echo /lite6_fk/pose
```

**Terminal 3** — the checker:

```bash
ros2 run lite6_kinematics fk_check --ros-args -p robot:=sim
```

The arm visits ten poses. At each one the checker reads the pose your node
published, prints it and its distance from `link_base → link_eef` in tf.
Result: `~/shared/lw1_fk_sim.csv`.

The robot model that tf and Gazebo use differs slightly from the manufacturer's
sheet, so a correct implementation shows about 1 mm of position error here.
That is within P1.3.

## 3. The real arm

Only Part D moves the physical robot. The rules of Labwork 0 part 1 §3 apply
without exception: reduced mode on, one group member at the emergency stop, no
hands within the workspace while the robot is enabled.

**Terminal 1:**

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — your node, the same one:

```bash
ros2 run lite6_kinematics fk_node
```

**Terminal 3:**

```bash
ros2 run lite6_kinematics fk_check --ros-args -p robot:=real
```

Same ten poses, slowly, one at a time: it waits for Enter before every motion.
Ctrl+C quits between motions; **it does not stop a motion in progress — the
emergency stop does.** Besides tf, it compares with the pose the arm's own
controller reports (`/ufactory/robot_states`). Result: `~/shared/lw1_fk_real.csv`.


---

# Labwork 2 — inverse kinematics

You solve the inverse kinematics of the Lite 6 by Newton's method, with the Jacobian
computed numerically from **your Labwork 1 forward kinematics**, and publish the
solution from a node. Open `lite6_kinematics/inverse_kinematics.py`: the four things to
write are marked `TODO (LW2, Part B)`.

## Getting the Labwork 2 files

Labwork 2 only **adds** files to this package (`inverse_kinematics.py`, `ik_check.py`,
`ik_study.py`, `config/lw2_targets.yaml`, `test/test_inverse_kinematics.py`) and updates
`setup.py`, `package.xml` and this README, which you never edited. Your
`forward_kinematics.py` is not touched. In the repository root:

```bash
git pull
```

If git complains about a file you did not edit, tell the instructor before you
overwrite anything; **never** replace `forward_kinematics.py`.

## Conventions

Those of Labwork 1: frame {0} = `link_base`, frame {6} = `link_eef`, joint angles in
**radians**, lengths in **metres**. The desired pose is the pose of `link_eef` in
`link_base`. Your node `ik_node` subscribes to `/lite6_ik/target`
(`geometry_msgs/PoseStamped`) and `/joint_states`, and publishes on
`/lite6_ik/solution` (`sensor_msgs/JointState`) the joint angles that reach the
target — only if the method converged. The `header.stamp` of the solution is the stamp
of the target: that is how `ik_check` knows which target it answers.

## 1. Tests

Inside the container (see *Before you start* above):

```bash
python3 -m pytest ~/dev_ws/src/lite6_kinematics/test -v
```

All green before you go further. They need your Labwork 1 `forward_kinematics()` to
work, and use it: they check structure, units and internal consistency — not whether
your forward kinematics is right; that is what Gazebo and the arm are for. The
`rotation_vector`, `transform_from_pose` and constants tests pass from the start: they
guard the provided code, not your work.

## 2. Gazebo

Once, to register the new commands:

```bash
lab build
```

Then open a new shell.

**Terminal 1:**

```bash
lab sim-lite
```

(To look at the robot, `lab rviz` in another terminal.)

**Terminal 2** — your node:

```bash
ros2 run lite6_kinematics ik_node
```

**Terminal 3** — the checker:

```bash
ros2 run lite6_kinematics ik_check --ros-args -p robot:=sim
```

For each of the five desired poses in `config/lw2_targets.yaml` the checker moves the
arm to the starting configuration S0 and records the angles it reaches, sends your node
the desired pose and waits 5 s for the solution, then moves the arm to the solution and
compares the pose it reaches (from tf, i.e. from the robot model) with the desired
pose. One of the five is out of the robot's reach: your node must not answer, and the
checker then records the pose as **refused** and does not move the arm.
Result: `~/shared/lw2_ik_sim.csv`.

As in Labwork 1, the robot model differs slightly from the manufacturer's sheet, so a
correct implementation shows about 1 mm of position error here. That is within P2.4.

## 3. The real arm

Only after Gazebo is done: every pose reached, the out-of-reach pose refused. The rules
of Labwork 0 part 1 §3 apply without exception: reduced mode on, one group member at
the emergency stop, no hands within the workspace while the robot is enabled.

**Terminal 1:**

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — your node, the same one:

```bash
ros2 run lite6_kinematics ik_node
```

**Terminal 3:**

```bash
ros2 run lite6_kinematics ik_check --ros-args -p robot:=real
```

The same poses, slowly, and **it waits for Enter before every motion**. Ctrl+C quits
between motions; **it does not stop a motion in progress — the emergency stop does.**
Besides tf, it compares with the pose the arm's own controller reports
(`/ufactory/robot_states`). Result: `~/shared/lw2_ik_real.csv`.

## 4. The analyses

`ik_study` needs neither Gazebo nor the arm. It builds 500 desired poses from random
joint vectors (the same every time) with your `forward_kinematics()` and solves them
with your `inverse_kinematics()`:

```bash
ros2 run lite6_kinematics ik_study convergence
```

```bash
ros2 run lite6_kinematics ik_study nudge
```

```bash
ros2 run lite6_kinematics ik_study zero
```

They write `~/shared/lw2_study_convergence.csv` (E.1), `lw2_study_nudge.csv` (E.3) and
`lw2_study_zero.csv` (E.4). `nudge` solves the 500 poses six times and takes a minute or
two.

E.2 uses the checker, with Gazebo and your node running, once per starting
configuration:

```bash
ros2 run lite6_kinematics ik_check --ros-args -p robot:=sim -p target:=A -p start:=S1
```

then `start:=S2`, `S3` and `S4`. Each run writes its own file, `~/shared/lw2_ik_sim_A_S1.csv`
and so on.

## The files in `shared/`

| File | Written by | Contents |
|---|---|---|
| `lw2_ik_<robot>.csv` | `ik_check` | one row per desired pose: the angles the arm started from, the desired pose, the solution, the angles and the pose reached (tf, and the controller on the real arm), the errors in mm and degrees, and the result: `reached`, `refused` (no solution within 5 s) or `rejected` (a solution outside the joint ranges, never executed) |
| `lw2_ik_<robot>_<target>_<start>.csv` | `ik_check` | the same, for a run with `-p target:=` or `-p start:=` |
| `lw2_ik_<robot>_once.csv` | `ik_check -p once:=true` | the same, for the checkpoint: the desired pose is the pose the end-effector has now; appended to on every run |
| `lw2_study_<mode>.csv` | `ik_study` | one row per pose: the joint vector it came from, the status, the iterations, the solution and its error |

Do not edit these files: the logged solutions are checked against what your submitted
code computes from the logged starting angles.
