# lite6_checkers

The checkerboard of the challenge and its pieces, as Gazebo models, and the node that puts them in
the simulation.

**8 × 8 squares of 30 mm: a 240 mm × 240 mm playing area** (Challenge, §2). It has a 12 mm
frame around the playing area, which is cosmetic and not part of the specification, and a slab 10 mm
thick. The dark squares are the ones the game is played on: a1 is dark, at White's left-hand
corner, so the long dark diagonal a1–h8 runs to each player's left (§1.1).

`lab sim` and `lab sim-lite` start the board and the 24 men on their starting squares for you: they
appear a few seconds after Gazebo opens, and the camera of the Gazebo window moves to look at the robot
and the board from about 0.8 m away, instead of the 3 m of Gazebo's default view.

## The pieces

Laboratory-supplied, as in the Challenge (§2): a **disc base of Ø 24 mm × 10 mm with a central peg
of Ø 11 mm × 15 mm**. The peg is the handle: the gripper opens to 16 mm and closes on it, which a
flat disc would not allow. The base is what a pushed piece is pushed on.

| Model | What it is |
|---|---|
| `checker_white`, `checker_black` | a man, ivory or black |
| `dama_white`, `dama_black` | the *dama* (crowned man): the same piece, with a **gold cap** on the top 4 mm of the peg, which is only a mark: shape, mass and grasp are those of a man (promotion exchanges the man for one of these, §1.1) |

Solid plastic, 1200 kg/m³: **7.1 g**, centre of mass at **8 mm** above the board, so a push has to
stay below 8 mm: on the base, not on the peg. Friction 0.5 against the board's 0.6. They are free
bodies, with gravity (the lab's `xarm_gazebo` switches it on, see its `LAB.md`): a pushed piece
slides and stops where it is left, and one pushed off the board falls.

The starting position is White on ranks 1–3 and Black on ranks 6–8, on the dark squares, 12 each,
named by their square: `white_a1`, `black_h8`. `spawn_piece` puts one more anywhere, to try a
promotion or to put one back:

```bash
ros2 run lite6_checkers spawn_piece dama_white d4
```

The model origin is the centre of the underside of the base, `z` up along the peg.

## Where the board is

The beside-table world of `xarm_ros2` is hard-coded in that package, so the board and the pieces are
spawned into the running world instead of being written into it (`spawn_board` waits for Gazebo,
spawns, exits).

| | World frame |
|---|---|
| Table | top at z = 1.015, x ∈ [−0.75, 0.75], y ∈ [−1.24, −0.44] |
| Robot base | (−0.2, −0.5, 1.021), yaw −90°: it faces −y, onto the table (upstream: +90°, off it) |
| Board | centre (−0.20, −0.74), playing surface at z = 1.025, yaw −90° |

The lab's copy of `xarm_gazebo`
(`src/xarm_gazebo`, see its `LAB.md`) turns the robot to face −y, onto the table, so the board is
**in front of the robot, on its axis**, 0.24 m from the base axis to its centre: the playing area
spans 0.12 m to 0.36 m, White's side towards the robot, the slab clear of the base.

Checked against the Lite 6 kinematics with the gripper fitted (tool 83.6 mm long) and the tool
vertical: every square, every corner and every edge midpoint of the 240 mm area has an inverse
kinematic solution inside the joint limits at 5 mm (pushing), 17 mm (grasping) and 50–120 mm
(flying), for tool yaws of 0°, 30°, 45°, 60° and ±90°, with the nearest joint limit 17° away. In
Gazebo, the arm driven to a1, h1, a8 and h8 reached each within 0.3 mm of the expected position.
Closer than about 0.18 m the near squares run into the elbow and wrist limits; beyond 0.30 m the far
corners approach the 0.43 m reach. A board that fits in the reach always crosses the circle (radius
about 0.22 m at the push and grasp heights) where the forearm is vertical and joints 4 and 6 line up:
the arm can be there, but it is a wrist singularity, which matters for FR-09 and PR-09.

That is a default, not the answer to LW1's *board placement*: the reachability survey of T-02 is what
decides where the real board goes. To try another pose of the board (the robot's own pose is the one
in `src/xarm_gazebo`):

```bash
ros2 run lite6_checkers spawn_board --x -0.2 --y -0.74 --z 1.025 --yaw -1.5708
```

(`--no-pieces` for the board alone, `--no-camera` to leave the camera where Gazebo put it; the
framing is `CAMERA_TARGET_SHIFT` and `CAMERA_OFFSET` in `spawn_board.py`: the whole arm has to fit
the *vertical* field of view, which gets narrower as the window gets wider, so it cannot go much
closer than this. Remove the first board and pieces before spawning another set:
`gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "checkerboard" type: MODEL'`,
and the same for each piece, or restart the simulation.)

## The board frame

The model origin is the **centre of the playing area, on the playing surface**; the slab is below it.
`x` runs from White's side to Black's side, `y` to White's left (towards the a-file), `z` up, so the
board frame can be laid with the same yaw as a robot facing the board. Square (file `i`, rank `j`),
counted from 0, has its centre at `x = (j − 3.5)·0.03`, `y = (3.5 − i)·0.03`.

The collision surface is one flat box with its top at z = 0: a piece pushed across the board meets no
step. The dark squares are drawn 0.5 mm proud of the light plate, for the renderer only.

## Changing the model

The `model.sdf` files are generated. Change `lite6_checkers/make_board_sdf.py` or
`lite6_checkers/make_piece_sdf.py` and regenerate:

```bash
cd ~/dev_ws/src/lite6_checkers
python3 -m lite6_checkers.make_board_sdf > models/checkerboard/model.sdf
python3 -m lite6_checkers.make_piece_sdf models/
```

then `lab build`. The tests in `test/` fail if a file and its generator disagree.

## Using it without `lab`

```bash
ros2 launch xarm_gazebo lite6_beside_table_gazebo.launch.py load_controller:=true show_rviz:=false
ros2 run lite6_checkers spawn_board     # in a second terminal
```
