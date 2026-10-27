# robotics-26-27
repository for my robotics class

## Project structure

```bash
robotica-grupo01/
    config/robot.yaml               # ip, home, tool offset, limits, etc
    lite6/
        __init__.py
        session.py                  # safe_session()
        kinematics.py               # LW1: fk_lite6, pose_to_T, T_to_pose, workspace_sample
        ik.py                       # LW2: ik_lite6
        velocity.py                 # LW3: jacobian, manipulability, rrmc_step, stream
        trajectory.py               # LW4: traj_*, slide_piece, pick_and_place
        game.py                     # Final Challenge: game state, rules
    scripts/                        # one script per experiment
    tests/                          # unit tests
    logs/
    requirements.txt
    README.md
```

## Setting things up with uv

```bash
uv venv --python 3.12.0
uv pip install xarm-python-sdk numpy scipy matplotlib pytest pyyaml lite6
```

## Runnign a script with uv

```bash
uv run python -m scripts.[script_name]
```