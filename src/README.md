# `src/` — your workspace

This folder **is** `~/dev_ws/src` inside the container. It is an ordinary folder
on your Mac: edit it with VS Code, back it up, put it in git.

Nothing else inside the container is permanent. Anything you want to keep goes
here, or in `../shared/`.

```
lab build        # inside the container: colcon build, from ~/dev_ws
```

The build output (`build/`, `install/`, `log/`) is deliberately *not* here — it
lives in a Docker volume, where compilation is fast and the macOS file-sharing layer cannot trip
over symlinks. That is why this folder stays clean.

## Starting a package

```bash
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python my_lite6_demo \
  --dependencies rclpy geometry_msgs tf2_ros
lab build
source ~/dev_ws/install/setup.bash
```

## Modifying xarm_ros2 itself

`xarm_ros2` is pre-built inside the image at `/opt/xarm_ws` — you do not need a
copy to *use* it. If you need to *change* it (custom MoveIt config, new launch
file):

```bash
lab overlay-xarm   # copies it into ~/dev_ws/src/xarm_ros2, i.e. here
lab build
```

Your copy then shadows the image's, because the overlay is sourced last.

This folder already carries one such overlay, of the package `xarm_gazebo` alone: it turns the
simulated robot to face the checkerboard of `lite6_checkers`. `xarm_gazebo/LAB.md` says what
differs from the image.
