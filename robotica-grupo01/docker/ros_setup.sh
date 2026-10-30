# shellcheck shell=bash
###############################################################################
# Lab 0 - Step 7: "Configure your shell"
#
# Sourced from ~/.bashrc (interactive shells) and from the entrypoint
# (non-interactive ones), so `docker exec`, `docker compose run` and the
# container's main process all see the same environment.
#
# Do not edit this inside the container - it is baked into the image.
# Put your own additions in ~/dev_ws/.bashrc_local (sourced at the end).
###############################################################################

# A login shell reaches this file twice (once via /etc/profile.d, once via
# ~/.bashrc). Sourcing ROS twice only duplicates AMENT_PREFIX_PATH entries, but
# colcon complains about it, so do it once.
if [ -n "${LAB_ROS_SETUP_DONE:-}" ]; then
    return 0 2>/dev/null || true
fi
export LAB_ROS_SETUP_DONE=1

# --- ROS 2 core --------------------------------------------------------------
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# --- underlay: xarm_ros2, built into the image -------------------------------
if [ -f /opt/xarm_ws/install/setup.bash ]; then
    source /opt/xarm_ws/install/setup.bash
fi

# --- overlay: your own workspace, built with `lab build` ---------------------
if [ -f "${HOME}/dev_ws/install/setup.bash" ]; then
    source "${HOME}/dev_ws/install/setup.bash"
elif [ -n "$(ls -A "${HOME}/dev_ws/src" 2>/dev/null)" ]; then
    # src/ has packages but nothing is built: `ros2 run` would say "package
    # not found". Say why, in interactive shells only.
    case $- in *i*)
        echo "!! your packages in ~/dev_ws/src are not built yet - run:  lab build" >&2 ;;
    esac
fi

# --- ROS_DOMAIN_ID -----------------------------------------------------------
# Set it in .env on the host. On the lab LAN this keeps your nodes apart from
# your classmates'. It is not optional - use the number you were given.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# Deliberately NOT setting ROS_LOCALHOST_ONLY: it is deprecated in Jazzy and
# setting it at all - even to 0, its default - makes every single node print
# two warning lines. Use ROS_AUTOMATIC_DISCOVERY_RANGE if you ever need to
# restrict discovery.

# --- display -----------------------------------------------------------------
# The entrypoint resolves DISPLAY once (WSLg / VNC / external X) and drops the
# answer here, so every `docker exec` terminal inherits the same one.
if [ -s /tmp/.lab-display ]; then
    export DISPLAY="$(cat /tmp/.lab-display)"
    if [ "$(cat /tmp/.lab-display-mode 2>/dev/null)" = "wslg" ]; then
        export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
        export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
    fi
fi

# --- rendering ---------------------------------------------------------------
# No GPU passthrough in Docker Desktop, so Mesa's software rasteriser it is.
# (Handout, Test 6: "export LIBGL_ALWAYS_SOFTWARE=1".)
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
# Qt in a container has no accessibility bus; silences a wall of warnings.
export NO_AT_BRIDGE=1

# --- Gazebo ------------------------------------------------------------------
export GZ_VERSION="${GZ_VERSION:-harmonic}"
# Keep downloaded Fuel models on a volume so the slow first launch happens once.
export GZ_PARTITION="${GZ_PARTITION:-lab$(id -u)}"

# --- shell comfort -----------------------------------------------------------
# History on a volume, so it survives `lab down` / container recreation.
export HISTFILE="${HOME}/.ros/bash_history"
export HISTSIZE=5000
export HISTFILESIZE=20000
shopt -s histappend 2>/dev/null || true

if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi
if [ -f /usr/share/bash-completion/bash_completion ]; then
    source /usr/share/bash-completion/bash_completion
fi

alias ws='cd ~/dev_ws'
alias cb='lab build'
alias src='source ~/dev_ws/install/setup.bash'

# --- your own additions ------------------------------------------------------
if [ -f "${HOME}/dev_ws/.bashrc_local" ]; then
    source "${HOME}/dev_ws/.bashrc_local"
fi
