#!/usr/bin/env bash
###############################################################################
# Container entrypoint.
#
#   1. sets up the display (browser VNC, or an external X such as XQuartz)
#   2. sources ROS + underlay + overlay
#   3. execs whatever you asked for (default: sleep infinity)
###############################################################################
set -o pipefail

# shellcheck source=ros_setup.sh
source /etc/profile.d/10-ros-lab.sh

# --- display -----------------------------------------------------------------
# DISPLAY_MODE = auto | wslg | vnc | x11 | none
/usr/local/bin/start-display || echo "!! display setup failed - GUI apps will not work" >&2
# start-display writes the resolved DISPLAY / mode here.
if [ -s /tmp/.lab-display ]; then
    DISPLAY="$(cat /tmp/.lab-display)"
    export DISPLAY
fi
DISPLAY_RESOLVED="$(cat /tmp/.lab-display-mode 2>/dev/null || echo "${DISPLAY_MODE}")"

# --- banner ------------------------------------------------------------------
if [ "${LAB_QUIET:-0}" != "1" ]; then
    gz_state="ON"
    [ "${LAB_GAZEBO:-1}" = "0" ] && gz_state="OFF  (lab sim is blocked)"
    cat <<BANNER
--------------------------------------------------------------------------
 Robotics 02000537 - Lab 0 container      UFACTORY Lite 6 / ROS 2 Jazzy
--------------------------------------------------------------------------
  ROS_DISTRO      : ${ROS_DISTRO:-?}
  ROS_DOMAIN_ID   : ${ROS_DOMAIN_ID}
  Gazebo          : ${gz_state}
  DISPLAY         : ${DISPLAY:-<none>}   (mode: ${DISPLAY_RESOLVED})
  Your workspace  : ~/dev_ws           (src/ is ./src on the host)
  Shared folder   : ~/shared           (= ./shared on the host)

  Type 'lab' for the commands that run Tests 1-7.
--------------------------------------------------------------------------
BANNER
fi

exec "$@"
