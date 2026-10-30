# syntax=docker/dockerfile:1
###############################################################################
# Lab 0 - Development Environment Setup
# Robotics (02000537) - 2026/2027
# Robot: UFACTORY Lite 6 . Middleware: ROS 2 Jazzy Jalisco
#
# This is "Path C - Docker container" of the handout. Every numbered step below
# maps 1:1 onto a step in Part 1 of Lab 0, so you can read this file as the
# handout and check what was actually done.
#
# Layout produced by this image:
#   /opt/ros/jazzy        ROS 2 Jazzy (Steps 4, 5, 6)
#   /opt/xarm_ws          xarm_ros2, cloned --recursive and built (Step 8)
#                         -> this is the "underlay". Read-only in practice.
#   /home/ros/dev_ws      YOUR workspace (the "overlay"). Lives on a volume;
#                         its src/ is a normal folder on your Windows disk.
###############################################################################
FROM ubuntu:24.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Branch/tag of xArm-Developer/xarm_ros2 to build into the underlay.
ARG XARM_REF=jazzy
# Step 8.4 of the handout: limit parallelism so the build cannot eat all RAM.
ARG COLCON_JOBS=2

###############################################################################
# Make apt survive a flaky link. This build runs for the best part of an hour
# and fetches a few GB; losing all of it because one mirror connection dropped
# is not a good trade. Needs no network itself, so it goes first.
###############################################################################
RUN printf '%s\n' \
      'Acquire::Retries "5";' \
      'Acquire::http::Timeout "30";' \
      'Acquire::https::Timeout "30";' \
      > /etc/apt/apt.conf.d/80-lab-retries

###############################################################################
# Step 1 - Set the locale
###############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales ca-certificates curl gnupg \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

###############################################################################
# Step 2 - Enable the `universe` repository
###############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Step 3 - Add the ROS 2 apt repository
#
# The handout says the official page is the source of truth here, because this
# is the part upstream keeps changing. The current official method is the
# `ros2-apt-source` .deb; if that cannot be fetched we fall back to the older
# keyring + sources.list method, which still works.
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
###############################################################################
RUN set -eux; \
    . /etc/os-release; \
    ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
           | grep -F 'tag_name' | head -n1 | cut -d'"' -f4 || true)"; \
    if [ -n "${ver:-}" ] && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"; then \
        echo ">> ROS 2 apt source: using ros2-apt-source ${ver}"; \
        apt-get update && apt-get install -y /tmp/ros2-apt-source.deb; \
        rm -f /tmp/ros2-apt-source.deb; \
    else \
        echo ">> ROS 2 apt source: falling back to manual keyring"; \
        curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             -o /usr/share/keyrings/ros-archive-keyring.gpg; \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
             > /etc/apt/sources.list.d/ros2.list; \
    fi; \
    apt-get update; \
    rm -rf /var/lib/apt/lists/*

###############################################################################
# Step 4 - Install ROS 2 Jazzy and the build tools
# (no --no-install-recommends here: RViz/rqt pull GUI bits in via recommends)
###############################################################################
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-jazzy-desktop ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Step 5 - Install MoveIt 2
###############################################################################
RUN apt-get update && apt-get install -y ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Step 6 - Install Gazebo (Harmonic - NOT "Gazebo Classic")
#
# Always installed: whether Gazebo *runs* is a runtime switch (LAB_GAZEBO),
# not a build switch. Disk is cheap; CPU while you are driving the real arm
# is not.
###############################################################################
ENV GZ_VERSION=harmonic
RUN apt-get update && apt-get install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Tooling you will actually want inside the container
#   graphviz            -> Test 4 (`ros2 run tf2_tools view_frames`) needs `dot`
#   mesa-utils          -> `glxinfo` when the GUI misbehaves
#   xvfb/x11vnc/novnc   -> the browser-based display fallback (DISPLAY_MODE=vnc)
###############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
        bash-completion \
        dbus-x11 \
        fluxbox \
        gdb \
        git \
        gnupg2 \
        graphviz \
        iproute2 \
        iputils-ping \
        less \
        libgl1-mesa-dri \
        mesa-utils \
        nano \
        net-tools \
        novnc \
        openssh-client \
        python3-pip \
        python3-venv \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-ros2controlcli \
        sudo \
        tmux \
        tree \
        unzip \
        vim \
        websockify \
        wget \
        x11-apps \
        x11-utils \
        x11vnc \
        xauth \
        xterm \
        xvfb \
    && ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Unprivileged user. Ubuntu 24.04 ships a stock `ubuntu` user on uid 1000,
# which collides with ours - remove it first.
###############################################################################
RUN set -eux; \
    if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu || true; fi; \
    groupadd -g "${USER_GID}" "${USERNAME}" 2>/dev/null || true; \
    useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USERNAME}"; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/90-${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/90-${USERNAME}"

# rosdep, once per machine (Step 4)
RUN rosdep init || true

###############################################################################
# Step 8 - xarm_ros2, built as an underlay at /opt/xarm_ws
#
# NOTE the --recursive: the C++ SDK that talks to the robot is a git submodule
# and the build fails without it.
###############################################################################
RUN mkdir -p /opt/xarm_ws/src && chown -R "${USERNAME}:${USERNAME}" /opt/xarm_ws

USER ${USERNAME}
RUN rosdep update

# Bump this to force a fresh clone+build of xarm_ros2 on the next image build.
ARG XARM_CACHEBUST=2026-09-17
RUN git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b "${XARM_REF}" \
        /opt/xarm_ws/src/xarm_ros2

# Step 8.3 - rosdep pulls the ROS dependencies (it calls sudo apt itself)
RUN sudo apt-get update \
    && rosdep install --from-paths /opt/xarm_ws/src --ignore-src --rosdistro jazzy -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Step 8.4 - the slow part (15-30 min)
RUN source /opt/ros/jazzy/setup.bash \
    && cd /opt/xarm_ws \
    && colcon build \
         --parallel-workers "${COLCON_JOBS}" \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/xarm_ws/build /opt/xarm_ws/log

###############################################################################
# Step 7 - shell configuration, plus the lab helper
###############################################################################
USER root
COPY docker/ros_setup.sh    /etc/profile.d/10-ros-lab.sh
COPY docker/entrypoint.sh   /usr/local/bin/entrypoint
COPY docker/start-display   /usr/local/bin/start-display
COPY docker/lab             /usr/local/bin/lab
RUN chmod +x /usr/local/bin/entrypoint /usr/local/bin/start-display /usr/local/bin/lab \
    && chmod 0644 /etc/profile.d/10-ros-lab.sh

USER ${USERNAME}
# `docker exec ... bash` does not run the entrypoint, so interactive shells get
# their ROS environment from ~/.bashrc - exactly like Step 7 of the handout.
RUN { \
      echo ''; \
      echo '# --- Lab 0 (Step 7) -------------------------------------------'; \
      echo 'source /etc/profile.d/10-ros-lab.sh'; \
      echo '# --------------------------------------------------------------'; \
    } >> "/home/${USERNAME}/.bashrc" \
    && mkdir -p "/home/${USERNAME}/dev_ws/src" \
                "/home/${USERNAME}/shared" \
                "/home/${USERNAME}/.gz" \
                "/home/${USERNAME}/.ros"

WORKDIR /home/${USERNAME}/dev_ws

ENV LAB_GAZEBO=1 \
    DISPLAY_MODE=auto \
    ROS_DOMAIN_ID=0 \
    COLCON_JOBS=2

ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["sleep", "infinity"]
