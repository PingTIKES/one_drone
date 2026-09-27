#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble setup for the one_drone PX4 1.14.3 Nav2 stack.
# Usage: ./setup_env.sh sim | ./setup_env.sh onboard
set -euo pipefail

MODE="${1:-sim}"
if [[ "$MODE" != sim && "$MODE" != onboard ]]; then
    echo 'Usage: ./setup_env.sh [sim|onboard]' >&2
    exit 2
fi
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14.3}"
OV_WS="${OV_WS:-$HOME/catkin_ws_ov}"
PX4_COMMIT=08310a5e8ac64d02edb41523460e7dc267298deb
PX4_MSGS_COMMIT=ffb6e80e1c17e5714395611a020c282a87af8fa4
OPENVINS_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'Install ROS 2 Humble before running setup_env.sh.' >&2
    exit 1
fi
# ROS/colcon setup scripts read optional variables without nounset guards.
set +u
source /opt/ros/humble/setup.bash
set -u

sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-numpy python3-pil \
    python3-yaml python3-opencv python3-pip git curl build-essential cmake \
    ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2 ros-humble-nav2-bringup

if [[ ! -d "$OV_WS/src/open_vins/.git" ]]; then
    mkdir -p "$OV_WS/src"
    git clone https://github.com/rpng/open_vins.git "$OV_WS/src/open_vins"
    git -C "$OV_WS/src/open_vins" checkout "$OPENVINS_COMMIT"
fi
if [[ "$(git -C "$OV_WS/src/open_vins" rev-parse HEAD)" != "$OPENVINS_COMMIT" ]]; then
    echo "OpenVINS must be the tested commit $OPENVINS_COMMIT." >&2
    exit 1
fi

mkdir -p "$ROOT_DIR/third_party"
if [[ ! -d "$ROOT_DIR/third_party/px4_msgs/.git" ]]; then
    git clone --branch release/1.14 https://github.com/PX4/px4_msgs.git \
        "$ROOT_DIR/third_party/px4_msgs"
    git -C "$ROOT_DIR/third_party/px4_msgs" checkout "$PX4_MSGS_COMMIT"
fi
if [[ "$(git -C "$ROOT_DIR/third_party/px4_msgs" rev-parse HEAD)" != "$PX4_MSGS_COMMIT" ]]; then
    echo "px4_msgs must be release/1.14 commit $PX4_MSGS_COMMIT." >&2
    exit 1
fi
if [[ ! -e "$ROOT_DIR/src/px4_msgs" ]]; then
    ln -s ../third_party/px4_msgs "$ROOT_DIR/src/px4_msgs"
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    AGENT_DIR="${AGENT_DIR:-$HOME/Micro-XRCE-DDS-Agent}"
    if [[ ! -d "$AGENT_DIR/.git" ]]; then
        git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
    fi
    cmake -S "$AGENT_DIR" -B "$AGENT_DIR/build"
    cmake --build "$AGENT_DIR/build" -j "$(nproc)"
    sudo cmake --install "$AGENT_DIR/build"
    sudo ldconfig
fi

if [[ "$MODE" == sim ]]; then
    if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
        sudo apt install -y ros-humble-ros-gzgarden
    fi
    if [[ ! -d "$PX4_DIR/.git" ]]; then
        git clone --recursive --branch micoair743-v1.14.3 \
            https://github.com/Minderring/PX4-Autopilot.git "$PX4_DIR"
        git -C "$PX4_DIR" checkout "$PX4_COMMIT"
    fi
    if [[ "$(git -C "$PX4_DIR" rev-parse HEAD)" != "$PX4_COMMIT" ]]; then
        echo "PX4_DIR must be the pinned MicoAir PX4 1.14.3 commit $PX4_COMMIT." >&2
        exit 1
    fi
    git -C "$PX4_DIR" submodule update --init --recursive
    if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
        echo 'Install Gazebo Garden and the matching ros_gz_bridge before simulation.' >&2
        exit 1
    fi
    python3 "$ROOT_DIR/tools/prepare_algorithm_sim.py" \
        --px4 "$PX4_DIR" --openvins "$OV_WS/src/open_vins"
    (cd "$PX4_DIR" && bash Tools/setup/ubuntu.sh --no-nuttx && DONT_RUN=1 make px4_sitl_default)
else
    sudo apt install -y ros-humble-realsense2-camera
    python3 "$ROOT_DIR/tools/prepare_algorithm_sim.py" --openvins "$OV_WS/src/open_vins"
fi

rosdep install --from-paths "$ROOT_DIR/src" --ignore-src -r -y
(cd "$OV_WS" && colcon build --packages-select ov_core ov_init ov_msckf ov_eval)
set +u
source "$OV_WS/install/setup.bash"
set -u
(cd "$ROOT_DIR" && colcon build --symlink-install)

echo "Ready: one_drone PX4 1.14.3 workspace ($MODE)."
echo "Source $OV_WS/install/setup.bash and $ROOT_DIR/install/setup.bash in each terminal."
