#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble setup for one_drone MAVROS / EGO-Planner.
# Usage: ./setup_env.sh sim | ./setup_env.sh onboard
set -euo pipefail

MODE="${1:-sim}"
if [[ "$MODE" != sim && "$MODE" != onboard ]]; then
    echo 'Usage: ./setup_env.sh [sim|onboard]' >&2
    exit 2
fi
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.17.0}"
OPENVINS_DIR="$ROOT_DIR/src/localization/open_vins"
PX4_COMMIT=d6f12ad1c4f70ad3230afd7d86e971421e02fef4

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
    python3-yaml python3-opencv python3-pip python3-dev python3-matplotlib python3-serial python3-lxml python3-future git curl build-essential cmake \
    ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp ros-humble-pcl-conversions \
    libpcl-dev libeigen3-dev libceres-dev libboost-all-dev libopencv-dev libopencv-contrib-dev \
    ros-humble-image-transport ros-humble-tf2-geometry-msgs ros-humble-mavros ros-humble-mavros-extras

# microvio reads PX4 MAVLink telemetry. Install only this protocol library;
# keep ROS/OpenCV on the distribution NumPy rather than pulling a pip NumPy.
if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
    python3 -m pip install --user --no-deps 'pymavlink>=2.4.41,<3'
fi

# OpenVINS is versioned with this repository and built in the same workspace.
if [[ ! -f "$OPENVINS_DIR/ov_msckf/src/core/VioManager.h" ]]; then
    echo "Bundled OpenVINS source is missing: $OPENVINS_DIR" >&2
    exit 1
fi

# MAVROS requires the geoid dataset even when no GPS is used.
if [[ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
    sudo bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
fi
if [[ "$MODE" == sim ]]; then
    # Harmonic is required by PX4 1.17.0; installing its ROS bridge replaces
    # a bridge built for an older Gazebo distribution.
    if ! apt-cache show gz-harmonic >/dev/null 2>&1; then
        curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | \
            sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
            sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
        sudo apt update
    fi
    sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic python3-venv \
        ccache ninja-build libssl-dev libxml2-dev libxml2-utils
    if [[ ! -d "$PX4_DIR/.git" ]]; then
        git clone --depth 1 --branch v1.17.0 \
            https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
        git -C "$PX4_DIR" checkout "$PX4_COMMIT"
    fi
    if [[ "$(git -C "$PX4_DIR" rev-parse HEAD)" != "$PX4_COMMIT" ]]; then
        echo "PX4_DIR must be the pinned PX4 1.17.0 commit $PX4_COMMIT." >&2
        exit 1
    fi
    git -C "$PX4_DIR" submodule update --init --recursive --depth 1 \
        src/drivers/gps/devices src/modules/mavlink/mavlink \
        src/lib/events/libevents src/lib/heatshrink/heatshrink Tools/simulation/gz
    if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
        echo 'Install Gazebo Harmonic and the matching ros_gz_bridge before simulation.' >&2
        exit 1
    fi
    python3 "$ROOT_DIR/tools/prepare_algorithm_sim.py" \
        --px4 "$PX4_DIR"
    # Build-time Python stays isolated from ROS/OpenCV and the user's NumPy.
    python3 -m venv --system-site-packages "$PX4_DIR/.venv"
    PYTHONNOUSERSITE=1 "$PX4_DIR/.venv/bin/python" -m pip install \
        -c "$ROOT_DIR/config/px4_sitl/python_constraints.txt" \
        -r "$PX4_DIR/Tools/setup/requirements.txt"
    (cd "$PX4_DIR" && PATH="$PX4_DIR/.venv/bin:$PATH" \
        PYTHONNOUSERSITE=1 GZ_DISTRO=harmonic DONT_RUN=1 \
        make px4_sitl_default -j "${BUILD_JOBS:-2}" \
        CMAKE_ARGS="-DCMAKE_DISABLE_FIND_PACKAGE_gazebo=TRUE -DCMAKE_DISABLE_FIND_PACKAGE_Java=TRUE")
else
    sudo apt install -y ros-humble-realsense2-camera
    python3 "$ROOT_DIR/tools/prepare_algorithm_sim.py"
fi

rosdep install --from-paths "$ROOT_DIR/src" --ignore-src -r -y
# Rebuild against the bundled source, without retaining an old OpenVINS overlay.
# Run colcon in a clean environment so install/setup.* records only ROS Humble.
env -i HOME="$HOME" USER="${USER:-$(id -un)}" PATH="/usr/local/bin:/usr/bin:/bin" \
    LANG="${LANG:-C.UTF-8}" PYTHONNOUSERSITE=1 ROOT_DIR="$ROOT_DIR" \
    MAKEFLAGS="-j${BUILD_JOBS:-2}" CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}" \
    bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash
        cd "$ROOT_DIR"
        colcon build --symlink-install --executor sequential --cmake-clean-cache'

echo "Ready: one_drone MAVROS workspace ($MODE; SITL uses PX4 1.17.0)."
echo "Source $ROOT_DIR/install/setup.bash in each terminal."
