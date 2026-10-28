#!/usr/bin/env bash
# Gazebo/PX4 environment only. Start bringup/startup.launch.py in another terminal.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.17.0}"
export PX4_DIR
SOURCE="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS"
PX4_EXEC="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_ROMFS="$PX4_DIR/build/px4_sitl_default/romfs_files.tar"
PROFILE="$ROOT_DIR/config/px4_sitl/22001_gz_one_drone"
EXPECTED_COMMIT=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
if [[ ! -d "$PX4_DIR" ]] || [[ "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null || true)" != "$EXPECTED_COMMIT" ]]; then
    echo 'PX4_DIR must point to the pinned PX4 1.17.0 source; see README.md.' >&2
    exit 1
fi
if ! grep -q ONE_DRONE_MAVLINK_ONLY "$SOURCE" || [[ ! -f "$PX4_EXEC" || ! -f "$PX4_ROMFS" || "$SOURCE" -nt "$PX4_ROMFS" || "$PROFILE" -nt "$PX4_ROMFS" ]]; then
    echo 'Run tools/prepare_algorithm_sim.py and rebuild PX4 v1.17.0 SITL first.' >&2
    exit 1
fi
exec bash "$ROOT_DIR/scripts/start_sim.sh" "$@"
