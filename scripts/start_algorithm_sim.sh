#!/usr/bin/env bash
# Gazebo/PX4 environment only. Start bringup/startup.launch.py in another terminal.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14.3}"
export PX4_DIR
SOURCE="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS"
PX4_EXEC="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_ROMFS="$PX4_DIR/build/px4_sitl_default/romfs_files.tar"
EXPECTED_COMMIT=08310a5e8ac64d02edb41523460e7dc267298deb
if [[ ! -d "$PX4_DIR" ]] || [[ "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null || true)" != "$EXPECTED_COMMIT" ]]; then
    echo 'PX4_DIR must point to the pinned MicoAir PX4 1.14.3 source; see README.md.' >&2
    exit 1
fi
if ! grep -q RM27_MAVROS_ONLY "$SOURCE" || [[ ! -f "$PX4_EXEC" || ! -f "$PX4_ROMFS" || "$SOURCE" -nt "$PX4_ROMFS" ]]; then
    echo 'Run tools/prepare_algorithm_sim.py and rebuild MicoAir PX4 v1.14.3 SITL first.' >&2
    exit 1
fi
export RM27_SIM_CLOCK=1
exec bash "$ROOT_DIR/scripts/start_sim.sh" "$@"
