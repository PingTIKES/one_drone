#!/usr/bin/env bash
# Gazebo/PX4 environment ONLY. Start navigation.launch.py in another terminal.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14.3}"
export PX4_DIR
SOURCE="$PX4_DIR/src/modules/uxrce_dds_client/uxrce_dds_client.cpp"
PX4_EXEC="$PX4_DIR/build/px4_sitl_default/bin/px4"
EXPECTED_COMMIT=08310a5e8ac64d02edb41523460e7dc267298deb
if [[ ! -d "$PX4_DIR" ]] || [[ "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null || true)" != "$EXPECTED_COMMIT" ]]; then
    echo 'PX4_DIR must point to the pinned MicoAir PX4 1.14.3 source; see README.md.' >&2
    exit 1
fi
if ! grep -q RM27_SIM_CLOCK "$SOURCE" || [[ ! -f "$PX4_EXEC" || "$SOURCE" -nt "$PX4_EXEC" ]]; then
    echo 'Run tools/prepare_algorithm_sim.py and rebuild MicoAir PX4 v1.14.3 SITL first.' >&2
    exit 1
fi
export RM27_SIM_CLOCK=1
exec "$ROOT_DIR/scripts/start_sim.sh" "$@"
