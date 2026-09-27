#!/usr/bin/env bash
# Start one sensor-only Gazebo/PX4 1.14.3 vehicle. No algorithm runs here.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14.3}"
PX4_COMMIT=08310a5e8ac64d02edb41523460e7dc267298deb
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
WORLD="${PX4_WORLD:-rmuc_2025_3m_vio_columns}"
if [[ "$WORLD" == rmuc_2025_field ]]; then
    echo '[sim] Obsolete world name rmuc_2025_field; using the 3 m VIO-column world.' >&2
    WORLD=rmuc_2025_3m_vio_columns
fi
AUTOSTART=4001
SPAWN_POSE="${SPAWN_POSE:-1.3,9.4,0.5,0,0,0}"

[[ -d "$PX4_DIR/.git" ]] || { echo "PX4 directory missing: $PX4_DIR" >&2; exit 1; }
[[ "$(git -C "$PX4_DIR" rev-parse HEAD)" == "$PX4_COMMIT" ]] || {
    echo "PX4_DIR must be MicoAir PX4 1.14.3 commit $PX4_COMMIT." >&2; exit 1;
}
[[ -x "$PX4_BIN" ]] || { echo "Build SITL first: cd $PX4_DIR && DONT_RUN=1 make px4_sitl_default" >&2; exit 1; }
command -v gz >/dev/null || { echo 'Gazebo gz command is missing.' >&2; exit 1; }
command -v MicroXRCEAgent >/dev/null || { echo 'MicroXRCEAgent is missing.' >&2; exit 1; }

if [[ "$WORLD" == default ]]; then
    WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/default.sdf"
else
    WORLD_SRC="$ROOT_DIR/worlds/$WORLD.sdf"
    WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf"
    [[ -f "$WORLD_SRC" ]] || { echo "World missing: $WORLD_SRC" >&2; exit 1; }
    if [[ "$WORLD" == rmuc_2025_3m_vio_columns ]]; then
        "$ROOT_DIR/scripts/prepare_field_model.sh"
    fi
    cp -f "$WORLD_SRC" "$WORLD_SDF"
fi

GEN_DIR="$ROOT_DIR/worlds/models/.gen"
rm -rf "$GEN_DIR"
for i in 1; do
    mkdir -p "$GEN_DIR/d435i_uav$i" "$GEN_DIR/x500_stereo_uav$i"
    sed "s|<name>d435i</name>|<name>d435i_uav$i</name>|" \
        "$ROOT_DIR/worlds/models/d435i/model.config" > "$GEN_DIR/d435i_uav$i/model.config"
    sed -e "s|model name='d435i'|model name='d435i_uav$i'|" \
        -e "s|d435i/base_link|d435i_uav$i/base_link|g" \
        -e "s|/vio_cam0/image|/uav$i/vio_cam0/image|g" \
        -e "s|/vio_cam1/image|/uav$i/vio_cam1/image|g" \
        -e "s|/d435i/|/uav$i/d435i/|g" \
        "$ROOT_DIR/worlds/models/d435i/model.sdf" > "$GEN_DIR/d435i_uav$i/model.sdf"
    sed "s|<name>x500_stereo</name>|<name>x500_stereo_uav$i</name>|" \
        "$ROOT_DIR/worlds/models/x500_stereo/model.config" > "$GEN_DIR/x500_stereo_uav$i/model.config"
    sed -e "s|model name='x500_stereo'|model name='x500_stereo_uav$i'|" \
        -e "s|model://d435i|model://d435i_uav$i|g" \
        -e "s|d435i/base_link|d435i_uav$i/base_link|g" \
        "$ROOT_DIR/worlds/models/x500_stereo/model.sdf" > "$GEN_DIR/x500_stereo_uav$i/model.sdf"
done

export PX4_GZ_WORLD="$WORLD"
export GZ_SIM_RESOURCE_PATH="$GEN_DIR:$ROOT_DIR/worlds/models:$PX4_DIR/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_PARTITION="${GZ_PARTITION:-one_drone_$$}"
printf 'export GZ_PARTITION=%q\n' "$GZ_PARTITION" > /tmp/one_drone_gz_env.sh


PIDS=()
finish() {
    trap - EXIT INT TERM
    trap '' INT TERM
    kill "${PIDS[@]}" 2>/dev/null || true
}
trap finish EXIT INT TERM

echo "[sim] Gazebo world=$WORLD partition=$GZ_PARTITION"
gz sim -r -s -v 2 "$WORLD_SDF" > /tmp/gz_server.log 2>&1 &
GZ_SERVER_PID=$!; PIDS+=("$GZ_SERVER_PID")
if [[ -z "${HEADLESS:-}" ]]; then
    gz sim -g -v 2 > /tmp/gz_gui.log 2>&1 & PIDS+=("$!")
fi

for _ in $(seq 1 90); do
    kill -0 "$GZ_SERVER_PID" 2>/dev/null || { tail -n 50 /tmp/gz_server.log; exit 1; }
    timeout 3 gz service -l 2>/dev/null | grep -q "/world/$WORLD/create" && break
    sleep 2
done
timeout 3 gz service -l 2>/dev/null | grep -q "/world/$WORLD/create" || {
    echo 'Gazebo world did not become ready.' >&2; exit 1;
}

cd "$PX4_DIR"
for i in 1; do
    model="x500_stereo_uav$i"
    echo "[sim] PX4 instance $i model=$model pose=$SPAWN_POSE"
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART="$AUTOSTART" \
        PX4_GZ_MODEL="$model" PX4_GZ_MODEL_POSE="$SPAWN_POSE" \
        "$PX4_BIN" -d -i "$i" > "/tmp/px4_instance_$i.log" 2>&1 &
    PIDS+=("$!")
    sleep 5
done

MicroXRCEAgent udp4 -p 8888 > /tmp/microxrce_agent.log 2>&1 & PIDS+=("$!")
echo "[sim] Ready. Source /tmp/one_drone_gz_env.sh in the algorithm terminal."
echo "[sim] Launch: PYTHONNOUSERSITE=1 ros2 launch one_drone_bringup navigation.launch.py sim:=true rviz:=true"
echo "[sim] Ctrl+C stops Gazebo, PX4 and the agent."
wait
