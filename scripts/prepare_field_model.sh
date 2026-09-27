#!/usr/bin/env bash
# Install the exact user-provided 3 m RMUC field mesh used by the default world.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE="$ROOT_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025_3m.stl.xz"
MESH="$ROOT_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl"
EXPECTED=f2cd3ed13481413a9dc6df15f58dc281483144d34f0a749c5b2d65280e447d16
[[ -s "$ARCHIVE" ]] || { echo "Missing bundled 3 m mesh: $ARCHIVE" >&2; exit 1; }
if [[ -s "$MESH" ]] && [[ "$(sha256sum "$MESH" | cut -d' ' -f1)" == "$EXPECTED" ]]; then
    echo "[field] 3 m mesh verified: $MESH"
    exit 0
fi
mkdir -p "$(dirname "$MESH")"
PARTIAL="$(mktemp "${MESH}.unpack.XXXXXX")"
trap 'rm -f "$PARTIAL"' EXIT
xz -dc "$ARCHIVE" > "$PARTIAL"
[[ "$(sha256sum "$PARTIAL" | cut -d' ' -f1)" == "$EXPECTED" ]] || {
    echo '[field] Bundled mesh failed SHA-256 verification' >&2
    exit 1
}
mv "$PARTIAL" "$MESH"
trap - EXIT
echo "[field] Installed 3 m mesh: $MESH"
