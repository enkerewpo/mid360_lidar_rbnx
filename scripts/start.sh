#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn upstream livox_ros_driver2's launch file (background) +
# the atlas_bridge (foreground). Trap discipline: when rbnx boot
# SIGTERMs our PGID, kill both.
#
# Layout invariant — populated by scripts/build.sh:
#   rbnx-build/ws/install/setup.bash   colcon overlay (livox_ros_driver2)
#   rbnx-build/codegen/proto_gen/      atlas_pb2.py for atlas_bridge
#
# The Mid-360 wants the host's IP in its config JSON
# (Livox/host_net_info). prepare_livox_config.sh resolves the host IP
# via `ip route get <lidar_ip>` and writes a tmp JSON. Override:
#   - LIVOX_HOST_IP  pin host IP if route resolution fails
#   - LIVOX_LIDAR_IP pin lidar IP (default reads from packaged JSON)
#   - LIVOX_MID360_CONFIG  full path to a pre-baked JSON (skips prepare)

set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

cleanup() {
    [[ -n "${LIVOX_PID:-}" ]] && kill -TERM "$LIVOX_PID" 2>/dev/null || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ROS env. Source the package-local colcon overlay (livox_ros_driver2).
ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PKG/rbnx-build/ws/install/setup.bash"
else
    echo "[mid360_lidar/start] ERROR: rbnx-build/ws/install missing — run rbnx build first" >&2
    exit 1
fi

# Path injection so atlas_bridge can find atlas_pb2 + robonix_py.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-py 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

# Resolve a config JSON with the right host_net_info IP for THIS host.
mkdir -p "$PKG/rbnx-build/data"
if [[ -z "${LIVOX_MID360_CONFIG:-}" ]]; then
    LIVOX_MID360_CONFIG="$(bash "$PKG/scripts/prepare_livox_config.sh" "$PKG")"
fi
export LIVOX_MID360_CONFIG
echo "[mid360_lidar] using config: $LIVOX_MID360_CONFIG"

# xfer_format=2 → standard PointCloud2 (XYZIT) — rtabmap/mapping consumes this.
# Override with LIVOX_XFER_FORMAT=0 (XYZRTL) or 1 (CustomMsg) if needed.
export LIVOX_XFER_FORMAT="${LIVOX_XFER_FORMAT:-2}"

echo "[mid360_lidar] launching livox_ros_driver2 (xfer_format=$LIVOX_XFER_FORMAT)…"
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
    > "$PKG/rbnx-build/data/livox.log" 2>&1 &
LIVOX_PID=$!

# atlas_bridge takes over the foreground. It waits for the lidar topic
# to start publishing, then RegisterCapability + DeclareInterface
# for lidar3d + imu, then heartbeats.
exec python3 -m mid360_driver.atlas_bridge
