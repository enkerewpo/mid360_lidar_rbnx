#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the atlas bridge. THIS SCRIPT DOES NOT START THE LIVOX DRIVER —
# the upstream `livox_ros_driver2` is spawned inside atlas_bridge's
# `Driver(CMD_INIT)` handler, AFTER `rbnx boot` calls in with config.
# Doing it that way means atlas only ever sees the lidar3d/imu data
# interfaces declared once we've confirmed the driver is publishing,
# and `rbnx boot`'s manifest config is the single source of truth for
# host_net_info IPs / xfer_format / topics.
#
# Layout invariant — populated by scripts/build.sh:
#   rbnx-build/ws/install/setup.bash   colcon overlay (livox_ros_driver2)
#   rbnx-build/codegen/proto_gen/      atlas_pb2.py for atlas_bridge

set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ROS env. Source the package-local colcon overlay so `ros2 launch
# livox_ros_driver2 …` (invoked by atlas_bridge) finds the driver.
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

exec python3 -m mid360_driver.atlas_bridge
