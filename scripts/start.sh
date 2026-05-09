#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn the mid360_lidar capability process. The Livox ROS driver is NOT
# launched here — it's spawned inside the cap's on_init handler, after
# rbnx boot delivers config via Driver(CMD_INIT).
#
# Layout invariant (populated by scripts/build.sh):
#   rbnx-build/ws/install/setup.bash   colcon overlay (livox_ros_driver2)
#   rbnx-build/codegen/proto_gen/      atlas_pb2.py + robonix_contracts_pb2*
set -euo pipefail
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${COLCON_TRACE:=}"
export AMENT_TRACE_SETUP_FILES COLCON_TRACE
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

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

# robonix_api is on the host at `rbnx path robonix-api`.
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi

exec python3 -m mid360_driver.main
