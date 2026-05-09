#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: colcon-build the vendored livox_ros_driver2, then
# rbnx codegen so atlas_bridge can import atlas_pb2.
#
# Vendored under src/livox_ros_driver2 — includes our local fixes
# on top of upstream Livox-SDK/livox_ros_driver2 (config IPs +
# topic name + xfer_format default). See src/livox_ros_driver2.patch
# for the diff against upstream HEAD at the time of vendoring.
#
# Output goes into rbnx-build/{ws/install,codegen}/. start.sh
# sources rbnx-build/ws/install/setup.bash before launching.
set -euo pipefail
# ROS 2 humble setup.bash references AMENT_TRACE_SETUP_FILES /
# COLCON_TRACE without an [ -z ] guard, so under `set -u` they
# trip "unbound variable". Initialise to empty.
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${COLCON_TRACE:=}"
export AMENT_TRACE_SETUP_FILES COLCON_TRACE
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[mid360_lidar/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink the vendored source into a scratch ws so colcon can find it
# without polluting our src/ tree with build artefacts.
ln -snf "$PKG/src/livox_ros_driver2" "$PKG/rbnx-build/ws/src/livox_ros_driver2"

# Source ROS env. Distro overridable for non-humble setups.
ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "[mid360_lidar/build] colcon build (livox_ros_driver2)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
cd "$PKG"

# Robonix codegen for atlas_bridge.py imports.
FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[mid360_lidar/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[mid360_lidar/build] done."
