#!/usr/bin/env bash
# Resolve a MID360_config.json path for Livox SDK: host_net_info must use a real local IP or bind fails.
# If the Livox USB NIC keeps losing a manual "ip addr add", use NetworkManager static config:
#   sudo wheatfox/scripts/nm-mid360-static-ip.sh
# (defaults: MID360_IFACE=auto picks first enx* ethernet; override MID360_IFACE=... if needed)
# - If LIVOX_HOST_IP is set, use it.
# - Else derive host IP from `ip route get <lidar_ip>` (src); lidar should be reachable.
# - On failure, fall back to the package JSON (bind may still fail; set LIVOX_HOST_IP manually).
set -euo pipefail
PKG="${1:?usage: prepare_livox_config.sh <mid360_drv package root>}"
SRC_CFG="$PKG/src/livox_ros_driver2/config/MID360_config.json"

LIDAR_IP="${LIVOX_LIDAR_IP:-}"
if [[ -z "$LIDAR_IP" ]] && [[ -f "$SRC_CFG" ]]; then
  LIDAR_IP="$(python3 -c "import json; print(json.load(open('$SRC_CFG'))['lidar_configs'][0]['ip'])" 2>/dev/null || echo 192.168.1.161)"
fi

HOST_IP="${LIVOX_HOST_IP:-}"
if [[ -z "$HOST_IP" ]] && command -v ip >/dev/null 2>&1 && [[ -n "$LIDAR_IP" ]]; then
  HOST_IP="$(ip -4 route get "$LIDAR_IP" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") { print $(i + 1); exit }}' || true)"
fi

if [[ -z "$HOST_IP" ]]; then
  echo "[mid360_drv] could not resolve host IP (set LIVOX_HOST_IP or ensure 'ip route get' works for lidar $LIDAR_IP); using: $SRC_CFG" >&2
  echo "$SRC_CFG"
  exit 0
fi

python3 - "$SRC_CFG" "$HOST_IP" <<'PY'
import json, os, sys, tempfile

src, ip = sys.argv[1], sys.argv[2]
fd, path = tempfile.mkstemp(prefix="mid360_livox_", suffix=".json")
os.close(fd)
with open(src) as f:
    d = json.load(f)
for k in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
    d["MID360"]["host_net_info"][k] = ip
with open(path, "w") as f:
    json.dump(d, f, indent=2)
print(path)
PY
