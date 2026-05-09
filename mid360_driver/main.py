#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""mid360_lidar_rbnx — Livox MID-360 lidar primitive (capability_id=mid360_lidar).

Owns `robonix/primitive/lidar/*`. The MID-360 also publishes `/livox/imu`
as a side-effect of the same upstream launch, but the IMU contract
surface lives in a SEPARATE package (`mid360_imu_rbnx`) per the
"one primitive namespace = one package" invariant.

Lifecycle:
    on_init  — parse cfg → spawn livox launch → wait for first PointCloud2
               → declare ros2 topic_out for primitive/lidar/lidar3d.
    on_shutdown — kill livox subprocess.

Config (from manifest's `config:` block, delivered via Driver(CMD_INIT)):
    lidar_topic        default "/scanner/cloud"  (matches our lddc.cpp patch)
    lidar_ip           default 192.168.1.161
    host_ip            default auto from `ip route get <lidar_ip>`
    xfer_format        default 2  (PointCloud2 XYZIT — rtabmap-friendly)
    publish_freq       default 10.0
    frame_id           default "livox_frame"
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Capability, Ok, Err

logging.basicConfig(
    level=os.environ.get("MID360_LOG_LEVEL", "INFO"),
    format="[mid360] %(message)s",
)
log = logging.getLogger("mid360")

cap = Capability(id="mid360_lidar", namespace="robonix/primitive/lidar")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_livox_proc: subprocess.Popen | None = None


# ── livox subprocess management ──────────────────────────────────────────
def _resolve_livox_config(cfg: dict) -> str:
    """Generate a Livox MID360_config.json with the right host_net_info.
    Returns the absolute path to the JSON to feed the upstream launch."""
    src_cfg = _pkg_root / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
    if not src_cfg.is_file():
        raise RuntimeError(f"packaged config missing: {src_cfg}")

    lidar_ip = str(cfg.get("lidar_ip") or os.environ.get("LIVOX_LIDAR_IP") or "")
    if not lidar_ip:
        try:
            data = json.loads(src_cfg.read_text())
            lidar_ip = data["lidar_configs"][0]["ip"]
        except Exception:  # noqa: BLE001
            lidar_ip = "192.168.1.161"

    host_ip = str(cfg.get("host_ip") or os.environ.get("LIVOX_HOST_IP") or "")
    if not host_ip:
        try:
            out = subprocess.run(
                ["ip", "-4", "route", "get", lidar_ip],
                capture_output=True, text=True, timeout=2, check=False,
            )
            parts = out.stdout.split()
            if "src" in parts:
                host_ip = parts[parts.index("src") + 1]
        except Exception:  # noqa: BLE001
            pass

    if not host_ip:
        log.warning(
            "could not resolve host IP (set host_ip in config or LIVOX_HOST_IP); "
            "using packaged JSON %s", src_cfg,
        )
        return str(src_cfg)

    out_path = _pkg_root / "rbnx-build" / "data" / "MID360_config.gen.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = json.loads(src_cfg.read_text())
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        data["MID360"]["host_net_info"][key] = host_ip
    if cfg.get("lidar_ip"):
        data["lidar_configs"][0]["ip"] = lidar_ip
    out_path.write_text(json.dumps(data, indent=2))
    log.info("livox config: lidar=%s host=%s → %s", lidar_ip, host_ip, out_path)
    return str(out_path)


def _spawn_livox(cfg: dict) -> None:
    global _livox_proc
    config_path = _resolve_livox_config(cfg)
    env = dict(os.environ)
    env["LIVOX_MID360_CONFIG"] = config_path
    env["LIVOX_XFER_FORMAT"] = str(cfg.get("xfer_format", 2))
    env["LIVOX_PUBLISH_FREQ"] = str(cfg.get("publish_freq", 10.0))
    env["LIVOX_FRAME_ID"] = str(cfg.get("frame_id", "livox_frame"))

    log_path = _pkg_root / "rbnx-build" / "data" / "livox.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning livox driver (xfer_format=%s) → %s",
             env["LIVOX_XFER_FORMAT"], log_path)
    _livox_proc = subprocess.Popen(
        ["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
        env=env,
        stdout=log_fh, stderr=log_fh,
        start_new_session=True,
    )


def _kill_livox() -> None:
    p = _livox_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── sentinel: wait for first PointCloud2 ─────────────────────────────────
def _wait_for_pointcloud(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("mid360_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(PointCloud2, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first PointCloud2 on %s — up to %.1fs", topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


# ── lifecycle handlers ───────────────────────────────────────────────────
@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn livox, wait for cloud, declare topic."""
    lidar_topic = cfg.get("lidar_topic", "/scanner/cloud")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_livox(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn livox failed: {e}")

    if not _wait_for_pointcloud(lidar_topic, sentinel_timeout):
        _kill_livox()
        return Err(f"no PointCloud2 on {lidar_topic} within {sentinel_timeout:.1f}s")

    cap.declare_ros2(
        "robonix/primitive/lidar/lidar3d",
        topic=lidar_topic,
        qos="best_effort",
    )
    log.info("init complete: lidar3d=%s", lidar_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_livox()
    return Ok()


if __name__ == "__main__":
    cap.run()
