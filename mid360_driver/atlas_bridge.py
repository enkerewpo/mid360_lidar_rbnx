#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""mid360_lidar_rbnx — atlas bridge.

Spawned by start.sh AFTER `ros2 launch livox_ros_driver2 …` is
already publishing on the host DDS bus. Our job:

1. Resolve the topic names the upstream driver uses (configurable
   via the deploy manifest's `config:` block — different deployments
   point the lidar at different IPs / different topic names).
2. Wait for the lidar topic to actually have data — proxy for "the
   driver is alive". Without this rbnx boot moves on prematurely
   and consumers connect to a silent endpoint.
3. RegisterCapability + DeclareInterface for each contract:
     primitive/lidar/lidar3d   → PointCloud2 stream
     primitive/imu/imu         → sensor_msgs/Imu stream
     primitive/lidar/lidar_snapshot — TODO (one-shot capture)
4. Heartbeat to atlas every 15 s.

Config keys read from RBNX_CONFIG_FILE (JSON written by rbnx boot):
    lidar_topic       default "/scanner/cloud"   (matches our lddc.cpp patch)
    imu_topic         default "/livox/imu"       (upstream default)
    sentinel_timeout_s   default 30.0  (how long to wait for first
                                         message before declaring the
                                         driver dead)
    capability_id     default "com.robonix.ranger.mid360_lidar"
"""
from __future__ import annotations

import json
import logging
import os
import sys
import threading
import time
from pathlib import Path

logging.basicConfig(level=os.environ.get("MID360_LOG_LEVEL", "INFO"),
                    format="[mid360] %(message)s")
log = logging.getLogger("mid360")


def _ensure_proto_gen() -> None:
    """rbnx codegen output lives at <pkg>/rbnx-build/codegen/proto_gen."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

import grpc
import atlas_pb2 as pb
import atlas_pb2_grpc as pb_grpc


def _load_config() -> dict:
    cfg_path = os.environ.get("RBNX_CONFIG_FILE", "")
    if cfg_path and Path(cfg_path).is_file():
        try:
            return json.loads(Path(cfg_path).read_text())
        except Exception as e:  # noqa: BLE001
            log.warning("failed to read %s: %s", cfg_path, e)
    return {}


def _wait_for_topic(topic: str, msg_type: str, timeout_s: float) -> bool:
    """Return True iff the topic has been published to within `timeout_s`.
    Spins a transient rclpy node; tears it down before atlas register so
    we don't double-publish on /tf etc."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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

    def _cb(_msg):
        seen.set()

    if msg_type == "PointCloud2":
        from sensor_msgs.msg import PointCloud2 as MsgCls
    elif msg_type == "Imu":
        from sensor_msgs.msg import Imu as MsgCls
    else:
        node.destroy_node()
        rclpy.shutdown()
        return False
    node.create_subscription(MsgCls, topic, _cb, qos)
    log.info("waiting for first message on %s (%s) — up to %.1fs", topic, msg_type, timeout_s)
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


def _decl_topic_out(stub, cap_id: str, contract_id: str, topic: str,
                     qos_profile: str = "best_effort") -> None:
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_ROS2,
        endpoint=topic,
        params=pb.TransportParams(ros2=pb.Ros2Params(qos_profile=qos_profile)),
    ))


def _heartbeat_loop(stub, cap_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=cap_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)


def main() -> None:
    cfg = _load_config()
    lidar_topic = cfg.get("lidar_topic", "/scanner/cloud")
    imu_topic = cfg.get("imu_topic", "/livox/imu")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
    cap_id = cfg.get("capability_id") or os.environ.get(
        "ROBONIX_CAPABILITY_ID", "com.robonix.ranger.mid360_lidar"
    )
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")

    log.info("config: lidar_topic=%s imu_topic=%s atlas=%s",
             lidar_topic, imu_topic, atlas_addr)

    if not _wait_for_topic(lidar_topic, "PointCloud2", sentinel_timeout):
        log.error("no PointCloud2 on %s within %.1fs — driver dead?", lidar_topic, sentinel_timeout)
        sys.exit(2)
    log.info("lidar alive on %s", lidar_topic)

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.AtlasStub(channel)
    pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
    md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
    try:
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/primitive/lidar",
            capability_md_path=md_path,
        ))
        _decl_topic_out(stub, cap_id, "robonix/primitive/lidar/lidar3d", lidar_topic)
        _decl_topic_out(stub, cap_id, "robonix/primitive/imu/imu",       imu_topic)
        log.info("registered cap %s → lidar3d=%s imu=%s", cap_id, lidar_topic, imu_topic)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered (re-deploy); ok", cap_id)
        else:
            log.warning("atlas registration failed: %s", e)

    threading.Thread(target=_heartbeat_loop, args=(stub, cap_id), daemon=True).start()
    log.info("ready — looping until SIGTERM")
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
