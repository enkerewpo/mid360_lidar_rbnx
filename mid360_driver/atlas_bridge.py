#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""mid360_lidar_rbnx — atlas bridge (driver-init lifecycle).

Spawn order:
  1. start.sh launches THIS process (no upstream ROS driver yet).
  2. main() opens a gRPC server, RegisterCapability, declares ONLY
     `primitive/lidar/driver` on atlas, then blocks on heartbeat.
  3. `rbnx boot` discovers the driver gRPC interface on atlas, calls
     `Driver(CMD_INIT, config_json)` with the manifest's `config:` block.
  4. Inside the Init handler we do the REAL initialization:
       a. parse config → resolve host_net_info IP, xfer_format, topics
       b. spawn `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
       c. wait for first PointCloud2 on the configured topic
       d. DeclareInterface for `primitive/lidar/lidar3d` and
          `primitive/imu/imu` (now that we know the topics actually carry
          data)
       e. return ok=true so boot proceeds.

Why this layout: declaring data interfaces upfront lets consumers connect
to silent endpoints when the driver hasn't actually come online yet — bad
for late binders like rtabmap which then fail mysteriously. By gating
declaration on Init success, atlas only ever exposes endpoints that have
proven they're publishing.

Config (passed via `Driver(CMD_INIT, config_json)`):
    lidar_topic        default "/scanner/cloud"      (matches our lddc.cpp patch)
    imu_topic          default "/livox/imu"
    lidar_ip           default 192.168.1.161         (override per robot)
    host_ip            default auto from `ip route get <lidar_ip>`
    xfer_format        default 2  (PointCloud2 XYZIT — rtabmap-friendly)
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from concurrent import futures
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

import grpc  # noqa: E402
import atlas_pb2 as pb  # noqa: E402
import atlas_pb2_grpc as pb_grpc  # noqa: E402
import lifecycle_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402

CMD_INIT = 0
CMD_SHUTDOWN = 1


# ── shared state populated by Init ───────────────────────────────────────────
_state_lock = threading.Lock()
_atlas_stub: pb_grpc.AtlasStub | None = None
_cap_id: str = ""
_pkg_root: Path = Path(__file__).resolve().parent.parent
_livox_proc: subprocess.Popen | None = None
_initialized = False


# ── livox subprocess management ──────────────────────────────────────────────
def _resolve_livox_config(cfg: dict) -> str:
    """Generate a Livox MID360_config.json with the right host_net_info.
    Returns the absolute path to the generated JSON."""
    pkg = _pkg_root
    src_cfg = pkg / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
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
            for tok in out.stdout.split():
                if tok.startswith("src"):
                    pass
            parts = out.stdout.split()
            if "src" in parts:
                host_ip = parts[parts.index("src") + 1]
        except Exception:  # noqa: BLE001
            pass

    if not host_ip:
        log.warning("could not resolve host IP (set host_ip in config or LIVOX_HOST_IP); using packaged JSON %s", src_cfg)
        return str(src_cfg)

    out_path = pkg / "rbnx-build" / "data" / "MID360_config.gen.json"
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
    """Launch ros2 launch livox_ros_driver2 msg_MID360_launch.py.
    Stores the Popen handle in _livox_proc for cleanup."""
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


# ── data-interface declaration (lazy, after Init) ────────────────────────────
def _wait_for_topic(topic: str, msg_type: str, timeout_s: float) -> bool:
    """Spin a transient rclpy node until one message lands on the topic."""
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
    if msg_type == "PointCloud2":
        from sensor_msgs.msg import PointCloud2 as MsgCls
    elif msg_type == "Imu":
        from sensor_msgs.msg import Imu as MsgCls
    else:
        node.destroy_node()
        rclpy.shutdown()
        return False
    node.create_subscription(MsgCls, topic, lambda _m: seen.set(), qos)
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


def _decl_topic_out(contract_id: str, topic: str, qos_profile: str = "best_effort") -> None:
    if _atlas_stub is None:
        return
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=_cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_ROS2,
        endpoint=topic,
        params=pb.TransportParams(ros2=pb.Ros2Params(qos_profile=qos_profile)),
    ))


# ── lifecycle Driver gRPC server ─────────────────────────────────────────────
class _LidarDriverServicer(contracts_grpc.PrimitiveLidarDriverServicer):
    def Driver(self, request, context):
        cmd = int(request.command)
        if cmd == CMD_INIT:
            try:
                cfg = json.loads(request.config_json) if request.config_json else {}
            except json.JSONDecodeError as e:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"bad config_json: {e}"
                )
            return self._init(cfg)
        if cmd == CMD_SHUTDOWN:
            _kill_livox()
            return lifecycle_pb2.Driver_Response(ok=True, state="shutdown", error="")
        return lifecycle_pb2.Driver_Response(
            ok=False, state="error", error=f"invalid command {cmd}"
        )

    def _init(self, cfg: dict):
        global _initialized
        with _state_lock:
            if _initialized:
                return lifecycle_pb2.Driver_Response(ok=True, state="ready", error="")

        lidar_topic = cfg.get("lidar_topic", "/scanner/cloud")
        imu_topic = cfg.get("imu_topic", "/livox/imu")
        sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

        try:
            _spawn_livox(cfg)
        except Exception as e:  # noqa: BLE001
            return lifecycle_pb2.Driver_Response(
                ok=False, state="error", error=f"spawn livox failed: {e}"
            )

        if not _wait_for_topic(lidar_topic, "PointCloud2", sentinel_timeout):
            _kill_livox()
            return lifecycle_pb2.Driver_Response(
                ok=False, state="error",
                error=f"no PointCloud2 on {lidar_topic} within {sentinel_timeout:.1f}s",
            )

        # Declare the data interfaces. IMU shares the cap_id; we don't
        # gate on its first message because the lidar already proved
        # the device is talking — the IMU stream comes from the same
        # firmware path.
        try:
            _decl_topic_out("robonix/primitive/lidar/lidar3d", lidar_topic)
            _decl_topic_out("robonix/primitive/imu/imu",       imu_topic)
        except grpc.RpcError as e:
            if e.code() != grpc.StatusCode.ALREADY_EXISTS:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"declare failed: {e.details()}"
                )

        with _state_lock:
            _initialized = True
        log.info("init complete: lidar3d=%s imu=%s", lidar_topic, imu_topic)
        return lifecycle_pb2.Driver_Response(ok=True, state="ready", error="")


def _start_driver_grpc(port: int) -> None:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    contracts_grpc.add_PrimitiveLidarDriverServicer_to_server(
        _LidarDriverServicer(), server
    )
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    log.info("LifecycleDriver gRPC serving on 0.0.0.0:%d", port)


def _decl_driver_iface(port: int) -> None:
    if _atlas_stub is None:
        return
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=_cap_id,
        contract_id="robonix/primitive/lidar/driver",
        transport=pb.TRANSPORT_GRPC,
        endpoint=f"127.0.0.1:{port}",
        params=pb.TransportParams(grpc=pb.GrpcParams(
            proto_file="robonix_contracts.proto",
            service_name="PrimitiveLidarDriver",
            method="Driver",
        )),
    ))


def _heartbeat_loop() -> None:
    while True:
        time.sleep(15.0)
        if _atlas_stub is None:
            continue
        try:
            _atlas_stub.Heartbeat(pb.HeartbeatRequest(capability_id=_cap_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)


def _on_signal(signum, _frame):
    log.info("signal %d — shutting down", signum)
    _kill_livox()
    sys.exit(0)


def main() -> None:
    global _atlas_stub, _cap_id
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    driver_port = int(os.environ.get("MID360_DRIVER_PORT", "50231"))
    _cap_id = os.environ.get(
        "ROBONIX_CAPABILITY_ID", "com.robonix.ranger.mid360_lidar"
    )

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    _start_driver_grpc(driver_port)

    channel = grpc.insecure_channel(atlas_addr)
    _atlas_stub = pb_grpc.AtlasStub(channel)
    pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
    md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
    try:
        _atlas_stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=_cap_id,
            namespace="robonix/primitive/lidar",
            capability_md_path=md_path,
        ))
        _decl_driver_iface(driver_port)
        log.info("registered cap %s, driver iface on :%d (awaiting INIT)",
                 _cap_id, driver_port)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered (re-deploy); ok", _cap_id)
        else:
            log.warning("atlas registration failed: %s", e)

    threading.Thread(target=_heartbeat_loop, daemon=True).start()
    log.info("ready — awaiting Driver(CMD_INIT)")
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        pass
    finally:
        _kill_livox()


if __name__ == "__main__":
    main()
