# mid360_lidar_rbnx

Robonix package wrapping the **Livox MID-360** LiDAR (Ethernet, 360° dome, 40 m range, integrated 6-axis IMU). Owns the `primitive/lidar/*` namespace. Publishes the lidar PointCloud2 on the host DDS bus and atlas-registers it under generic contracts so that mapping, navigation, and scene services discover the topic name through atlas — no hardcoded `/scanner/cloud` paths on the consumer side.

The MID-360 also produces an IMU stream (`/livox/imu`, published as a side-effect of the same upstream launch). The `primitive/imu/*` contract surface for that IMU lives in a separate package, [`mid360_imu_rbnx`](https://github.com/enkerewpo/mid360_imu_rbnx). Robonix's invariant is "one primitive namespace = one package".

## Capability surface

The `mode` is the abstract communication pattern declared in the contract TOML (`rpc` / `topic_in` / `topic_out`). The `transport` column records how this package realises it on the wire. Both columns matter — the same mode can ride different middleware (e.g. an `rpc` mode can be a gRPC method or an MCP tool).

| Contract                                 | Mode      | Transport | Source / handler                            |
| ---------------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/primitive/lidar/driver`         | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle |
| `robonix/primitive/lidar/lidar3d`        | topic_out | ROS 2     | `/scanner/cloud` (PointCloud2)              |
| `robonix/primitive/lidar/lidar_snapshot` | rpc       | MCP       | one-shot capture (TODO)                     |

## Driver-init lifecycle

`start.sh` brings up the atlas bridge process — no ROS spawn at this point. The bridge opens a gRPC server on `MID360_DRIVER_PORT` (default 50231), `RegisterCapability`s + declares **only** `primitive/lidar/driver` on atlas, then blocks on heartbeat awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The handler resolves the host's IP for `Livox/host_net_info` (config override `host_ip:`, env `LIVOX_HOST_IP`, or auto-detect via `ip route get <lidar_ip>`), generates an MID360 config JSON with the right IPs, spawns `ros2 launch livox_ros_driver2 msg_MID360_launch.py` with the appropriate environment (`LIVOX_MID360_CONFIG`, `LIVOX_XFER_FORMAT`), waits for the first PointCloud2 on the configured topic, declares `primitive/lidar/lidar3d` on atlas, and returns ok. Atlas only ever advertises endpoints we've confirmed are publishing.

## Layout

```
mid360_lidar_rbnx/
├── package_manifest.yaml         robonix dev-packaging spec
├── mid360_driver/
│   └── atlas_bridge.py           driver gRPC + lazy Init + livox spawn
├── scripts/
│   ├── build.sh                  colcon build vendored src + rbnx codegen
│   └── start.sh                  source ROS, exec atlas_bridge
├── src/
│   ├── livox_ros_driver2/        VENDORED upstream + our patches
│   └── livox_ros_driver2.patch   diff vs upstream HEAD at vendoring time
└── .gitignore                    excludes rbnx-build/
```

## What we patched on top of upstream

`src/livox_ros_driver2.patch` documents the diff against [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2). The vendored copy already has them applied:

1. `config/MID360_config.json` — host_net_info IPs `192.168.1.5 → .50`, lidar IP `.12 → .161`. Atlas_bridge can override at runtime via the `host_ip` / `lidar_ip` config keys; the JSON file just provides the baseline.
2. `launch_ROS2/msg_MID360_launch.py` — `xfer_format` default `1 → 2` (PointCloud2 XYZIT instead of Livox CustomMsg). Now also reads `LIVOX_XFER_FORMAT` / `LIVOX_PUBLISH_FREQ` / `LIVOX_FRAME_ID` from the env so atlas_bridge can pass config through.
3. `src/lddc.cpp` — global publisher topic `livox/lidar → scanner/cloud`. `multi_topic=1` keeps publishing per-lidar topics under `livox/lidar_*` unchanged.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "lidar_topic": "/scanner/cloud",
  "lidar_ip": "192.168.1.161",
  "host_ip": "",
  "xfer_format": 2,
  "publish_freq": 10.0,
  "frame_id": "livox_frame",
  "sentinel_timeout_s": 30.0
}
```

`host_ip: ""` triggers auto-detect via `ip route get <lidar_ip>`. Override only when route resolution can't pick the right interface (e.g. the Jetson has multiple Ethernet ports and the route picks the wrong one).

## Build / run standalone

```bash
bash scripts/build.sh           # or:  rbnx build -p .
bash scripts/start.sh           # or:  rbnx boot  -p .
# the driver gRPC will sit awaiting Driver(CMD_INIT). For a smoke test
# without rbnx boot, drive it manually with grpcurl or a local Python script.
```

After Init the lidar should appear on:

```bash
ros2 topic hz /scanner/cloud   # ~10 Hz PointCloud2
ros2 topic hz /livox/imu       # ~200 Hz sensor_msgs/Imu (consumed by mid360_imu_rbnx)
```

## Network — host-side prereqs

The lidar's IP is config-driven; the host's IP is not. Whatever the lidar's configured IP is (we ship `192.168.1.161` as default; flash to a different value via Livox Viewer 2 if your network policy dictates), the host's NIC must live on the same /24 subnet (e.g. `192.168.1.50/24` for a `192.168.1.0/24` lidar) and have a route to the lidar IP. How you set that up is host-specific — `nmcli`, `systemd-networkd`, plain `ip addr`, whatever your distro standardises on. The package can't subsume that step because Ethernet IP config is inherently host-state.

## License

This package: MulanPSL-2.0. Vendored `livox_ros_driver2/`: see `src/livox_ros_driver2/LICENSE` (BSD).
