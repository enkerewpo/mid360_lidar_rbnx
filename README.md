# mid360_lidar_rbnx

Robonix package wrapping the **Livox MID-360** LiDAR (Ethernet, 360° dome,
40 m range, integrated 6-axis IMU). Publishes lidar+IMU streams on the
host DDS bus and atlas-registers them under generic contracts so that
mapping, navigation, and scene services discover the topic names through
atlas — no hardcoded `/scanner/cloud` paths on the consumer side.

## Capability surface

The `mode` is the abstract communication pattern declared in the
contract TOML (rpc / topic_in / topic_out). The `transport` column
records how THIS package realises it on the wire — both matter because
the same mode can ride different middleware (an `rpc` mode can be a
gRPC method or an MCP tool call).

| Contract                                 | Mode      | Transport | Source / handler                            |
| ---------------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/primitive/lidar/driver`         | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle |
| `robonix/primitive/lidar/lidar3d`        | topic_out | ROS 2     | `/scanner/cloud` (PointCloud2)              |
| `robonix/primitive/lidar/lidar_snapshot` | rpc       | MCP       | one-shot capture (TODO)                     |
| `robonix/primitive/imu/driver`           | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle |
| `robonix/primitive/imu/imu`              | topic_out | ROS 2     | `/livox/imu` (sensor_msgs/Imu)              |

Both `lidar/driver` and `imu/driver` are exposed as separate gates.
They share one underlying init path (the MID-360 is one device — a
single Init brings up both streams), but the abstraction stays clean:
an IMU-only consumer can gate on `imu/driver` without knowing the IMU
happens to live inside a lidar. Whichever gate `rbnx boot` calls
first triggers the shared Init; the other returns `ready` immediately.

## Layout

```
mid360_lidar_rbnx/
├── package_manifest.yaml         robonix dev-packaging spec
├── mid360_driver/                Python package (atlas registration)
│   └── atlas_bridge.py
├── scripts/
│   ├── build.sh                  colcon build vendored src + rbnx codegen
│   ├── start.sh                  spawn livox driver + atlas_bridge
│   └── prepare_livox_config.sh   resolve host IP, generate config JSON
├── src/
│   ├── livox_ros_driver2/        VENDORED upstream + our fixes
│   └── livox_ros_driver2.patch   diff vs upstream HEAD at vendoring time
└── .gitignore                    excludes rbnx-build/
```

## What we patched on top of upstream

`src/livox_ros_driver2.patch` documents the diff against
[Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).
The vendored copy already has them applied:

1. `config/MID360_config.json` — host_net_info IPs `192.168.1.5 → .50`,
   lidar IP `.12 → .161` (matches the Ranger Mini's static config).
   Override at runtime with `LIVOX_HOST_IP` / `LIVOX_LIDAR_IP`.
2. `launch_ROS2/msg_MID360_launch.py` — `xfer_format` default `1 → 2`
   (PointCloud2 XYZIT instead of Livox CustomMsg). Now also reads
   `LIVOX_XFER_FORMAT` / `LIVOX_PUBLISH_FREQ` / `LIVOX_FRAME_ID` env.
3. `src/lddc.cpp` — global publisher topic `livox/lidar → scanner/cloud`.
   `multi_topic=1` keeps publishing per-lidar topics under `livox/lidar_*`
   unchanged.

## Config (passed via `RBNX_CONFIG_FILE`)

```json
{
  "lidar_topic": "/scanner/cloud",
  "imu_topic": "/livox/imu",
  "sentinel_timeout_s": 30.0,
  "capability_id": "com.robonix.ranger.mid360_lidar"
}
```

The ranger_mini_deploy manifest passes these values; standalone
`rbnx boot -p .` uses the defaults.

## Build / run standalone

```bash
# build
bash scripts/build.sh
# or  rbnx build -p .

# run (host network must be configured for 192.168.1.50/24, or override LIVOX_HOST_IP)
bash scripts/start.sh
# or  rbnx boot -p .
```

After boot the lidar should appear on:

```bash
ros2 topic hz /scanner/cloud   # ~10 Hz PointCloud2
ros2 topic hz /livox/imu       # ~200 Hz sensor_msgs/Imu
```

## Network

The MID-360 ships configured for `192.168.1.12`; we re-flashed it to
`192.168.1.161` (see Livox Viewer 2). The Jetson NIC needs to live on
`192.168.1.50/24` (use the helper `wheatfox/scripts/nm-mid360-static-ip.sh`
on the robot, which sets a NetworkManager profile that survives reboots).

## License

This package: MulanPSL-2.0.
Vendored `livox_ros_driver2/`: see `src/livox_ros_driver2/LICENSE` (BSD).
