# ROS2 Integration Guide

## Overview

The framework uses an adapter pattern to keep the navigation stack testable
without ROS2. Phase 3-A splits ROS2 integration into two layers:

1. `ROS2Adapter` is the narrow control-facing contract. It sends
   `ControlCommand` objects and returns the latest `SensorObservation`
   odometry. `MockROS2Adapter` remains the default for tests and development.
2. `ROS2Bridge` is the real ROS2 lifecycle wrapper. It owns the `rclpy` node,
   publishers, subscriptions, services, `spin_once()`, and shutdown behavior.

All ROS2 imports are guarded. Importing `src.ros2_bridge` works in a normal
Python environment; constructing real ROS2 objects requires a ROS2 install.

## Current Interfaces

### Control Adapter

```python
class ROS2Adapter(ABC):
    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...

    @property
    def is_connected(self) -> bool: ...

    @abstractmethod
    def send_command(self, command: ControlCommand) -> bool: ...

    @abstractmethod
    def get_odometry(self) -> SensorObservation | None: ...
```

`MockROS2Adapter` implements this interface in memory. `RealROS2Adapter` uses
`ROS2Bridge` internally and defaults to:

| Purpose | Topic | ROS2 Message |
|---------|-------|--------------|
| Velocity command | `/cmd_vel` | `geometry_msgs/Twist` |
| Odometry feedback | `/odom` | `nav_msgs/Odometry` |

### ROS2 Bridge

```python
bridge = ROS2Bridge(node_name="gwm_uav_bridge", config=config)
publisher = bridge.create_publisher("/cmd_vel", Twist, qos)
subscription = bridge.create_subscription("/odom", Odometry, callback, qos)
service = bridge.create_service("/planner/replan", Trigger, callback)
bridge.spin_once(timeout_sec=0.1)
bridge.shutdown()
```

`ROS2Bridge` raises a clear `RuntimeError` if `rclpy` is unavailable.

## Pure-Python Conversion Layer

Message conversion is intentionally testable without ROS2 message packages:

- `control_command_to_twist_dict(cmd)`
- `twist_dict_to_control_command(twist)`
- `odometry_dict_to_sensor_observation(odom)`
- `sensor_observation_to_odom_dict(obs)`
- `image_to_rgb_array(image)`
- `image_to_depth_array(depth)`
- `lidar_to_array(scan_or_cloud)`

The real adapter converts through these dictionaries and then wraps or unwraps
actual ROS2 message objects when ROS2 is present.

## Phase 4-D Sensor Synchronization

Phase 4-D adds `ROS2SensorSynchronizer`, a mock-first synchronization layer for
Generated World Model observations. It aligns RGB, depth, LiDAR, optional IMU,
and odometry messages into one `SensorObservation`.

```python
from src.ros2_bridge import ROS2SensorSynchronizer

synchronizer = ROS2SensorSynchronizer()
synchronizer.ingest("rgb", rgb_msg)
synchronizer.ingest("depth", depth_msg)
synchronizer.ingest("lidar", lidar_msg)
synchronizer.ingest("odom", odom_msg)
observation = synchronizer.latest_observation()
```

Normal tests use manual ingest with dictionaries or object-like payloads. Real
ROS2 mode is optional and guarded behind `message_filters`, `sensor_msgs`, and
`nav_msgs`. The implementation follows ROS2's approximate timestamp
synchronization pattern, but importing the package still works without ROS2.

Default synchronized topics:

| Purpose | Topic | ROS2 Message |
|---------|-------|--------------|
| RGB image | `/camera/rgb` | `sensor_msgs/Image` |
| Depth image | `/camera/depth` | `sensor_msgs/Image` |
| LiDAR | `/scan` | `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2` |
| Odometry | `/odom` | `nav_msgs/Odometry` |
| IMU metadata | `/imu` | `sensor_msgs/Imu` |

Header timestamps are required by default. Setting `allow_headerless: true`
uses receipt time and records that choice in observation metadata.

## Phase 5-C Guarded Runtime Smoke

Phase 5-C adds an optional smoke runner for `ROS2SensorSynchronizer`. Normal
tests and default CLI runs remain ROS2-free:

```bash
python scripts/run_ros2_sensor_sync_smoke.py --no-write-output
```

The command reports `status=skipped` unless both runtime gates are set:

```text
GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1
GWM_ALLOW_OPTIONAL_RUNTIME=1
```

With both gates set and ROS2 sensor modules importable, the runner can start
guarded real-mode synchronization and wait briefly for a synchronized packet.
It does not create ROS2 publishers, launch ROS2, inspect live topics by default,
or touch MAVSDK, PX4, Isaac Sim, Nav2, or hardware. Mock tests exercise the same
conversion path using synthetic RGB, depth, LiDAR, odometry, and IMU payloads.

## QoS Configuration

Phase 3-A uses `configs/ros2_control.yaml`:

```yaml
ros2:
  node_name: "gwm_uav_bridge"
  topics:
    cmd_vel: "/cmd_vel"
    odom: "/odom"
  qos_profiles:
    control_commands:
      reliability: "reliable"
      history_depth: 10
      deadline_ms: 20.0
      lifespan_sec: 1.0
    odometry:
      reliability: "best_effort"
      history_depth: 5
      deadline_ms: 100.0
      lifespan_sec: 1.0
  sensor_sync:
    enabled: false
    real_mode: false
    required_streams: ["rgb", "depth", "lidar", "odom"]
    slop_sec: 0.05
    queue_size: 10
    allow_headerless: false
```

`QoSConfig` normalizes reliability values case-insensitively and currently
supports `reliable` and `best_effort`.

## Migration: Mock to Real ROS2

1. Install ROS2 Humble or another compatible ROS2 distribution.
2. Source the ROS2 environment so `rclpy`, `geometry_msgs`, and `nav_msgs` are
   importable by Python.
3. Load `configs/ros2_control.yaml`.
4. Construct `RealROS2Adapter(config=config)` and use it anywhere a
   `ROS2Adapter` is accepted.

## Phase 3-D Deployment Interfaces

Phase 3-D adds mock-first deployment interfaces next to the existing ROS2
adapter layer:

- `MAVLinkBridge` converts `ControlCommand` objects into MAVLink-like command
  dictionaries and records async mock command history.
- `HardwareInterface` and `MockHardwareInterface` expose a `read()` / `write()`
  contract inspired by `ros2_control`.
- `ROS2ControlHardwareInterface` is a guarded Python stub, not a real
  `ros2_control` C++ plugin.
- `WorldModelCostmapLayer` and `WorldModelPlannerPlugin` are pure-Python
  Nav2-style skeletons, not runtime Nav2 plugins.
- `ControlBarrierFunction` is a baseline runtime filter, not a certification
  proof.

The deployment config defaults to:

```yaml
deployment:
  mock: true
  real_hardware_enabled: false
```

PX4, ArduPilot, MAVSDK, Nav2 runtime integration, SITL/HIL automation, and real
hardware flight remain future work.
