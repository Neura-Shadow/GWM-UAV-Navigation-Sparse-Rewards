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

The real adapter converts through these dictionaries and then wraps or unwraps
actual ROS2 message objects when ROS2 is present.

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

The rest of Phase 3 remains out of scope for this slice: Isaac Sim, PX4,
MAVSDK, distributed DDS, Nav2, and barrier certificates are planned separately.
