# ROS2 Integration Guide

## Overview

This framework uses an **adapter pattern** to decouple the navigation stack from ROS2.  All ROS2 interactions go through an abstract `ROS2AdapterBase` class, with two implementations:

1. **`MockROS2Adapter`** — A pure-Python mock that simulates pub/sub and service calls without requiring `rclpy` or a ROS2 installation.  This is the default and is used for all unit tests and development.
2. **`RealROS2Adapter`** (planned) — A thin wrapper around `rclpy` that connects to a real ROS2 graph for deployment.

---

## Current Adapter Pattern

### Abstract Interface

```python
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict

class ROS2AdapterBase(ABC):
    """Abstract interface for ROS2 communication."""

    @abstractmethod
    def publish(self, topic: str, message: Any) -> None: ...

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable) -> None: ...

    @abstractmethod
    def call_service(self, service: str, request: Any) -> Any: ...

    @abstractmethod
    def spin_once(self, timeout_sec: float = 0.1) -> None: ...
```

### Mock Implementation

The `MockROS2Adapter` stores published messages in an in-memory dictionary and dispatches them to registered callbacks synchronously.  This allows the full control stack to be tested without any ROS2 installation:

```python
adapter = MockROS2Adapter()
adapter.subscribe("/odom", lambda msg: print(msg))
adapter.publish("/odom", {"x": 1.0, "y": 2.0, "z": -5.0})
# Callback fires immediately with the published message
```

---

## Planned ROS2 Topics

| Topic | Message Type | Direction | QoS | Description |
|-------|-------------|-----------|-----|-------------|
| `/uav/odom` | `nav_msgs/Odometry` | Pub | Reliable, 10 Hz | Vehicle odometry |
| `/uav/imu` | `sensor_msgs/Imu` | Pub | Best-effort, 200 Hz | IMU data |
| `/uav/depth` | `sensor_msgs/Image` | Pub | Best-effort, 30 Hz | Depth camera |
| `/uav/lidar` | `sensor_msgs/PointCloud2` | Pub | Best-effort, 10 Hz | 3D LiDAR scan |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | Sub | Reliable, 50 Hz | Velocity command |
| `/uav/trajectory` | `nav_msgs/Path` | Pub | Reliable, 5 Hz | Planned trajectory |
| `/uav/takeover` | `std_msgs/Bool` | Pub | Reliable, event | Takeover notification |
| `/uav/uncertainty` | `std_msgs/Float32` | Pub | Reliable, 10 Hz | Current uncertainty |
| `/world_model/latent` | `std_msgs/Float32MultiArray` | Pub | Reliable, 10 Hz | Latent state vector |

---

## Planned ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/world_model/predict` | Custom | Request future state prediction for a given action sequence |
| `/planner/replan` | `std_srvs/Trigger` | Force the planner to recompute the trajectory |
| `/safety/set_threshold` | Custom | Dynamically update safety thresholds |

---

## Planned ROS2 Actions

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_goal` | `nav2_msgs/NavigateToPose` | Navigate to a goal pose with world-model guidance |
| `/execute_trajectory` | Custom | Execute a pre-planned trajectory with safety monitoring |

---

## DDS QoS Configuration

Quality-of-Service profiles are critical for deterministic real-time behaviour:

```yaml
qos_profiles:
  sensor_data:
    reliability: BEST_EFFORT
    durability: VOLATILE
    history_depth: 1
    deadline_ms: 50

  control_commands:
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history_depth: 5
    deadline_ms: 20

  state_broadcast:
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history_depth: 10
    deadline_ms: 100
```

- **Sensor data** uses `BEST_EFFORT` to avoid blocking on slow subscribers.
- **Control commands** use `RELIABLE` to guarantee delivery.
- **State broadcasts** (multi-agent) use `RELIABLE` with deeper history for late-joining agents.

---

## ros2_control Integration Plan

The framework will integrate with `ros2_control` for deterministic actuator control:

1. **Hardware Interface** — Implement a `SystemInterface` plugin that bridges the flight controller (PX4 / ArduPilot) via MAVLink or serial.
2. **Controller Manager** — Load a position/velocity controller that accepts setpoints from the world-model planner.
3. **Safety Controller** — Implement as a `ros2_control` controller that runs at the highest priority and can override any other controller.

```
┌─────────────────────────────────────────────────┐
│                Controller Manager                │
│  ┌───────────┐  ┌──────────────┐  ┌───────────┐ │
│  │  Planner  │  │   Safety     │  │  Telemetry│ │
│  │ Controller│  │ Controller   │  │ Controller│ │
│  │  (5 Hz)   │  │ (200 Hz)     │  │ (10 Hz)   │ │
│  └─────┬─────┘  └──────┬───────┘  └─────┬─────┘ │
│        │               │                │        │
│        └───────┬───────┘                │        │
│                ▼                        │        │
│        Hardware Interface               │        │
│        (PX4 / ArduPilot)               │        │
└─────────────────────────────────────────┘        │
                                                    │
                                              Logging
```

---

## Nav2 Integration Plan

For ground vehicles (UGV / AMR), the framework will integrate with Nav2:

1. **Costmap Layer** — Custom costmap plugin that incorporates world-model uncertainty as a cost.
2. **Planner Plugin** — Replace or augment the default planner with the world-model-guided planner.
3. **Behaviour Tree** — Custom BT nodes for uncertainty checking and takeover.

---

## Migration: Mock → Real ROS2

Switching from mock to real ROS2 requires:

1. **Install ROS2 Humble** (or later) and build the workspace.
2. **Change config**:
   ```yaml
   ros2:
     type: "rclpy"  # was "mock"
   ```
3. **Implement `RealROS2Adapter`** — a thin wrapper that:
   - Creates a `rclpy.node.Node`.
   - Maps topic names and message types.
   - Handles serialisation/deserialisation.
4. **Launch file** — Provide a ROS2 launch file that starts the node graph:
   ```
   ros2 launch gwm_nav bringup.launch.py config:=configs/ros2.yaml
   ```

The migration is designed to be incremental: individual topics can be switched from mock to real one at a time for debugging.
