# Deployment Hardware Interface

## Overview

Phase 3-D adds a mock-first deployment interface layer. It prepares the
framework for PX4, ArduPilot, MAVLink, `ros2_control`, Nav2, and CBF-style
safety integration without requiring any of those runtimes in CI.

This slice does not enable autonomous real flight.

## Deployment Levels

### Level 0: Mock Deployment

Level 0 is the default and required CI path.

- No hardware
- No ROS2
- No MAVSDK
- No PX4 or ArduPilot
- No Nav2
- Pure Python tests only

Default config:

```yaml
deployment:
  mock: true
  real_hardware_enabled: false
```

### Level 1: SITL / HIL-Ready Interface

Level 1 provides command conversion and connection configuration only.

- `MAVLinkBridge` can convert `ControlCommand` objects into MAVLink-like dicts.
- `HardwareInterface` exposes a `read()` / `write()` contract.
- `ROS2ControlHardwareInterface` is a guarded stub, not a real controller plugin.
- `WorldModelCostmapLayer` and `WorldModelPlannerPlugin` are pure-Python Nav2-style skeletons.

Phase 3-D does not launch PX4 SITL, ArduPilot SITL, MAVSDK connections, or HIL sessions.

### Level 2: Real Hardware Deployment

Level 2 is documented only in this slice.

Real hardware work requires:

- Explicit `real_hardware_enabled: true`
- Manual operator review
- Vehicle-specific safety checklist
- Flight controller validation
- Physical emergency-stop procedure
- Site-specific geofence and altitude limits

No autonomous real flight is enabled by default.

## Interfaces

### MAVLink Bridge

`MAVLinkBridge` provides async lifecycle methods and mock command history:

- `connect()`
- `disconnect()`
- `arm()`
- `takeoff(altitude)`
- `send_velocity(vx, vy, vz, yaw_rate)`
- `land()`
- `emergency_stop()`
- `command_to_mavlink(command)`

Real mode is guarded. Without MAVSDK or an injected client, real connection
attempts raise a clear `RuntimeError`.

### Hardware Interface

`HardwareInterface` is the deployment read/write contract:

- `read() -> HardwareState`
- `write(command: ControlCommand) -> bool`
- `emergency_stop() -> bool`

`MockHardwareInterface` implements this in memory and applies safety saturation
before accepting writes.

### Nav2-Style Skeletons

`WorldModelCostmapLayer` mutates a plain 2-D grid with risk costs.
`WorldModelPlannerPlugin` returns a deterministic straight-line path. These are
Python skeletons only, not real Nav2 plugins.

## Safety Layer

`ControlBarrierFunction` is a conservative baseline filter, not a certification
proof. It supports:

- Emergency stop command
- Safe hover command behavior
- Safe land command metadata
- Velocity saturation
- Yaw-rate saturation
- Altitude bounds
- Geofence placeholder checks
- Geometric obstacle barrier checks

The safety controller remains advisory/mock-first until real deployment work is
approved separately.

## Out Of Scope

- Real PX4 SITL launch automation
- Real ArduPilot SITL launch automation
- Real MAVSDK connection tests
- Real hardware flight
- Real Nav2 plugin build system
- Real `ros2_control` C++ plugin
- Formal CBF certification proof
- Isaac Sim runtime execution
- Phase 4 work

## Verification

All Phase 3-D tests must pass without PX4, ArduPilot, MAVSDK, ROS2, Nav2,
Isaac Sim, GPU, or real hardware:

```bash
python -m pytest tests/test_deployment.py -q
python -m pytest tests/ -q
python -m compileall -q src/control src/ros2_bridge tests/test_deployment.py
```
