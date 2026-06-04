# Deployment Hardware Interface

## Overview

Phase 3-D adds a mock-first deployment interface layer. Phase 4-E extends the
MAVLink path with a guarded MAVSDK / PX4 SITL command interface. The framework
still does not require PX4, ArduPilot, MAVSDK, ROS2, Nav2, Isaac Sim, GPU, SITL,
or real hardware in normal tests.

These slices do not enable autonomous real flight.

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
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

### Level 1: SITL / HIL-Ready Interface

Level 1 provides guarded SITL command plumbing only.

- `MAVLinkBridge` can convert `ControlCommand` objects into MAVLink-like dicts.
- `MAVLinkBridge` can connect to an injected fake MAVSDK client or optional
  MAVSDK `System` when SITL is explicitly enabled.
- PX4 SITL uses `udp://:14540` by default.
- Offboard mode requires an initial safe setpoint before start.
- `HardwareInterface` exposes a `read()` / `write()` contract.
- `ROS2ControlHardwareInterface` is a guarded stub, not a real controller plugin.
- `WorldModelCostmapLayer` and `WorldModelPlannerPlugin` are pure-Python Nav2-style skeletons.

Phase 4-E does not launch PX4 SITL automatically. The operator must start SITL
externally before enabling the optional command path.

Explicit SITL opt-in:

```yaml
deployment:
  mock: false
  sitl_enabled: true
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

### Phase 5-D Guarded SITL Smoke

Phase 5-D adds an optional command-path smoke runner for `MAVLinkBridge`.
Default runs do not connect to MAVSDK or PX4 SITL:

```bash
python scripts/run_mavsdk_px4_sitl_smoke.py --no-write-output
```

A real SITL command attempt requires all gates:

```text
GWM_RUN_MAVSDK_SITL_TESTS=1
GWM_ALLOW_OPTIONAL_RUNTIME=1
GWM_ALLOW_SITL_COMMANDS=1
```

PX4 SITL must already be running externally. The smoke runner never launches
PX4, never enables real hardware, and never enables autonomous real flight. It
sends only a safe zero-velocity command through the guarded offboard path and
records command-history metadata. This is SITL command-path plumbing
validation, not real flight validation or certified safety evidence.

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
- `wait_until_ready(timeout_sec)`
- `start_offboard(initial_command)`
- `stop_offboard()`
- `send_command(command)`
- `hold()`
- `return_to_launch()`
- `command_to_mavlink(command)`

SITL mode is guarded. Without MAVSDK or an injected client, optional SITL
connection attempts raise a clear `RuntimeError`. Real hardware flags are
rejected in Phase 4-E even if a fake client is injected.

Frame convention:

```text
frame: body_ned
vx: forward
vy: right
vz: down
yaw_rate: rad/s input, converted to deg/s for MAVSDK-style velocity payloads
```

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

- Automatic PX4 SITL launch automation
- Real ArduPilot SITL launch automation
- Required MAVSDK connection tests
- Real hardware flight
- Autonomous real flight
- Real Nav2 plugin build system
- Real `ros2_control` C++ plugin
- Formal CBF certification proof
- Isaac Sim runtime execution changes
- Phase 4-F end-to-end demo

## Verification

All Phase 3-D / 4-E tests must pass without PX4, ArduPilot, MAVSDK, ROS2, Nav2,
Isaac Sim, GPU, SITL, or real hardware:

```bash
python -m pytest tests/test_mavsdk_sitl.py -q
python -m pytest tests/test_deployment.py -q
python -m pytest tests/ -q
python -m compileall -q src/control src/ros2_bridge tests/test_mavsdk_sitl.py
```
