# Phase 6-E Isaac + PX4 SITL Bridge Design

## Status

Phase 6-E defines the bridge design between an Isaac Sim / Isaac Lab simulated
world and a PX4 SITL offboard-control loop. It is a design and dry-run
readiness layer only.

It does not:

- launch Isaac Sim
- start ROS2 nodes
- connect to MAVSDK or PX4 SITL
- launch PX4
- connect to real hardware
- enable autonomous real flight
- run the Phase 6-F closed-loop demo

## Command

```bash
python scripts/run_isaac_px4_bridge_design.py --no-write-output
python scripts/run_isaac_px4_bridge_design.py --require-prior-reports --json --pretty
```

The default report path is:

```text
outputs/runtime_validation/isaac_px4_bridge_design.json
```

Reports are machine-specific runtime artifacts and must not be committed.

## Bridge Strategy

The primary Phase 6-E strategy is a MAVSDK-only lightweight command and
telemetry path to an externally running PX4 SITL endpoint.

```text
Isaac Sim / Isaac Lab world
  -> direct Isaac sensors or ROS2 sensor sync
  -> ObservationBuffer
  -> GWM / WAM planner
  -> ControlBarrierFunction
  -> MAVLinkBridge
  -> MAVSDK
  -> PX4 SITL offboard control
```

ROS2 / Micro XRCE-DDS remains a documented future bridge option. It is not
implemented in Phase 6-E.

## State Ownership

- Isaac owns the simulated world, scene stepping, virtual sensors, and
  visual/physics context.
- PX4 SITL owns autopilot state, arming state, command acceptance, and offboard
  mode state.
- MAVSDK provides the SITL-only command and telemetry transport.
- The GWM / WAM planner owns future rollout, trajectory scoring, and action
  selection.
- `ControlBarrierFunction` remains mandatory before any future MAVSDK command
  write.

## Coordinate Frames

Phase 6-E does not silently convert coordinate frames.

```text
project_default -> current repository convention
isaac_z_up      -> Isaac world frame
px4_ned         -> PX4 world frame
px4_body_ned    -> MAVSDK body velocity command frame
```

The generated `FrameTransformPolicy` records:

- `coordinate_conversion_applied: false`
- `transforms_defined: false`
- `blocks_silent_conversion: true`
- `stale_transform_rejection: true`

An explicit transform policy must be implemented before Phase 6-F can claim a
physically coupled Isaac/PX4 loop.

## Timing Policy

Default dry-run design values:

- Isaac simulation step: `0.05s`
- safety / local command loop: `50 Hz`
- GWM / WAM planning loop: `2 Hz`
- MAVSDK command loop: `10 Hz`
- stale observation timeout: `0.25s`
- stale command timeout: `0.2s`

Stale sensor data, stale planner output, PX4 telemetry gaps, and MAVSDK
disconnects should first produce a zero/hold command and then emergency-stop
handling in the future live loop.

## Prior Reports

With `--require-prior-reports`, Phase 6-E checks only for existing report files:

- `outputs/runtime_validation/isaac_sensor_runtime.json`
- `outputs/runtime_validation/ros2_sim_sensor_bridge.json`
- `outputs/runtime_validation/px4_sitl_command_validation.json`

The command does not run Phase 6-B, 6-C, or 6-D. Missing or unready reports
produce `status=not_ready`.

## Refusal Rules

Phase 6-E refuses:

- `real_hardware_enabled: true`
- `autonomous_real_flight_enabled: true`
- `deployment.mock: true` for the live SITL bridge design
- `deployment.sitl_enabled: false`
- PX4 launch requests
- future coupled execution without an explicit frame-transform policy

PX4 SITL remains externally managed. Automatic PX4 launch is not part of this
slice.

## Verification

```bash
python -m pytest tests/test_phase6_isaac_px4_bridge_design.py -q
python -m pytest tests/ -q
python -m compileall -q src/runtime_validation scripts tests/test_phase6_isaac_px4_bridge_design.py
python scripts/run_isaac_px4_bridge_design.py --no-write-output
python scripts/run_isaac_px4_bridge_design.py --help
```

Normal verification requires no Isaac Sim, ROS2, MAVSDK, PX4, GPU, SITL, or
hardware.
