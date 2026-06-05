# Phase 6-F GWM / WAM Closed-Loop Simulation Demo

## Status

Phase 6-F adds the guarded pure-simulation GWM / WAM closed-loop demo. It is
the final Phase 6 slice and is simulation / SITL only.

It does not:

- launch PX4
- connect to real hardware
- enable autonomous real flight
- start Nav2
- claim production readiness
- claim certified safety

Normal tests use injected fake Isaac, ROS2, and MAVSDK / PX4 SITL objects. Live
runtime attempts require explicit operator gates and must report skipped,
`runtime_unavailable`, or `not_ready` when prerequisites are missing.

## Command

```bash
python scripts/run_phase6_gwm_simulation_demo.py --no-write-output
python scripts/run_phase6_gwm_simulation_demo.py --runtime-mode fake --steps 3 --no-require-prior-reports --no-write-output
python scripts/run_phase6_gwm_simulation_demo.py --require-prior-reports --json --pretty
```

`--runtime-mode fake` is a local verification mode that constructs fake Isaac,
ROS2, and MAVSDK/PX4 SITL facades inside the runner. It exercises the full
ObservationBuffer -> GWM/WAM planner -> scorer -> CBF -> command-history loop
without launching Isaac Sim, starting ROS2 nodes, connecting to MAVSDK/PX4 SITL,
launching PX4, or touching hardware. The default `guarded` mode still requires
the explicit live-runtime gates before real simulation/SITL paths are attempted.

The default report path is:

```text
outputs/runtime_validation/phase6_gwm_simulation_demo.json
```

Reports are local runtime artifacts and must not be committed.

## Runtime Flow

```text
Isaac Sim / Isaac Lab simulated sensors
  -> direct Isaac observations or ROS2 sensor synchronization
  -> ObservationBuffer
  -> Generated World Model / WAM rollout
  -> candidate trajectory scoring
  -> ControlBarrierFunction safety gate
  -> MAVLinkBridge / MAVSDK
  -> externally running PX4 SITL
  -> runtime metrics and safety report
```

The v1 state update policy is intentionally explicit:

```text
state_coupling: command_mirror
px4_telemetry_used_for_isaac_state: false
```

This means the safe command sent toward PX4 SITL is also used to advance the
Isaac-side simulation step. Phase 6-F does not claim full physics-coupled
PX4-telemetry-to-Isaac state synchronization.

## Gates

Live runtime mode requires:

```text
GWM_ALLOW_OPTIONAL_RUNTIME=1
GWM_RUN_ISAAC_RUNTIME_TESTS=1
GWM_RUN_MAVSDK_SITL_TESTS=1
GWM_ALLOW_SITL_COMMANDS=1
```

When `--observation-path ros2` is selected, it also requires:

```text
GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1
```

`GWM_ALLOW_PX4_LAUNCH` is not used by Phase 6-F. PX4 SITL must be started
externally by the operator.

## Required Safe Deployment

```yaml
deployment:
  mock: false
  sitl_enabled: true
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

Any real hardware or autonomous real-flight flag is rejected before runtime
construction.

## Prior Reports

With `--require-prior-reports`, the demo checks existing reports only:

```text
outputs/runtime_validation/isaac_sensor_runtime.json
outputs/runtime_validation/ros2_sim_sensor_bridge.json
outputs/runtime_validation/px4_sitl_command_validation.json
outputs/runtime_validation/isaac_px4_bridge_design.json
```

It does not run Phase 6-B, 6-C, 6-D, or 6-E automatically.

## Safety Behavior

Before every MAVSDK write, the demo applies `ControlBarrierFunction` for:

- velocity and yaw-rate saturation
- altitude bounds
- geofence placeholder checks
- obstacle-distance barrier checks

Runtime failures attempt emergency-stop handling when a bridge is connected and
always run cleanup in `finally`.

## Report Schema

Result JSON uses:

```text
gwm_phase6_simulation_demo_v1
```

The report records runtime gates, prior-report readiness, runtime invocation
summary, loop summary, bridge summary, coordinate-frame summary, per-step
planner/safety/command records, metrics, cleanup status, and errors.

## Verification

```bash
python -m pytest tests/test_phase6_gwm_simulation_demo.py -q
python -m pytest tests/ -q
python -m compileall -q src/generated_world_model src/runtime_validation scripts tests/test_phase6_gwm_simulation_demo.py
python scripts/run_phase6_gwm_simulation_demo.py --runtime-mode fake --steps 3 --no-require-prior-reports --no-write-output
python scripts/run_phase6_gwm_simulation_demo.py --no-write-output
python scripts/run_phase6_gwm_simulation_demo.py --help
```

Normal verification requires no Isaac Sim, ROS2, MAVSDK, PX4, GPU, SITL, Nav2,
or hardware.
