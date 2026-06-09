# Placeholder Audit

## Purpose

This Phase 8-C audit records how placeholder-like scripts, comments, and
deferred code paths should be interpreted after the `v0.7.1` research-framework
closure. The repository is complete for its current safe research-framework
scope; the items below are intentionally scoped, guarded, or deferred rather
than accidentally unfinished.

This audit does not add runtime features, launch simulators, start ROS2 nodes,
connect to MAVSDK / PX4 SITL, enable real hardware, or enable autonomous real
flight.

## Audit Categories

| Category | Meaning |
| --- | --- |
| Scoped mock/planning path | Kept as a lightweight CLI, report, or test path that runs without optional runtimes. |
| Deferred research extension | Useful future work, but not required for the completed `v0.7.1` framework. |
| Guarded safety placeholder | Explicit pure-Python safety or metadata hook retained to make boundaries visible. |
| Compatibility sentinel | Deterministic return value retained for stable tests and downstream behavior. |

## Reviewed Areas

| Area | Classification | Current interpretation |
| --- | --- | --- |
| `scripts/evaluate_policy.py` | Scoped mock/planning path | Legacy evaluator remains mock-only. AirSim-family evaluation is intentionally routed to `run_multisim_gwm_demo.py` and guarded AirSim-family validation scripts. |
| `scripts/run_digital_twin_generation.py` | Scoped mock/planning path | CLI parses config/scenario inputs and prints a generation plan. Runtime scene export is a planned research extension, not a project blocker. |
| `scripts/run_real2sim2real_loop.py` | Scoped mock/planning path | Mock Real2Sim2Real pipeline records deterministic per-variant coverage metadata. Live simulator training on variants is a planned research extension. |
| `src/world_model/uncertainty.py` | Compatibility sentinel / deferred research extension | `EnsembleUncertainty` preserves deterministic historical behavior while variance-based ensemble estimation remains a planned research extension. |
| `src/ros2_bridge/mavlink_bridge.py` hold/RTL paths | Guarded safety placeholder | Hold and return-to-launch calls record fallback history metadata when a client action is unavailable. Existing `placeholder` metadata keys are retained for compatibility. |
| Geofence checks and safety-boundary markers | Guarded safety placeholder | Geofence placeholder checks are explicit safety hooks, not certified safety proof or real hardware support. |
| `ROS2ControlHardwareInterface` wording | Guarded safety placeholder | The interface is a guarded Python stub, not a real `ros2_control` controller plugin. Real controller plugins remain out of scope for safety reasons. |

## Remaining Placeholder-Like Terms

After the cleanup pass, remaining `placeholder` or `stub` terms are intentional:

- geofence placeholder checks in safety documentation and tests;
- MAVLink command-history metadata fields named `placeholder`;
- guarded `ROS2ControlHardwareInterface` stub wording;
- historical release notes that preserve checkpoint wording.

These remaining terms document explicit safety boundaries or compatibility
metadata. They should not be read as unfinished Phase 8 work.

## Safety Boundaries

Phase 8-C does not change the runtime stance:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

The repository still does not claim real flight validation, production
readiness, formal safety certification, real Nav2 plugins, real `ros2_control`
plugins, SITL/HIL launch automation, or physical UAV deployment support.

## Verification

The Phase 8-C focused verification used normal mock/fake checks only:

```bash
python -m pytest tests/test_world_model.py tests/test_mavsdk_sitl.py -q
python -m compileall -q scripts/evaluate_policy.py scripts/run_digital_twin_generation.py scripts/run_real2sim2real_loop.py src/world_model/uncertainty.py src/ros2_bridge/mavlink_bridge.py tests/test_world_model.py tests/test_mavsdk_sitl.py
git diff --check
```

No optional live Cosys-AirSim / AirSim validation, Isaac Sim launch, ROS2 node
startup, MAVSDK / PX4 SITL connection, PX4 launch, Nav2 work, hardware check,
or autonomous real-flight enablement was run.
