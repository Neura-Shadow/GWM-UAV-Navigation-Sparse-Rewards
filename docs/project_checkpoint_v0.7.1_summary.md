# v0.7.1 Project Checkpoint Summary

## Checkpoint

- Latest checkpoint: `v0.7.1-cosys-airsim-live-validation`
- Target commit: `1d45a4b Add optional Cosys-AirSim live validation`
- Related checkpoints:
  - `v0.6.0-pure-simulation-runtime`
  - `v0.7.0-optional-airsim-backend`
  - `v0.7.1-cosys-airsim-live-validation`

## Summary

The project is a mock-first and runtime-guarded research framework for
world-model-guided UAV navigation under sparse rewards. By `v0.7.1`, the
framework includes generated-world-model planning, guarded pure-simulation
runtime integration, and an optional AirSim-family simulator backend.

Phase 6 remains the primary guarded full-stack pure-simulation baseline:
Isaac Sim / Isaac Lab, ROS2 simulation sensor paths, externally managed PX4
SITL, MAVSDK, GWM/WAM planning, and CBF safety gates.

Phase 7 adds the optional multi-simulator layer. Cosys-AirSim / `cosysairsim`
is the preferred AirSim-family runtime, legacy AirSim / `airsim` remains the
fallback, and the stable backend registry key remains `airsim`.

Phase 7-B adds an optional live validation runner for externally started
Cosys-AirSim or legacy AirSim sessions. No-gate runs skip safely, and
zero-command validation requires `GWM_ALLOW_AIRSIM_API_CONTROL=1`.

## Verification

The Phase 7-B stabilization run completed with:

```text
402 passed, 12 skipped
```

No-gate/runtime-safe checks:

```text
airsim_live_validation status=skipped frames=0/0 commands=0 closed=false
airsim_runtime_smoke status=skipped frames=0/3 closed=false
simulator_backend_comparison status=passed backends=airsim,isaac,mock
```

## Safety Defaults

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

## Safety Boundaries

- Normal tests require no Cosys-AirSim, legacy AirSim, Unreal, Isaac Sim, ROS2,
  MAVSDK, PX4, Nav2, GPU, SITL, or real hardware.
- Cosys-AirSim and legacy AirSim live paths are optional guarded opt-ins.
- The repository does not launch Cosys-AirSim, legacy AirSim, Unreal, PX4, or
  hardware automatically.
- Real hardware and autonomous real flight remain disabled by default.
- The checkpoint does not claim production readiness, real flight validation,
  or certified safety.

## Roadmap Closure

The project should be read as complete through `v0.7.1` for the current
research-framework scope. `docs/roadmap.md` classifies remaining legacy ideas
as completed in mock-first / guarded-runtime form, planned research
extensions, deferred work beyond the current scope, or explicitly safety
out-of-scope items. Missing real hardware validation, real Nav2 plugins, real
`ros2_control` plugins, automatic SITL/HIL launch automation, and formal safety
certification are not unresolved requirements for this checkpoint.

## Research Position

This checkpoint should be read as a research framework milestone rather than a
production flight stack. The emphasis is on safe defaults, explicit runtime
gates, reproducible tests, and clear simulator boundaries.
