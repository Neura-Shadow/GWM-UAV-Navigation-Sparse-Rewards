# Phase 5 Runtime Validation

## Status

Phase 5-A adds read-only runtime capability detection for future real-runtime
validation. It does not launch Isaac Sim, start ROS2 nodes, connect to MAVSDK,
launch PX4 SITL, or touch real hardware.

## Goal

The detector answers a narrow question:

```text
What optional runtime tools appear available on this machine?
```

It reports Python, platform, CUDA/PyTorch CUDA visibility, optional
`nvidia-smi` GPU metadata, Isaac Sim / Omniverse import availability, ROS2
environment and import availability, MAVSDK import availability, PX4-related
command availability, GitHub CLI availability, selected environment variables,
and safe PATH hints.

## Usage

```bash
python scripts/check_runtime_capabilities.py --no-write-output
python scripts/check_runtime_capabilities.py --json --pretty --no-write-output
python scripts/check_runtime_capabilities.py --output outputs/runtime_validation/runtime_capability_report.json
```

The default report path is:

```text
outputs/runtime_validation/runtime_capability_report.json
```

Runtime reports are machine-specific artifacts and should not be committed.

## Safety Boundary

The detector uses safe probes only:

- `importlib.util.find_spec(...)`
- selected environment variable reads
- `shutil.which(...)`
- version/status subprocesses such as `nvidia-smi --query-gpu`, `gh --version`,
  and `gh auth status`

It does not instantiate `SimulationApp`, start ROS2 nodes, connect to MAVSDK,
launch PX4, or run GPU workloads.

Safe defaults remain:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

Phase 5-A does not enable real hardware flight, autonomous real flight, real
hardware tests, PX4 launch, Isaac Sim launch, ROS2 live-topic tests, or MAVSDK
connections.

## Environment Handling

Only an allowlist of environment variables is reported:

- `GWM_RUNTIME_ARTIFACT_DIR`
- `GWM_RUN_ISAAC_RUNTIME_TESTS`
- `GWM_RUN_ROS2_SENSOR_SYNC_TESTS`
- `GWM_RUN_MAVSDK_SITL_TESTS`
- `GWM_ALLOW_OPTIONAL_RUNTIME`
- `GWM_ALLOW_SITL_COMMANDS`
- `GWM_SITL_CONNECTION_URL`
- `GWM_ROS2_LIVE_TOPICS`
- `GWM_ALLOW_PX4_LAUNCH`
- `ROS_DISTRO`
- `CUDA_VISIBLE_DEVICES`

Keys containing `TOKEN`, `SECRET`, `PASSWORD`, `KEY`, `CREDENTIAL`, or `AUTH`
are redacted. The detector never dumps the full environment.

## Future Slices

- Phase 5-B: guarded Isaac Sim runtime smoke test.
- Phase 5-C: guarded ROS2 sensor synchronization smoke test.
- Phase 5-D: guarded MAVSDK / PX4 SITL command-path smoke test.
- Phase 5-E: closed-loop mock-to-SITL integration plan.

These future slices remain opt-in and are not part of Phase 5-A.

