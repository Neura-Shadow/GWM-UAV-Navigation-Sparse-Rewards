# Phase 7 Multi-Simulator / Optional AirSim Backend

Phase 7 adds AirSim / CosysAirSim as an optional simulator backend beside the
Phase 6 Isaac Sim / ROS2 / PX4 SITL / MAVSDK mainline.

This phase does not replace Phase 6. Isaac remains the pure-simulation
full-stack baseline for PX4 SITL integration, while AirSim is a pluggable
navigation environment for simulator comparison and optional runtime smoke
checks.

## Safety Boundaries

Phase 7 does not:

- launch AirSim or Unreal automatically
- connect to physical UAV hardware
- enable autonomous real flight
- enable PX4 hardware or flight-controller checks
- weaken the Phase 6 CBF / safety-gate requirements
- route Phase 6-F live Isaac/PX4 mode through AirSim by default

Default deployment stays locked down:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

## Backend Registry

The simulator backend registry exposes:

```python
from src.simulator_backends import (
    SimulatorBackendConfig,
    SimulatorBackendRegistry,
    create_navigation_env,
)
```

Registered backends:

- `mock`: default, no optional runtime dependency
- `isaac`: guarded Isaac environment path
- `airsim`: guarded AirSim / CosysAirSim environment path

Backends are lazy-loaded, so importing the registry does not import Isaac Sim,
AirSim, ROS2, MAVSDK, PX4, or GPU runtimes.

## AirSim Runtime

`AirSimRuntime` lives in `src.digital_twin` and is import-safe without AirSim
installed:

```python
from src.digital_twin import AirSimRuntime
```

It prefers `cosysairsim` when available, then falls back to `airsim`. It supports
fake-client injection for tests and live connection only when explicitly gated.

AirSim frame metadata is preserved:

```text
source_frame: airsim_ned
target_frame: project_default
coordinate_conversion_applied: false
```

Phase 7 does not silently convert AirSim NED to Isaac Z-up.

## Runtime Gates

AirSim live smoke requires:

```text
GWM_ALLOW_OPTIONAL_RUNTIME=1
GWM_RUN_AIRSIM_RUNTIME_TESTS=1
GWM_ALLOW_AIRSIM_API_CONTROL=1
```

Without those gates:

```bash
python scripts/run_airsim_runtime_smoke.py --no-write-output
```

returns a safe skipped status and does not connect to AirSim.

## Multi-Simulator Demo Wrapper

The wrapper command is:

```bash
python scripts/run_multisim_gwm_demo.py --simulator-backend mock --steps 3 --no-write-output
```

`mock` remains the default. The `isaac` option delegates to the existing Phase
6-F guarded runner. The `airsim` option is simulation-only and requires the
AirSim runtime gates before live API-control commands.

## Backend Comparison

The comparison command is read-only:

```bash
python scripts/run_simulator_backend_comparison.py --no-write-output
```

It compares registration, observation schema compatibility, frame metadata, and
runtime availability flags for `mock`, `isaac`, and `airsim`. It does not launch
simulators, connect to AirSim, start Isaac, or claim performance parity.

## Normal Verification

Normal tests remain optional-runtime-free:

```bash
python -m pytest tests/test_simulator_backends.py -q
python -m pytest tests/test_airsim_runtime.py -q
python -m pytest tests/ -q
python -m compileall -q src tests scripts
python scripts/run_airsim_runtime_smoke.py --no-write-output
python scripts/run_multisim_gwm_demo.py --simulator-backend mock --steps 3 --no-write-output
python scripts/run_simulator_backend_comparison.py --no-write-output
```

These checks do not launch AirSim, Unreal, Isaac Sim, ROS2, MAVSDK, PX4 SITL,
Nav2, or hardware.
