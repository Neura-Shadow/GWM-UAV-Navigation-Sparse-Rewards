# Final Verification Bundle

## Scope

This Phase 8-D bundle records the final safe verification pass for the
completed `v0.7.1` research-framework state plus the Phase 8-A/B/C closure
cleanup commits.

It is verification and documentation only. It does not implement runtime
features, launch simulators, start ROS2 nodes, connect to MAVSDK / PX4 SITL,
run real hardware checks, move tags, or create a GitHub Release.

## Baseline

- Latest released checkpoint: `v0.7.1-cosys-airsim-live-validation`
- Verification baseline commit: `379a078 Audit placeholder extension points`
- Verification date: 2026-06-09
- Current project state: completed safe research framework through `v0.7.1`

Phase 6 remains the guarded pure-simulation baseline:

```text
Isaac Sim / Isaac Lab + ROS2 + PX4 SITL + MAVSDK + GWM/WAM + CBF
```

Phase 7 remains the optional multi-simulator extension:

```text
Cosys-AirSim / cosysairsim preferred
legacy AirSim / airsim fallback
backend registry key: airsim
```

## Safety Defaults

The repository default deployment stance remains:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

The project does not claim real flight validation, production readiness,
certified safety, real Nav2 plugins, real `ros2_control` plugins, SITL/HIL
launch automation, or physical UAV deployment support.

## Verification Commands

The Phase 8-D pass used normal regression, compile, and mock/no-write-output
checks only.

| Command | Result |
| --- | --- |
| `python -m pytest tests/ -q` | `402 passed, 12 skipped in 135.62s` |
| `python -m compileall -q src scripts tests` | Passed |
| `python scripts/run_gwm_navigation_demo.py --backend mock --steps 5 --no-write-output` | `gwm_demo status=timeout steps=5 commands=5 safety_overrides=0` |
| `python scripts/run_phase6_gwm_simulation_demo.py --runtime-mode fake --steps 3 --no-require-prior-reports --no-write-output` | `phase6_gwm_simulation_demo status=passed steps=3 commands=3 safety_overrides=0` |
| `python scripts/run_multisim_gwm_demo.py --simulator-backend mock --steps 3 --no-write-output` | `multisim_gwm_demo backend=mock status=timeout steps=3 commands=3 safety_overrides=0` |
| `python scripts/run_simulator_backend_comparison.py --no-write-output` | `simulator_backend_comparison status=passed backends=airsim,isaac,mock` |

The `gwm_demo` and `multisim_gwm_demo` `status=timeout` results are bounded
mock/default smoke outcomes: each command ran the requested fixed step count,
sent mock commands, and stopped at the configured horizon. They are not failed
runtime tests and are not external-runtime validation.

The first full-test command attempt exceeded the local command timeout before
returning output; it was rerun with a longer timeout and passed with the result
shown above.

## Explicit Non-Runs

The Phase 8-D verification did not run:

- optional live Cosys-AirSim or legacy AirSim validation against a real running simulator;
- Cosys-AirSim, legacy AirSim, Unreal, or Isaac Sim launch;
- ROS2 node startup;
- MAVSDK / PX4 SITL connection;
- PX4 launch;
- Nav2 or `ros2_control` runtime implementation;
- SITL/HIL launch automation;
- hardware checks;
- autonomous real flight;
- tag creation, movement, recreation, or deletion;
- GitHub Release creation.

## Artifact Policy

All demo commands used `--no-write-output`; no runtime report, simulator log,
SITL artifact, rosbag, screenshot, dataset, checkpoint, result file, credential,
or token is part of this verification bundle.

Local `.codegraph/` data remains untracked and outside the repository artifact.

## Result

Phase 8-D confirms the repository is internally consistent and regression-clean
for the completed safe research-framework scope. Remaining work should be
treated as planned research extensions, deferred scope, or safety out-of-scope
items as classified in `docs/roadmap.md`.
