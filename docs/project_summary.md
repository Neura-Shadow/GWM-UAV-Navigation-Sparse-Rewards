# World-Model-Guided Digital-Twin UAV Navigation Research Framework

## Abstract

This project is a mock-first and runtime-guarded research framework for
sparse-reward UAV, UGV, and AMR navigation. It combines latent world models,
generated future-observation rollouts, asymmetric control, Real2Sim2Real
scenario generation, OpenUSD-style digital-twin descriptors, ROS2-style sensor
and command abstractions, distributed multi-agent coordination,
deployment-facing safety interfaces, and optional simulator backends.

The latest released checkpoint, `v0.7.1-cosys-airsim-live-validation`, extends
the Phase 6 pure-simulation runtime baseline and Phase 7 optional
multi-simulator backend with an explicitly gated live validation runner for
externally started Cosys-AirSim or legacy AirSim sessions. Normal tests and
smoke checks run without GPU, Cosys-AirSim, legacy AirSim, ROS2, Isaac Sim, PX4,
ArduPilot, MAVSDK, Nav2, SITL, Unreal, or real hardware. Optional Isaac Sim,
ROS2 sensor synchronization, MAVSDK / PX4 SITL, and AirSim-family paths remain
guarded opt-ins.

## Motivation

Sparse-reward robotics tasks make it difficult to learn useful long-horizon
navigation behavior from direct task success alone. This framework explores
whether generated world models, physically grounded scenario variation, and
mock-first deployment interfaces can provide a safer and more testable path
from simulation research toward future runtime validation.

The central design principle is:

> Physical Consistency > Pixel Fidelity

The current checkpoint prioritizes modular interfaces, reproducible tests,
runtime guardrails, explicit simulator boundaries, and safe defaults over
ungated high-fidelity simulator execution or real flight.

## Checkpoint Lineage

Recent released checkpoints:

- `v0.6.0-pure-simulation-runtime`: Phase 6 pure-simulation runtime integration
  with guarded Isaac Sim / Isaac Lab, ROS2, PX4 SITL, MAVSDK, and GWM/WAM
  simulation-demo paths.
- `v0.7.0-optional-airsim-backend`: Phase 7 optional simulator backend
  expansion with Cosys-AirSim preferred, legacy AirSim fallback, backend key
  `airsim`, multisim wrapper, and comparison report.
- `v0.7.1-cosys-airsim-live-validation`: Phase 7-B optional live validation
  runner for externally started Cosys-AirSim / legacy AirSim sessions, with
  no-gate safe skip behavior.

## Completion Framing

`v0.7.1-cosys-airsim-live-validation` completes the current research-framework
artifact. The remaining roadmap items are classified in `docs/roadmap.md` as
completed in mock-first / guarded-runtime form, planned research extensions,
deferred work beyond the current project scope, or safety out-of-scope items.
They should not be read as accidental incompletion of the repository. Phase
8-A completes this roadmap closure framing.

## System Overview

The framework is organized around seven connected research layers:

- Latent and generated world-model training for prediction, uncertainty
  estimation, future observation rollout, and trajectory scoring.
- Future Frame Projection geometry priors for lightweight camera-motion
  consistency without large diffusion or transformer video models.
- Real2Sim2Real data flow for scenario extraction, domain randomization, and
  evaluation loops.
- Digital-twin scene description and guarded Isaac Sim runtime adapters,
  including pure-simulation runtime checks and fake-backend tests by default.
- Mock-first ROS2, DDS, MAVLink, hardware, Nav2-style, and sensor-sync
  interfaces that preserve importability without optional robotics runtimes.
- Safety and coordination modules, including takeover logic, CBF-style command
  filtering, priority-based swarm assignment, shared latent maps, and an
  end-to-end GWM navigation demo.
- Optional simulator backend expansion, including a Cosys-AirSim-preferred
  AirSim-family backend, legacy AirSim fallback, multi-simulator wrapper, and
  guarded live validation runner.

Every required test remains executable on a plain Python environment. Guarded
real-runtime paths raise clear errors instead of silently enabling unsupported
hardware or simulator behavior.

## Contributions

The current checkpoint contributes:

- A modular research codebase with typed interfaces, mock environments, and a
  full regression suite.
- A simulation-driven training and evaluation path for baseline and latent
  world models.
- A Generated World Model core with observation encoding, action conditioning,
  lightweight video dynamics, autoregressive rollout, trajectory sampling, and
  deterministic trajectory scoring.
- A Future Frame Projection geometry prior for RGB/depth projection across
  poses with explicit coordinate-conversion metadata.
- A guarded Isaac Sim runtime adapter and `BaseNavigationEnv` wrapper that
  import and test without Isaac Sim.
- A mock-first ROS2 sensor synchronization path that aligns RGB, depth, LiDAR,
  optional IMU, and odometry into `SensorObservation` objects.
- A guarded MAVSDK / PX4 SITL command path with fake-client tests, explicit
  SITL opt-in, and no automatic PX4 launch.
- An end-to-end GWM navigation demo that connects mock observations,
  `ObservationBuffer`, generated rollouts, candidate scoring, CBF safety
  filtering, and mock command execution.
- A Phase 5 runtime-readiness layer with read-only capability detection,
  guarded Isaac, ROS2, and MAVSDK/PX4 smoke tests, and closed-loop readiness
  reporting.
- A Phase 6 pure-simulation runtime integration layer for Isaac Sim / Isaac Lab,
  ROS2 simulation sensor bridging, externally managed PX4 SITL command
  validation through MAVSDK, Isaac/PX4 bridge design, a guarded GWM/WAM
  simulation demo, and mandatory CBF safety-gate checks before command writes.
- A Phase 7 multi-simulator backend layer where Cosys-AirSim / `cosysairsim` is
  the preferred AirSim-family runtime, legacy AirSim / `airsim` is fallback, and
  the backend registry key remains `airsim`.
- A Phase 7-B optional live validation runner for externally started
  Cosys-AirSim or legacy AirSim sessions. No-gate runs skip safely, and
  zero-command validation requires `GWM_ALLOW_AIRSIM_API_CONTROL=1`.
- Distributed multi-agent coordination infrastructure with mock DDS transport,
  priority assignment, and shared latent map behavior.
- Deployment-facing mock interfaces for MAVLink, hardware state, Nav2-style
  costmaps/planners, and baseline CBF-style command filtering.

## Evaluation Status

The latest Phase 7-B stabilization run completed with:

```text
402 passed, 12 skipped
```

The no-gate optional runtime checks completed safely:

```text
airsim_live_validation status=skipped frames=0/0 commands=0 closed=false
airsim_runtime_smoke status=skipped frames=0/3 closed=false
simulator_backend_comparison status=passed backends=airsim,isaac,mock
```

This verification covers repository tests and mock-first integration checks. It
is not a claim of live Cosys-AirSim validation, live legacy AirSim validation,
real flight validation, SITL/HIL readiness, production safety, or certification
evidence.

## Safety Defaults

Deployment remains locked down by default:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

Optional Isaac Sim, ROS2 sensor synchronization, MAVSDK / PX4 SITL, and
AirSim-family paths require explicit opt-in. Real hardware and autonomous real
flight flags remain disabled by default and are rejected by guarded runtime
paths. The CBF module is a baseline runtime filter and should not be interpreted
as a formal barrier-certificate proof for hardware deployment.

## Limitations

The current framework does not implement real hardware flight validation,
autonomous real flight, formal CBF certification, automatic PX4 launch,
automatic Cosys-AirSim / Unreal launch, production deployment readiness, real
Nav2 plugins, or real `ros2_control` C++ plugins. Optional Isaac Sim, ROS2,
MAVSDK, PX4 SITL, and AirSim-family paths are guarded integration hooks rather
than required runtimes.

## Future Work

Future work can extend the checkpoint through planned research extensions such
as PPO / SAC fine-tuning, metrics dashboards, Sim2Real gap tracking, audited
coordinate conversion, richer simulator comparisons, externally managed runtime
experiments, runtime latency measurement, and safety analysis. Real-world
flight experiments, certified safety, hardware launch automation, and
production deployment remain outside the current project scope for safety and
scope-control reasons.

## Citation

```bibtex
@software{gwm_uav_nav_2026,
  title   = {World-Model-Guided Digital-Twin UAV Navigation Research Framework},
  author  = {Neura-Shadow},
  year    = {2026},
  url     = {https://github.com/Neura-Shadow/GWM-UAV-Navigation-Sparse-Rewards},
  note    = {Sparse-reward navigation via generated world models, asymmetric control,
             Real2Sim2Real data engines, and mock-first deployment interfaces}
}
```
