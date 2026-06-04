# World-Model-Guided Digital-Twin UAV Navigation Research Framework

## Abstract

This project is a mock-first research framework for sparse-reward UAV, UGV, and
AMR navigation. It combines latent world models, generated future-observation
rollouts, asymmetric control, Real2Sim2Real scenario generation, OpenUSD-style
digital-twin descriptors, ROS2-style sensor and command abstractions,
distributed multi-agent coordination, and deployment-facing safety interfaces.

The `v0.4.0-gwm-uav-runtime` checkpoint completes the Phase 4 Generated World
Model UAV runtime baseline. Normal tests and demo smoke checks run without GPU,
AirSim, ROS2, Isaac Sim, PX4, ArduPilot, MAVSDK, Nav2, SITL, or real hardware.
Optional Isaac Sim, ROS2 sensor synchronization, and MAVSDK / PX4 SITL paths
remain guarded opt-ins.

## Motivation

Sparse-reward robotics tasks make it difficult to learn useful long-horizon
navigation behavior from direct task success alone. This framework explores
whether generated world models, physically grounded scenario variation, and
mock-first deployment interfaces can provide a safer and more testable path
from simulation research toward future runtime validation.

The central design principle is:

> Physical Consistency > Pixel Fidelity

The current checkpoint prioritizes modular interfaces, reproducible tests,
runtime guardrails, and safe defaults over high-fidelity simulator execution or
real flight.

## System Overview

The framework is organized around six connected research layers:

- Latent and generated world-model training for prediction, uncertainty
  estimation, future observation rollout, and trajectory scoring.
- Future Frame Projection geometry priors for lightweight camera-motion
  consistency without large diffusion or transformer video models.
- Real2Sim2Real data flow for scenario extraction, domain randomization, and
  evaluation loops.
- Digital-twin scene description and guarded Isaac Sim runtime adapters using
  pure-Python descriptors and fake-backend tests by default.
- Mock-first ROS2, DDS, MAVLink, hardware, Nav2-style, and sensor-sync
  interfaces that preserve importability without optional robotics runtimes.
- Safety and coordination modules, including takeover logic, CBF-style command
  filtering, priority-based swarm assignment, shared latent maps, and an
  end-to-end GWM navigation demo.

Every required test remains executable on a plain Python environment. Guarded
real-runtime paths raise clear errors instead of silently enabling unsupported
hardware or simulator behavior.

## Contributions

The `v0.4.0-gwm-uav-runtime` checkpoint contributes:

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
- Distributed multi-agent coordination infrastructure with mock DDS transport,
  priority assignment, and shared latent map behavior.
- Deployment-facing mock interfaces for MAVLink, hardware state, Nav2-style
  costmaps/planners, and baseline CBF-style command filtering.

## Evaluation Status

The Phase 4 stabilization run completed with:

```text
244 passed, 4 skipped
```

The mock GWM navigation demo smoke check completed with:

```text
gwm_demo status=timeout steps=5 commands=5 safety_overrides=0
```

This verification covers repository tests and mock-first integration checks. It
is not a claim of real flight validation, SITL/HIL readiness, production
safety, or certification evidence.

## Safety Defaults

Deployment remains locked down by default:

```yaml
deployment:
  mock: true
  sitl_enabled: false
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

Optional Isaac Sim, ROS2 sensor synchronization, and MAVSDK / PX4 SITL paths
require explicit opt-in. Real hardware and autonomous real flight flags remain
disabled by default and are rejected by the demo path. The CBF module is a
baseline runtime filter and should not be interpreted as a formal
barrier-certificate proof for hardware deployment.

## Limitations

The current framework does not implement real hardware flight validation,
autonomous real flight, formal CBF certification, automatic PX4 launch,
production deployment readiness, real Nav2 plugins, or real `ros2_control` C++
plugins. Optional Isaac Sim, ROS2, and MAVSDK paths are guarded integration
hooks rather than required runtimes.

## Future Work

Future work can extend the checkpoint toward simulation and deployment realism
through trained generated world models, richer Isaac Sim sensor extraction,
real ROS2/DDS validation, externally managed SITL/HIL experiments, runtime
latency measurement, audited coordinate conversion, certified safety analysis,
and real-world flight experiments. These items remain outside the
`v0.4.0-gwm-uav-runtime` checkpoint.

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
