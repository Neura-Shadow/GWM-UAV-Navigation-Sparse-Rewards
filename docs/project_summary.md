# World-Model-Guided Digital-Twin UAV Navigation Research Framework

## Abstract

This project is a mock-first research framework for sparse-reward navigation
with UAV, UGV, and AMR agents. It combines latent world models, asymmetric
control, Real2Sim2Real scenario generation, OpenUSD-style digital-twin scene
descriptors, ROS2-style middleware abstractions, distributed multi-agent
coordination, and deployment-facing safety interfaces. The `v0.3.0-mock-first`
checkpoint is designed to run and test without GPU, AirSim, ROS2, Isaac Sim,
PX4, ArduPilot, MAVSDK, Nav2, or real hardware.

## Motivation

Sparse-reward robotics tasks make it difficult to learn useful navigation
behavior from direct task success alone. This framework explores whether latent
world models, physically grounded scenario variation, and mock-first deployment
interfaces can provide a safer and more testable path from simulation research
to future real-world validation.

The central design principle is:

> Physical Consistency > Pixel Fidelity

The current checkpoint prioritizes modular interfaces, reproducible tests, and
safe defaults over high-fidelity runtime execution.

## System Overview

The framework is organized around five connected research layers:

- Latent world-model training for prediction, uncertainty estimation, and
  policy support.
- Real2Sim2Real data flow for scenario extraction, domain randomization, and
  evaluation loops.
- Digital-twin scene description using pure-Python, OpenUSD-style descriptor
  dictionaries and JSON export.
- Mock-first ROS2, DDS, MAVLink, hardware, and Nav2-style interfaces that
  preserve importability without optional robotics runtimes.
- Safety and coordination modules, including takeover logic, CBF-style command
  filtering, priority-based swarm assignment, and shared latent maps.

Every required test remains executable on a plain Python environment. Guarded
real-runtime paths raise clear errors instead of silently enabling unsupported
hardware or simulator behavior.

## Contributions

The `v0.3.0-mock-first` checkpoint contributes:

- A modular research codebase with typed interfaces, mock environments, and a
  full regression suite.
- A simulation-driven training and evaluation path for baseline and latent world
  models.
- A ROS2 bridge layer that imports without ROS2 and supports mock-first adapter
  testing.
- An Isaac Sim / OpenUSD scene descriptor builder that produces descriptor JSON
  without requiring Isaac Sim or OpenUSD.
- Distributed multi-agent coordination infrastructure with mock DDS transport,
  priority assignment, and shared latent map behavior.
- Deployment-facing mock interfaces for MAVLink, hardware state, Nav2-style
  costmaps/planners, and CBF-style command filtering.

## Evaluation Status

The checkpoint verification run completed with:

```text
175 passed
```

This result covers the repository test suite in mock-first mode. It is not a
claim of real flight validation, SITL/HIL readiness, production safety, or
certification evidence.

## Limitations

The current framework does not implement autonomous real flight, real PX4 or
ArduPilot integration, MAVSDK runtime control, real `ros2_control` plugins, real
Nav2 plugins, Isaac Sim runtime execution, SITL/HIL launch automation, or safety
certification proof. The CBF module is a baseline runtime filter and should not
be interpreted as a formal barrier-certificate proof for hardware deployment.

## Future Work

Future work can extend the checkpoint toward simulation and deployment realism
through Isaac Sim runtime generation, real ROS2/DDS validation, SITL/HIL
integration, hardware-in-the-loop testing, runtime latency measurement,
certified safety analysis, and real-world flight experiments. These items remain
outside the `v0.3.0-mock-first` checkpoint.

## Citation

```bibtex
@software{gwm_uav_nav_2026,
  title   = {World-Model-Guided Digital-Twin UAV Navigation Research Framework},
  author  = {Neura-Shadow},
  year    = {2026},
  url     = {https://github.com/Neura-Shadow/GWM-UAV-Navigation-Sparse-Rewards},
  note    = {Sparse-reward navigation via latent world models, asymmetric control,
             Real2Sim2Real data engines, and mock-first deployment interfaces}
}
```
