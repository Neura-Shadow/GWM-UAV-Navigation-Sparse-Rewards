<!-- badges -->
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue?logo=python&logoColor=white)
![License: MIT](https://img.shields.io/badge/license-MIT-green)
![Mock-first framework](https://img.shields.io/badge/status-Mock--first%20checkpoint-brightgreen)
![Research](https://img.shields.io/badge/type-Research%20Framework-blueviolet)

# World-Model-Guided Digital-Twin UAV Navigation Research Framework

> Physical Consistency > Pixel Fidelity

This repository is a mock-first research framework for UAV / UGV / AMR
navigation under sparse rewards. It connects latent world models,
Real2Sim2Real scenario generation, OpenUSD-style digital-twin descriptors,
ROS2 bridge architecture, distributed multi-agent coordination, and safe
deployment-interface primitives.

The code is designed to run and test without GPU, AirSim, ROS2, Isaac Sim,
PX4, ArduPilot, MAVSDK, Nav2, or real hardware.

## Current Status

Completed mock-first slices:

| Slice | Status | Key capability |
| --- | --- | --- |
| Phase 1 | Complete | Research-ready modular refactor, mocks, docs, tests |
| Phase 2 | Complete mock-first slice | World-model training, sparse curriculum, Real2Sim2Real pipeline |
| Phase 3-A | Complete mock-first slice | ROS2 bridge and adapter contracts with guarded imports |
| Phase 3-B | Complete mock-first slice | Isaac Sim / OpenUSD-style descriptor builder and JSON export |
| Phase 3-C | Complete mock-first slice | Distributed coordination, DDS-style channel, shared latent map |
| Phase 3-D | Complete mock-first slice | MAVLink, hardware, Nav2-style, and CBF deployment interfaces |

Future runtime work is not started: real SITL/HIL automation, real hardware
flight, real Nav2 plugins, real `ros2_control` C++ plugins, Isaac Sim runtime
execution, and certification proof.

## Safety Defaults

The deployment layer is safe by default:

```yaml
deployment:
  mock: true
  real_hardware_enabled: false
  autonomous_real_flight_enabled: false
```

Phase 3-D does not enable autonomous real flight. `ControlBarrierFunction` is a
baseline runtime filter, not a formal certification proof.

## Architecture

```mermaid
flowchart TD
    A["Mock / future real sensors"] --> B["ROS2 / DDS adapter layer"]
    B --> C["World model encoder"]
    C --> D["Latent dynamics predictor"]
    D --> E["Uncertainty estimator"]
    E --> F{"Confidence high?"}
    F -- "yes" --> G["World-model-guided planner"]
    F -- "no" --> H["Safety controller / CBF filter"]
    G --> I["Mock-first deployment interface"]
    H --> I
    I --> J["Mock vehicle state / future hardware"]

    K["Real or mock logs"] --> L["Scenario extractor"]
    L --> M["Digital twin scene builder"]
    M --> N["OpenUSD-style descriptor JSON"]
    N --> O["Domain randomization"]
    O --> P["Training / evaluation"]
    P --> G
```

Core package areas:

- `src/world_model/`: latent encoders, dynamics, uncertainty, policy intent.
- `src/rl/`: replay buffer, trainer, sparse curriculum, baseline world model.
- `src/digital_twin/`: scenario extraction, domain randomization, scene descriptors.
- `src/control/`: planner, takeover, safety controller, CBF filter.
- `src/ros2_bridge/`: ROS2 adapter, MAVLink/hardware/Nav2-style mock interfaces.
- `src/multi_agent/`: agent registry, mock DDS, shared maps, swarm coordinator.
- `src/evaluation/`: metrics and sim-to-real gap helpers.

## Quick Start

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m pytest tests/ -q
```

The test suite is the main readiness check and must pass without optional
robotics runtimes.

## Script Smoke Commands

```bash
python scripts/train_baseline.py --env mock --max-steps 20
python scripts/train_world_model.py --env mock --model baseline --steps 100
python scripts/train_world_model.py --env mock --model latent --steps 100
python scripts/run_real2sim2real_loop.py --mock --episode-steps 30 --variants 2
python scripts/run_digital_twin_generation.py --num-variations 2
python scripts/evaluate_policy.py --env mock --num-episodes 3
```

Help is available for every script:

```bash
python scripts/train_baseline.py --help
python scripts/train_world_model.py --help
python scripts/run_real2sim2real_loop.py --help
python scripts/run_digital_twin_generation.py --help
python scripts/evaluate_policy.py --help
python scripts/diagnose_airsim.py --help
```

## Documentation

- [Architecture](docs/architecture.md)
- [Roadmap](docs/roadmap.md)
- [ROS2 integration](docs/ros2_integration.md)
- [Deployment hardware interface](docs/deployment_hardware_interface.md)
- [Digital twin / Isaac Sim / OpenUSD](docs/digital_twin_isaac_sim_openusd.md)
- [Multi-agent shared world model](docs/multi_agent_shared_world_model.md)
- [Real2Sim2Real pipeline](docs/real2sim2real_pipeline.md)
- [Sparse rewards](docs/sparse_rewards.md)

## Citation / Research Direction

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
