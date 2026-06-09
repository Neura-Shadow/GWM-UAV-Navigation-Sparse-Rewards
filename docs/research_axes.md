# Four Research Axes

This repository organizes the World-Model-Guided Digital-Twin UAV Navigation
framework around four research axes. Each axis is implemented as a mock-first,
testable Python slice with future runtime integrations kept optional.

## Axis 1: Latent World Model

The world-model layer compresses observations into latent state, predicts
future latent dynamics under candidate actions, and estimates uncertainty.

Current mock-first implementation:

- `WorldModelBase` interface and trainable baseline paths.
- `LatentWorldModel` with encoder, latent dynamics, uncertainty estimation, and
  policy intent mapping.
- `train_world_model.py` smoke path for baseline and latent models.
- Tests that run on CPU without GPU, AirSim, ROS2, or Isaac Sim.

Future research:

- Larger datasets, richer encoders, and onboard latency profiling.
- Object-centric or slot-based latent spaces.
- Real-log pretraining and deployment feedback loops.

## Axis 2: Asymmetric Control

The control layer separates slow deliberative planning from fast safety
fallbacks. The safety layer has override authority.

Current mock-first implementation:

- `HighLevelPlanner`, `SafetyController`, and `TakeoverArbiter`.
- `ROS2Adapter` / `MockROS2Adapter` and guarded `RealROS2Adapter`.
- `ControlBarrierFunction` baseline for command saturation, altitude checks,
  geofence placeholders, and obstacle filtering.
- Deployment defaults keep `mock: true` and `real_hardware_enabled: false`.

Future research:

- Formal CBF proofs and certification evidence.
- Real `ros2_control` plugins and hardware timing validation.
- Field-tested safety envelopes and operator procedures.

## Axis 3: Real2Sim2Real Digital Twin Data Engine

The digital-twin layer extracts challenging scenarios, generates randomized
variants, and emits descriptor artifacts that can later feed high-fidelity
simulation.

Current mock-first implementation:

- `ScenarioSpec`, `ScenarioExtractor`, and `DomainRandomizer`.
- `SimSceneBuilder` descriptor path.
- `IsaacSimSceneBuilder.build()` OpenUSD-style descriptor generation without
  requiring Isaac Sim or OpenUSD.
- `run_real2sim2real_loop.py` mock pipeline report generation.
- `run_digital_twin_generation.py` scoped planning-audit script.

Future research:

- Real Isaac Sim / OpenUSD stage generation.
- Isaac runtime environment wrappers.
- Real-time digital-twin mirroring from live sensors.

## Axis 4: Multi-Agent Shared World Model

The multi-agent layer supports fleet state, shared maps, DDS-style messaging,
and deterministic coordination strategies.

Current mock-first implementation:

- `AgentRegistry`, `SharedSpatiotemporalMap`, and `SharedLatentMap`.
- `MockDDSChannel` and `ROS2DDSChannel` with pure-Python fallback.
- `SwarmCoordinator` with round-robin and priority strategies.
- `PriorityCoordinator` for deterministic task scoring.

Future research:

- Decentralized consensus protocols.
- Heterogeneous UAV / UGV / AMR fleet experiments.
- Large-scale map fusion and communication-load studies.

## Deployment Boundary

Phase 3-D adds mock-first deployment interfaces only:

- `MAVLinkBridge`
- `MockHardwareInterface`
- `ROS2ControlHardwareInterface` guarded stub
- `WorldModelCostmapLayer`
- `WorldModelPlannerPlugin`

PX4, ArduPilot, MAVSDK, Nav2 runtime plugins, real SITL/HIL automation, real
hardware flight, and certification proof remain future work.
