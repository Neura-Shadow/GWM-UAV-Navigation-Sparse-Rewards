# Development Roadmap

This document tracks the implementation progress of the World-Model-Guided Digital-Twin UAV Navigation Research Framework across three planned phases.

---

## Phase 1: Research-Ready Refactor *(Current)*

Establish a clean, modular codebase with abstract interfaces and mock implementations that allow the full research stack to be developed and tested without any external dependencies.

- [x] Repository structure (`src/`, `tests/`, `scripts/`, `configs/`, `examples/`, `docs/`)
- [x] World model interface (`WorldModelBase`, `DummyWorldModel`)
- [x] Digital twin scene specification (`ScenarioSpec` dataclass)
- [x] ROS2 mock adapter (`MockROS2Adapter`)
- [x] Takeover logic (`TakeoverController` with distance + uncertainty thresholds)
- [x] Evaluation metrics (`EpisodeMetrics`, `MetricsTracker`)
- [x] Multi-agent stubs (`AgentState`, `SharedSpatiotemporalMap`, `FleetCoordinator`)
- [x] Example configurations (single UAV, corner case, multi-agent swarm)
- [x] Placeholder scripts (train, evaluate, generate scenes, R2S2R loop)
- [x] Architecture and research documentation
- [x] Basic tests (importability, metrics, mock episodes)

---

## Phase 2: Simulation-Driven Training

Build out the training and data pipelines, enabling world model pre-training and RL fine-tuning entirely in simulation.

- [x] Domain randomization pipeline (`DomainRandomizer` with configurable parameter ranges)
- [x] Real2Sim2Real loop (scenario extraction → scene building → RL → deploy)
- [x] World model pre-training (contrastive encoder + latent dynamics on collected data)
- [x] Sparse reward curriculum (progressive goal distance, obstacle density scaling)
- [ ] RL fine-tuning loop (PPO / SAC with world-model-augmented rewards)
- [ ] Evaluation metrics dashboard (TensorBoard / Weights & Biases integration)
- [x] Automated test suite (unit + integration, CI/CD pipeline)
- [ ] Sim2Real gap tracking (automated comparison between sim and mock-real trackers)

---

## Phase 3: ROS2 / Isaac Sim / Multi-Agent Integration

Connect the framework to real-world platforms and high-fidelity simulators for deployment-ready validation.

- [ ] ROS2 bridge implementation (`RealROS2Adapter` with `rclpy`)
- [x] Isaac Sim / OpenUSD scene descriptor generation (`IsaacSimSceneBuilder`, mock-first)
- [ ] Isaac Sim RL gym environment wrapper
- [ ] Multi-agent shared map (distributed 4D voxel grid)
- [ ] Swarm coordination (round-robin → priority → consensus)
- [ ] Real-time digital twin mirroring (live sensor → OpenUSD sync)
- [ ] ros2_control integration (hardware interface + controller plugins)
- [ ] Nav2 integration (custom costmap layer + planner plugin)
- [ ] Real deployment interface (PX4 / ArduPilot MAVLink bridge)
- [ ] Safety certification baseline (barrier certificates for Cerebellum controller)

---

## Timeline (Tentative)

| Phase | Target | Key Milestone |
|-------|--------|--------------|
| Phase 1 | ✅ Complete | All interfaces defined, mock stack testable |
| Phase 2 | Q3–Q4 2026 | World model trained, RL fine-tuning in simulation |
| Phase 3 | 2027 | First real-world flight with world-model guidance |

---

## How to Contribute

1. Pick an unchecked item from Phase 2 or Phase 3.
2. Create a feature branch: `git checkout -b feature/<item-name>`.
3. Implement with tests.  All modules must pass `pytest` without GPU / AirSim / ROS2.
4. Submit a pull request referencing this roadmap item.
