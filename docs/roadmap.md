# Development Roadmap

This document tracks implementation progress for the World-Model-Guided
Digital-Twin UAV Navigation Research Framework.

---

## Phase 1: Research-Ready Refactor

Establish a clean, modular codebase with abstract interfaces and mock
implementations that allow the full research stack to be developed and tested
without external dependencies.

- [x] Repository structure (`src/`, `tests/`, `scripts/`, `configs/`, `examples/`, `docs/`)
- [x] World model interface (`WorldModelBase`, `DummyWorldModel`)
- [x] Digital twin scene specification (`ScenarioSpec` dataclass)
- [x] ROS2 mock adapter (`MockROS2Adapter`)
- [x] Takeover logic (`TakeoverController` with distance and uncertainty thresholds)
- [x] Evaluation metrics (`EpisodeMetrics`, `MetricsTracker`)
- [x] Multi-agent stubs (`AgentState`, `SharedSpatiotemporalMap`, `FleetCoordinator`)
- [x] Example configurations
- [x] Placeholder scripts
- [x] Architecture and research documentation
- [x] Basic tests

---

## Phase 2: Simulation-Driven Training

Build out the training and data pipelines, enabling world model pre-training
and RL fine-tuning entirely in simulation.

- [x] Domain randomization pipeline (`DomainRandomizer` with configurable parameter ranges)
- [x] Real2Sim2Real loop
- [x] World model pre-training
- [x] Sparse reward curriculum
- [ ] RL fine-tuning loop (PPO / SAC with world-model-augmented rewards)
- [ ] Evaluation metrics dashboard (TensorBoard / Weights & Biases integration)
- [x] Automated test suite
- [ ] Sim2Real gap tracking

---

## Phase 3: ROS2 / Isaac Sim / Multi-Agent Integration

Connect the framework to real-world platforms and high-fidelity simulators for
deployment-ready validation.

- [x] Mock-first ROS2 bridge and adapter (`ROS2Bridge`, `RealROS2Adapter`, guarded `rclpy`)
- [x] Isaac Sim / OpenUSD scene descriptor generation (`IsaacSimSceneBuilder`, mock-first)
- [x] Guarded Isaac Sim runtime adapter (`IsaacSimRuntime`, `IsaacSimNavigationEnv`; optional, mock-first)
- [ ] Full Isaac Sim RL gym environment wrapper
- [x] Distributed multi-agent coordination infrastructure (`ROS2DDSChannel`, `PriorityCoordinator`, `SharedLatentMap`)
- [x] Swarm coordination strategy extension (round-robin and priority; consensus remains future work)
- [ ] Real-time digital twin mirroring (live sensor to OpenUSD sync)
- [ ] ros2_control integration (hardware interface and controller plugins)
- [ ] Nav2 integration (custom costmap layer and planner plugin)
- [x] Guarded PX4 SITL MAVSDK command path (`MAVLinkBridge`; optional, fake-client tested)
- [ ] Real deployment interface (PX4 / ArduPilot MAVLink bridge)
- [x] Mock deployment hardware interface (`MAVLinkBridge`, `MockHardwareInterface`, Nav2-style skeletons)
- [x] Baseline CBF-style safety filter (runtime filter only, not certification proof)
- [ ] Real SITL / HIL launch automation
- [ ] Real hardware flight validation
- [ ] Safety certification proof for Cerebellum controller

---

## Phase 4: Generated World Model UAV Navigation

Evolve the mock-first framework toward generated observation rollouts and
real-runtime integration while keeping every slice independently testable.

- [x] Generated World Model core (`ObservationEncoder`, `ActionConditioner`, `VideoDynamicsModel`, `AutoregressiveRollout`, `TrajectoryScorer`)
- [x] Future Frame Projection geometry prior (`FutureFrameProjection`, `CameraIntrinsics`, masked projection loss)
- [x] Guarded Isaac Sim runtime environment wrapper (`IsaacSimRuntime`, `IsaacSimNavigationEnv`)
- [x] Mock-first ROS2 image/depth/lidar/odom synchronization (`ROS2SensorSynchronizer`)
- [x] MAVSDK / PX4 SITL command path (`MAVLinkBridge`, guarded optional MAVSDK)
- [x] End-to-end Generated World Model navigation demo (`GWMDemoRunner`, mock-first CLI)

---

## Phase 5: Real-Runtime Readiness and SITL Integration

Validate optional runtime paths gradually while preserving safe defaults and
mock-first normal tests.

- [x] Runtime capability detection (`RuntimeCapabilityDetector`, read-only probes)
- [ ] Guarded Isaac Sim runtime smoke test
- [ ] Guarded ROS2 sensor synchronization runtime smoke test
- [ ] Guarded MAVSDK / PX4 SITL command-path smoke test
- [ ] Closed-loop mock-to-SITL integration plan

---

## Timeline

| Phase | Target | Key Milestone |
| --- | --- | --- |
| Phase 1 | Complete | All interfaces defined, mock stack testable |
| Phase 2 | Q3-Q4 2026 | World model trained, RL fine-tuning in simulation |
| Phase 3 | 2027 | First real-world flight with world-model guidance |
| Phase 4 | 2027+ | Generated observation rollouts and guarded runtime paths |
| Phase 5 | 2027+ | Real-runtime readiness checks and guarded SITL validation |

---

## How to Contribute

1. Pick an unchecked item from Phase 2 or Phase 3.
2. Create a feature branch: `git checkout -b feature/<item-name>`.
3. Implement with tests. All modules must pass `pytest` without GPU, AirSim, or ROS2.
4. Submit a pull request referencing this roadmap item.
