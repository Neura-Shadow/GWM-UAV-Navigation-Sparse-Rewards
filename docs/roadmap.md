# Development Roadmap

This document tracks implementation progress for the World-Model-Guided
Digital-Twin UAV Navigation Research Framework.

---

## Project Completion Status

`v0.7.1-cosys-airsim-live-validation` completes the current
research-framework artifact. The repository now contains the mock-first,
guarded-runtime, pure-simulation/SITL, and optional multi-simulator layers
needed for the scoped project.

Remaining legacy roadmap ideas are not required completion blockers. They are
classified below as completed in mock-first / guarded-runtime form, planned
research extensions, deferred work beyond the current project scope, or
explicitly out of scope for safety reasons.

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
- PPO / SAC fine-tuning: planned research extension
- Metrics dashboard: planned research extension
- [x] Automated test suite
- Sim2Real gap tracking: planned research extension

---

## Phase 3: ROS2 / Isaac Sim / Multi-Agent Integration

Connect the framework to real-world platforms and high-fidelity simulators for
deployment-ready validation.

- [x] Mock-first ROS2 bridge and adapter (`ROS2Bridge`, `RealROS2Adapter`, guarded `rclpy`)
- [x] Isaac Sim / OpenUSD scene descriptor generation (`IsaacSimSceneBuilder`, mock-first)
- [x] Guarded Isaac Sim runtime adapter (`IsaacSimRuntime`, `IsaacSimNavigationEnv`; optional, mock-first)
- Full Isaac Gym wrapper: deferred beyond current project scope
- [x] Distributed multi-agent coordination infrastructure (`ROS2DDSChannel`, `PriorityCoordinator`, `SharedLatentMap`)
- [x] Swarm coordination strategy extension (round-robin and priority; consensus remains future work)
- Live digital twin mirroring: deferred beyond current project scope
- `ros2_control`: completed in mock-first / guarded-runtime form; real controller plugins explicitly out of scope for safety reasons
- Nav2: completed in mock-first / guarded-runtime form; real Nav2 plugins explicitly out of scope for safety reasons
- [x] Guarded PX4 SITL MAVSDK command path (`MAVLinkBridge`; optional, fake-client tested)
- Real deployment interface: completed in guarded-runtime/SITL form; physical UAV deployment explicitly out of scope for safety reasons
- [x] Mock deployment hardware interface (`MAVLinkBridge`, `MockHardwareInterface`, Nav2-style skeletons)
- [x] Baseline CBF-style safety filter (runtime filter only, not certification proof)
- SITL / HIL launch automation: deferred beyond current project scope; HIL/hardware automation explicitly out of scope for safety reasons
- Real hardware validation: explicitly out of scope for safety reasons
- Safety certification proof: explicitly out of scope for safety reasons

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
- [x] Guarded Isaac Sim runtime smoke test (`run_isaac_runtime_smoke`, env-gated)
- [x] Guarded ROS2 sensor synchronization runtime smoke test (`run_ros2_sensor_sync_smoke`, env-gated)
- [x] Guarded MAVSDK / PX4 SITL command-path smoke test (`run_mavsdk_sitl_smoke`, env-gated)
- [x] Closed-loop mock-to-SITL integration readiness (`run_closed_loop_readiness`, mock-first)

---

## Phase 6: Pure-Simulation Full-Stack Runtime Integration

Use real simulation/runtime technologies for a simulation-only UAV stack while
continuing to reject real hardware and autonomous real flight.

- [x] Pure simulation runtime profile (`configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml`)
- [x] Isaac Sim / Isaac Lab sensor runtime execution (`run_isaac_sensor_runtime`, env-gated)
- [x] ROS2 simulation sensor bridge (`run_ros2_sim_sensor_bridge`, env-gated)
- [x] PX4 SITL + MAVSDK command validation (`run_px4_sitl_command_validation`, env-gated)
- [x] Isaac + PX4 SITL closed-loop bridge design (`run_isaac_px4_bridge_design`, dry-run)
- [x] GWM / WAM closed-loop simulation demo (`run_phase6_gwm_simulation_demo`, env-gated)

---

## Phase 7: Multi-Simulator Backend Expansion

Add Cosys-AirSim / `cosysairsim` as the primary AirSim-family optional backend,
with legacy AirSim / `airsim` as a fallback, while leaving the Phase 6 Isaac +
ROS2 + PX4 SITL + MAVSDK mainline stable. The backend registry name remains
`airsim`.

- [x] Simulator backend registry (`mock`, `isaac`, `airsim`)
- [x] Guarded Cosys-AirSim primary / legacy AirSim fallback runtime and navigation environment hardening
- [x] AirSim-family capability detection and disabled-by-default runtime smoke
- [x] Optional live Cosys-AirSim validation runner for externally started sessions
- [x] Multi-simulator GWM demo wrapper with mock default
- [x] Backend comparison report for mock / Isaac readiness / AirSim-family readiness
- Operator-run live validation on an externally started Cosys-AirSim or legacy AirSim session: planned research extension

---

## Completion Classification Matrix

These classifications close the legacy roadmap items that remain useful as
research directions but are not required for the completed `v0.7.1` framework.

| Item | Classification |
| --- | --- |
| PPO / SAC fine-tuning | planned research extension |
| Metrics dashboard | planned research extension |
| Sim2Real gap tracking | planned research extension |
| Full Isaac Gym wrapper | deferred beyond current project scope |
| Live digital twin mirroring | deferred beyond current project scope |
| ros2_control | completed in mock-first / guarded-runtime form; real controller plugins explicitly out of scope for safety reasons |
| Nav2 | completed in mock-first / guarded-runtime form; real Nav2 plugins explicitly out of scope for safety reasons |
| Real deployment interface | completed in guarded-runtime/SITL form; physical UAV deployment explicitly out of scope for safety reasons |
| SITL / HIL launch automation | deferred beyond current project scope; HIL/hardware automation explicitly out of scope for safety reasons |
| Real hardware validation | explicitly out of scope for safety reasons |
| Safety certification proof | explicitly out of scope for safety reasons |
| Optional live Cosys-AirSim / legacy AirSim validation | planned research extension |

The category vocabulary is:

- completed in mock-first / guarded-runtime form
- planned research extension
- deferred beyond current project scope
- explicitly out of scope for safety reasons

---

## Timeline

| Phase | Status | Key Milestone |
| --- | --- | --- |
| Phase 1 | Complete | All interfaces defined, mock stack testable |
| Phase 2 | Complete framework baseline | World model training, sparse curriculum, Real2Sim2Real, and planned research extensions classified |
| Phase 3 | Complete guarded interfaces | ROS2, Isaac, MAVLink, Nav2-style, hardware-style, and CBF interfaces completed in mock-first / guarded-runtime form |
| Phase 4 | Complete | Generated observation rollouts and guarded runtime paths |
| Phase 5 | Complete | Runtime readiness checks and guarded SITL validation |
| Phase 6 | Complete | Pure-simulation Isaac / ROS2 / PX4 SITL closed-loop integration |
| Phase 7 | Complete | Optional Cosys-AirSim primary / legacy AirSim fallback backend and multi-simulator comparison |
| Phase 8-A | Complete | Roadmap closure and research-framework completion framing |
| Phase 8-B | Complete | v0.7.1 documentation artifact consistency pass |
| Phase 8-C | Complete | Placeholder audit and intentionally scoped extension-point cleanup |
| Phase 8-D | Complete | Final verification bundle and mock/no-write-output evidence |
| Phase 8-E | Complete | Final archive tag and GitHub Release documentation |

---

## How to Contribute

1. Start from the completion classification matrix above.
2. Treat `planned research extension` and `deferred beyond current project scope`
   items as explicit new research proposals, not unfinished project blockers.
3. Do not implement items marked `explicitly out of scope for safety reasons`
   inside this project without a separate safety review and scope reset.
4. Keep normal tests runnable without GPU, Isaac Sim, ROS2, Cosys-AirSim,
   legacy AirSim, MAVSDK, PX4, Nav2, SITL, or real hardware.

---

## Post-v1 Optional Extension: GWM-UAV-C2

`v1.0.0-research-framework-complete` remains the completed archived research
artifact.

Future work may explore a separate UAV command and mission intelligence
extension:

- [x] v2-0 C2 concept and scope freeze
- [x] v2-1 Mission data model and event bus
  - [x] v2-1 planning spec
  - [x] v2-1A Mission dataclasses and validation
  - [x] v2-1B Event bus and state store
  - [x] v2-1C Mock replay and metrics
- [ ] v2-2 Mission dispatcher and fleet manager
- [ ] v2-3 Defensive threat and risk prediction
- [ ] v2-4 Risk-aware planning and UTM-style airspace layer
- [ ] v2-5 Dashboard replay and metrics
- [ ] v2-6 Optional simulator benchmark integration

These are optional post-v1 extensions and are not blockers for `v1.0.0`.
