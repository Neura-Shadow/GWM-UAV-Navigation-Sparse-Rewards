# Four Research Axes

This document details the four core research axes of the World-Model-Guided Digital-Twin UAV Navigation framework.  Each axis addresses a distinct gap between current autonomous navigation systems and the requirements for robust, safety-critical deployment in unstructured environments.

---

## Axis 1: Latent World Model instead of Pure Geometric Digital Twin

### Motivation

Conventional digital twins replicate geometry and physics in high fidelity, but their explicit state representations scale poorly and fail to capture causal dynamics that govern real-world outcomes.  A purely geometric model can tell you *where* obstacles are, but not *what will happen* if the wind changes or a surface is unexpectedly slippery.

### Key Idea

Replace (or augment) the geometric digital twin with a **learned latent world model** that encodes observations into a compact latent space, predicts future latent states given candidate actions, and estimates the **epistemic uncertainty** of those predictions.  The key insight is: **physical consistency matters more than pixel fidelity**.

### Engineering Design

The world model consists of three components:

1. **Encoder** — Maps raw observations (depth, LiDAR, IMU) into a fixed-size latent vector `z_t`.
2. **Latent Dynamics** — Given `(z_t, a_t)`, predicts `z_{t+1}` and an associated reward estimate.
3. **Uncertainty Estimator** — Uses an ensemble of dynamics heads (or MC-Dropout) to quantify prediction variance.

Planning is performed entirely in latent space via trajectory sampling: the planner imagines multiple action sequences, rolls them out through the dynamics model, ranks them by predicted return, and selects the best.

### Current Minimal Implementation

- Abstract `WorldModelBase` interface with encoder / dynamics / uncertainty methods.
- `DummyWorldModel` that returns zero latent states and uniform uncertainty.
- Uncertainty-gated planner prototype that falls back to a safety controller.

### Future Research Direction

- Contrastive pre-training of the encoder on real + simulated depth data.
- Structured latent spaces (e.g. slot-attention for object-centric representations).
- World model distillation: train a small student model from an expensive teacher for real-time onboard inference.

---

## Axis 2: Asymmetric Control between World Model Brain and ROS2 Cerebellum

### Motivation

End-to-end learned policies struggle with hard safety guarantees.  Certified deterministic controllers are safe but myopic.  Neither alone is sufficient for navigating complex, partially-observed environments under sparse rewards.

### Key Idea

Implement an **asymmetric actor** architecture with two control loops running at different frequencies:

- **Brain** (World Model Planner): slow (~5–20 Hz), deliberative, imagination-based.
- **Cerebellum** (Safety Controller): fast (~100–200 Hz), reflexive, deterministic.

The Cerebellum always has override authority.  The Brain proposes trajectories; the Cerebellum validates and, if necessary, rejects them in favour of a safe fallback.

### Engineering Design

| Property | Brain (World Model) | Cerebellum (Safety Controller) |
|----------|--------------------|---------------------------------|
| Frequency | 5–20 Hz | 100–200 Hz |
| Inputs | Latent state, predicted futures | Raw sensor, obstacle distance |
| Output | Multi-step trajectory | Single velocity command |
| Guarantee | Optimality-seeking | Safety-certified |
| Override | Can be overridden | Cannot be overridden |

Takeover events (when the Cerebellum overrides the Brain) are logged and used as supervision signal for improving the world model.

### Current Minimal Implementation

- `TakeoverController` with distance-threshold and uncertainty-threshold triggers.
- Mock ROS2 adapter that simulates pub/sub without requiring `rclpy`.
- Takeover events are counted in `EpisodeMetrics`.

### Future Research Direction

- Formal verification of the Cerebellum control law (e.g. barrier certificates).
- Adaptive threshold tuning based on environment context.
- Takeover-aware reward shaping: penalise policies that frequently trigger fallbacks.

---

## Axis 3: Real2Sim2Real Digital Twin Data Engine

### Motivation

Sim-to-real transfer remains the Achilles heel of simulation-trained policies.  Static simulation environments do not represent the diversity and messiness of the real world.  Collecting large-scale real data is expensive and dangerous for aerial robots.

### Key Idea

Build a **closed-loop data engine** that continuously shuttles information between the real world and simulation:

1. **Real → Sim** — Extract challenging real-world scenarios (near-misses, high-uncertainty events) from flight logs.
2. **Sim Augmentation** — Reconstruct those scenarios as digital-twin scenes and apply domain randomization.
3. **Policy Improvement** — Fine-tune the policy on randomised simulations.
4. **Sim → Real** — Deploy the improved policy and collect fresh data.

### Engineering Design

```
Real Logs ──▶ Scenario Extractor ──▶ Scene Builder ──▶ Domain Randomizer
    ▲                                                          │
    │                                                          ▼
    └──────────────── Deploy ◀── RL Fine-Tune ◀── Sim Env ◀───┘
```

The scenario extractor identifies critical events (e.g., takeover triggers, high-uncertainty segments) in real logs.  The scene builder generates an OpenUSD scene matching the extracted geometry.  The domain randomizer produces N variations with randomised physics, textures, and obstacle placements.

### Current Minimal Implementation

- `ScenarioSpec` dataclass defining obstacles, goal, physics parameters.
- `MockSceneBuilder` that serialises scenes as JSON.
- `run_real2sim2real_loop.py` placeholder script.

### Future Research Direction

- Automated scenario difficulty scoring and curriculum learning.
- Neural scene reconstruction from onboard RGB-D for zero-shot sim-to-real.
- Photorealistic rendering via Isaac Sim / Omniverse for visual policy training.

---

## Axis 4: Multi-Agent Shared World Model

### Motivation

Real-world deployments increasingly involve fleets of robots.  Independent per-agent planning leads to conflicts, redundant exploration, and suboptimal global coverage.  Sharing information can dramatically improve efficiency, but naive state broadcasting does not scale and lacks the causal structure needed for prediction.

### Key Idea

Extend the latent world model to a **shared, 4D spatiotemporal representation** that multiple agents read from and write to.  Each agent:

1. Encodes its own observations into the shared latent map.
2. Queries the map for predictions about regions it has not yet visited.
3. Plans in latent space while accounting for other agents' planned trajectories.

### Engineering Design

The shared map is a voxel grid indexed by `(x, y, z, t)`.  Each cell stores a latent feature vector and an occupancy probability.  Agents broadcast state updates via DDS topics with QoS-guaranteed delivery.  A coordinator assigns time-slots to avoid spatial conflicts.

| Component | Implementation |
|-----------|---------------|
| Shared Map | `SharedSpatiotemporalMap` — voxel grid with latent features |
| Agent State | `AgentState` dataclass — position, velocity, planned path |
| Coordinator | `FleetCoordinator` — round-robin or priority-based scheduling |
| Communication | DDS topics via ROS2 (mock or real adapter) |

### Current Minimal Implementation

- `AgentState` dataclass with position, velocity, heading, and plan.
- `SharedSpatiotemporalMap` stub with read/write latent features.
- `FleetCoordinator` with round-robin time-slot allocation.
- `multi_agent_swarm.yaml` example config for 4 UAVs.

### Future Research Direction

- Attention-based map fusion to handle heterogeneous agent observations.
- Decentralised consensus protocols for map consistency without a central server.
- Scalability experiments: 4 → 16 → 64 agents.
- Heterogeneous fleets: UAV scouts + UGV ground stations.
