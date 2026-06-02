# Digital Twin with Isaac Sim and OpenUSD

## Vision

The digital twin layer serves as a **generative environment proxy** — a physics-consistent, photorealistic simulation that can produce unlimited training scenarios from a compact scene specification.  By leveraging NVIDIA Isaac Sim and the OpenUSD scene format, we aim to:

1. Generate high-fidelity environments that mirror real-world deployment sites.
2. Apply systematic domain randomization to improve sim-to-real transfer.
3. Enable photorealistic rendering for visual policy training.
4. Support multi-agent simulation with deterministic physics.

The key insight is: **the digital twin is not a static replica — it is a generative engine** that produces diverse training environments from real-world scenarios.

---

## Isaac Sim Scene Generation Pipeline

```mermaid
flowchart TD
    A[Scenario Specification] --> B[Scene Builder]
    B --> C[OpenUSD Stage]
    C --> D[Physics Configuration]
    D --> E[Sensor Setup]
    E --> F[Domain Randomizer]
    F --> G[Variant 1 .usd]
    F --> H[Variant 2 .usd]
    F --> I[Variant N .usd]
    G --> J[RL Training Environment]
    H --> J
    I --> J
```

### Pipeline stages

1. **Scenario Specification** — A YAML file describing obstacles, goals, physics params, and randomization ranges.
2. **Scene Builder** — Translates the specification into an OpenUSD stage with prims for each obstacle, the ground plane, and the vehicle.
3. **Physics Configuration** — Sets up PhysX parameters: gravity, friction materials, aerodynamic drag coefficients.
4. **Sensor Setup** — Attaches simulated sensors (LiDAR, depth camera, IMU) to the vehicle prim.
5. **Domain Randomizer** — Applies Isaac Sim's built-in randomization to produce N distinct environment variants.

---

## OpenUSD Scene Format

[OpenUSD](https://openusd.org/) (Universal Scene Description) is the standard scene format used by Isaac Sim and Omniverse.  Key concepts:

| Concept | Description |
|---------|-------------|
| **Stage** | The root container for a scene (`.usd` / `.usda` / `.usdc` file) |
| **Prim** | A scene element (mesh, xform, camera, physics body) |
| **Property** | Attributes and relationships on prims |
| **Layer** | Composable scene overrides (used for domain randomization) |
| **Variant Set** | Named alternatives for a prim (e.g., different obstacle textures) |

A typical scene hierarchy:

```
/World
  /GroundPlane
  /Obstacles
    /Obstacle_0  (Sphere, radius=3.0, position=[30,10,-8])
    /Obstacle_1  (Sphere, radius=2.0, position=[45,15,-6])
  /Vehicle
    /UAV
      /Sensors
        /LiDAR
        /DepthCamera
        /IMU
  /Goal  (Xform, position=[60,20,-8])
```

---

## Scene Specification Format (Our YAML/JSON)

Our framework uses a simplified YAML format that is translated into OpenUSD by the scene builder:

```yaml
scene:
  name: "warehouse_nav"
  ground:
    type: "plane"
    size: [200.0, 200.0]
    friction: 0.8

  obstacles:
    - type: "sphere"
      position: [30.0, 10.0, -8.0]
      radius: 3.0
      material: "concrete"
    - type: "box"
      position: [45.0, 15.0, -6.0]
      size: [2.0, 2.0, 4.0]
      material: "metal"

  vehicle:
    type: "uav"
    start_position: [0.0, 0.0, -5.0]
    sensors:
      - type: "lidar"
        range: 50.0
        fov: 360.0
      - type: "depth_camera"
        resolution: [640, 480]
        fov: 90.0

  goal:
    position: [60.0, 20.0, -8.0]
    reach_radius: 3.0

  physics:
    gravity: [0.0, 0.0, -9.81]
    wind: [0.0, 0.0, 0.0]
    time_step: 0.01
```

---

## Domain Randomization in Isaac Sim

Isaac Sim provides native support for domain randomization through its Replicator API.  We plan to randomise:

### Geometry randomization
- Obstacle positions: uniform noise within ± N metres
- Obstacle sizes: scale factor 0.8–1.2×
- Additional distractor objects

### Physics randomization
- Wind speed and direction
- Surface friction coefficients
- Vehicle mass and drag coefficients
- Motor response latency

### Visual randomization (for RGB policies)
- Texture randomization on all surfaces
- Lighting direction, intensity, and colour temperature
- Sky / background HDR maps
- Camera noise and lens distortion

### Sensor randomization
- LiDAR noise standard deviation
- Depth camera dropout probability
- IMU bias and drift rates

Each randomization parameter is specified as a range in the scenario YAML.  The domain randomizer samples uniformly (or from a configured distribution) within these ranges to produce each variant.

---

## Current Implementation: Mock-First Scene Builders

The current digital-twin scene layer provides two mock-first builders:

1. `SimSceneBuilder` produces the existing simulator-agnostic JSON/YAML scene
   descriptor.
2. `IsaacSimSceneBuilder` produces an OpenUSD-style descriptor dictionary with
   stable prim paths for `/World`, `/World/GroundPlane`, obstacles, vehicle,
   sensors, and goal.

`IsaacSimSceneBuilder.build(spec)` always works without NVIDIA GPU, Isaac Sim,
or OpenUSD installed. It preserves the project's current coordinate convention
exactly and records the pending Isaac conversion explicitly:

```python
from src.digital_twin import IsaacSimSceneBuilder

builder = IsaacSimSceneBuilder()
descriptor = builder.build(scenario_spec)
builder.export_json(descriptor, "outputs/scenes/scene_001.json")
```

Descriptor metadata includes:

```yaml
source_coordinate_frame: "project_default"
target_coordinate_frame: "isaac_z_up_pending"
coordinate_conversion_applied: false
```

`build_usd_stage(spec, output_path)` is guarded and raises a clear
`RuntimeError` when Isaac Sim / OpenUSD Python APIs are unavailable. Phase 3-B
exports descriptor JSON first; real `.usd` / `.usda` generation remains an
optional later step.

Switching builders is done through the factory:

```python
from src.digital_twin import SimSceneBuilder

mock_builder = SimSceneBuilder.create(backend="mock")
isaac_descriptor_builder = SimSceneBuilder.create(backend="isaac_sim")
```

---

## Future Integration Plan

### Phase 3-B2: Isaac Sim Runtime Connection

1. Add real OpenUSD stage generation with installed Isaac Sim / OpenUSD APIs.
2. Convert the project coordinate frame to Isaac Z-up explicitly.
3. Connect to a headless Isaac Sim instance for batch scene generation.
4. Use Replicator for domain randomization.

### Phase 3-C+: Full Pipeline

1. Add an Isaac Sim navigation environment wrapper.
2. Bidirectional sync between the latent world model and the Isaac Sim environment.
3. Real-time digital twin mirroring: update the OpenUSD scene from live sensor data.
4. Multi-agent simulation with PhysX-based inter-agent collision detection.
5. Photorealistic synthetic data generation for visual policy pre-training.

### Integration architecture

```mermaid
flowchart LR
    subgraph Framework
        A[ScenarioSpec] --> B[AbstractSceneBuilder]
    end

    subgraph Mock["Mock Backend"]
        B --> C[MockSceneBuilder]
        C --> D[JSON Scene]
    end

    subgraph Isaac["Isaac Sim Backend"]
        B --> E[IsaacSimSceneBuilder]
        E --> F[OpenUSD Stage]
        F --> G[Isaac Sim Runtime]
        G --> H[RL Gym Environment]
    end
```

The adapter pattern ensures that switching from mock to Isaac Sim requires only a configuration change:

```yaml
digital_twin:
  type: "isaac_sim"  # was "mock"
  isaac_sim:
    headless: true
    gpu_id: 0
```
