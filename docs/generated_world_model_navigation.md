# Generated World Model Navigation

## Status

Phase 4-A introduces the Generated World Model core for GWM-UAV:

```text
past observations + action sequence
  -> generated future observations
  -> autoregressive rollout
  -> trajectory scoring
```

This is a lightweight PyTorch baseline. It is designed for local CPU tests and
synthetic data, not for real flight or high-fidelity video generation.

## Core Idea

The UAV receives a short context window containing RGB, depth, pose, and
velocity. A candidate action sequence conditions a small recurrent dynamics
model, which predicts future RGB frames, depth maps, latent states, and
uncertainty. Candidate trajectories are ranked using goal progress, generated
depth risk, uncertainty, energy, smoothness, and safety-boundary placeholders.

The interface is inspired by aerial and navigation world model research such as
ANWM, AirScape, NWM, MWM, and AR Forcing, but Phase 4-A intentionally avoids
large diffusion, transformer, and CDiT-style models.

## Package

The Phase 4-A package is under:

```text
src/generated_world_model/
```

Important interfaces:

- `ObservationEncoder`
- `ActionConditioner`
- `VideoDynamicsModel`
- `AutoregressiveRollout`
- `TrajectoryScorer`
- `GeneratedWorldModelDataset`
- `CandidateTrajectorySampler`
- `GeneratedWorldModelPlanner`

Synthetic training can be run with:

```bash
python scripts/train_generated_world_model.py --synthetic --steps 20
```

## Dataset Layout

The intended dataset layout is:

```text
datasets/generated_world_model/
  metadata.json
  sequences/
    seq_000001.npz
```

Each `.npz` shard stores context RGB/depth/pose/velocity, future
RGB/depth/pose/velocity, and action sequences. Tests create synthetic shards in
temporary directories only.

Generated datasets, checkpoints, results, and outputs should not be committed.

## Safety Boundary

Phase 4-A does not enable:

- Future Frame Projection
- Isaac Sim runtime
- ROS2 image/depth synchronization
- MAVSDK real runtime
- PX4 SITL
- real UAV hardware
- autonomous real flight
- diffusion models
- transformer video models
- Replicator
- Phase 4-B, 4-C, 4-D, 4-E, or 4-F

Deployment defaults remain safe. This slice adds planning and training
interfaces only.
