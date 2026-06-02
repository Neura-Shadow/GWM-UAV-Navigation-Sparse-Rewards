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

Phase 4-B adds a Future Frame Projection geometry prior. It warps a past
RGB/depth frame into a future camera viewpoint using depth, pose, and pinhole
camera intrinsics. This is a pure PyTorch/NumPy prototype, not a complete ANWM
implementation.

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
- `FutureFrameProjection`
- `CameraIntrinsics`

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

## Future Frame Projection

`FutureFrameProjection.project()` accepts:

```text
past_frame: [B, C, H, W]
past_depth: [B, 1, H, W]
past_pose: [B, 6] or [B, 4, 4]
future_pose: [B, 6] or [B, 4, 4]
```

The 6D pose convention is `(x, y, z, roll, pitch, yaw)`. The projection
preserves the current project coordinate convention and records
`coordinate_conversion_applied: false` in metadata. Isaac Z-up conversion and
runtime camera-frame calibration remain future work.

The projection returns `projected_rgb`, `projected_depth`, `valid_mask`, and
metadata such as coverage ratio and valid pixel count. The optional
`future_frame_projection_loss()` can compare generated RGB predictions against
projected RGB priors while masking invalid pixels.

## Safety Boundary

Phase 4-A/4-B do not enable:

- Isaac Sim runtime
- ROS2 image/depth synchronization
- MAVSDK real runtime
- PX4 SITL
- real UAV hardware
- autonomous real flight
- diffusion models
- transformer video models
- Replicator
- Phase 4-C, 4-D, 4-E, or 4-F

Deployment defaults remain safe. This slice adds planning and training
interfaces only.
