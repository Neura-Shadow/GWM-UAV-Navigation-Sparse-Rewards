"""Dataset helpers for generated-world-model `.npz` sequence shards."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch

from src.generated_world_model.types import ActionSequence, GWMConfig, ObservationBatch


class GeneratedWorldModelDataset:
    """Load generated-world-model sequences from a metadata + `.npz` layout."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self.metadata_path = self.root / "metadata.json"
        self.sequence_dir = self.root / "sequences"
        self.metadata = _load_metadata(self.metadata_path)
        self.sequence_paths = sorted(self.sequence_dir.glob("*.npz"))

    def __len__(self) -> int:
        return len(self.sequence_paths)

    def __getitem__(self, index: int) -> Dict[str, torch.Tensor]:
        return load_npz_sequence(self.sequence_paths[index])


def create_synthetic_batch(
    batch_size: int = 2,
    context_length: int = 4,
    horizon: int = 3,
    image_height: int = 32,
    image_width: int = 32,
    pose_dim: int = 6,
    action_dim: int = 3,
    intention_dim: int = 4,
    seed: Optional[int] = 7,
) -> Dict[str, Any]:
    """Create deterministic synthetic data for tests and CPU smoke training."""
    generator = torch.Generator()
    if seed is not None:
        generator.manual_seed(seed)

    config = GWMConfig(
        image_height=image_height,
        image_width=image_width,
        pose_dim=pose_dim,
        action_dim=action_dim,
        intention_dim=intention_dim,
        context_length=context_length,
        horizon=horizon,
    )

    context_rgb = torch.rand(
        batch_size, context_length, config.image_channels, image_height, image_width,
        generator=generator,
    )
    context_depth = 0.5 + torch.rand(
        batch_size, context_length, config.depth_channels, image_height, image_width,
        generator=generator,
    )
    actions = torch.randn(batch_size, horizon, action_dim, generator=generator) * 0.2
    intentions = torch.randn(batch_size, horizon, intention_dim, generator=generator) * 0.1

    context_pose = torch.zeros(batch_size, context_length, pose_dim)
    context_velocity = torch.zeros(batch_size, context_length, 3)
    pose_noise = torch.randn(batch_size, context_length, min(3, pose_dim), generator=generator) * 0.05
    context_pose[:, :, :pose_noise.shape[-1]] = torch.cumsum(pose_noise, dim=1)
    context_velocity[:] = torch.randn(batch_size, context_length, 3, generator=generator) * 0.1

    future_pose = torch.zeros(batch_size, horizon, pose_dim)
    future_velocity = torch.zeros(batch_size, horizon, 3)
    last_pose = context_pose[:, -1:, :].clone()
    action_xyz = _pad_last_dim(actions, 3)
    future_pose[:, :, :3] = last_pose[:, :, :3] + torch.cumsum(action_xyz[:, :, :3], dim=1)
    if pose_dim > 3:
        future_pose[:, :, 3:] = last_pose[:, :, 3:].expand(batch_size, horizon, pose_dim - 3)
    future_velocity[:] = action_xyz[:, :, :3]

    last_rgb = context_rgb[:, -1:, :, :, :].expand(batch_size, horizon, 3, image_height, image_width)
    last_depth = context_depth[:, -1:, :, :, :].expand(batch_size, horizon, 1, image_height, image_width)
    action_shift = actions.mean(dim=-1, keepdim=True).unsqueeze(-1).unsqueeze(-1)
    future_rgb = torch.clamp(last_rgb + action_shift * 0.05, 0.0, 1.0)
    future_depth = torch.clamp(last_depth - action_shift.abs() * 0.02, min=0.05)

    context = ObservationBatch(
        rgb=context_rgb.float(),
        depth=context_depth.float(),
        pose=context_pose.float(),
        velocity=context_velocity.float(),
        metadata={"synthetic": True},
    )
    future = ObservationBatch(
        rgb=future_rgb.float(),
        depth=future_depth.float(),
        pose=future_pose.float(),
        velocity=future_velocity.float(),
        metadata={"synthetic": True},
    )
    return {
        "context": context,
        "future": future,
        "actions": actions.float(),
        "action_sequence": ActionSequence(
            actions=actions.float(),
            poses=future_pose.float(),
            intentions=intentions.float(),
            metadata={"synthetic": True},
        ),
        "config": config,
    }


def save_npz_sequence(path: str | Path, sample: Dict[str, Any]) -> None:
    """Save a synthetic or real sample to an `.npz` sequence shard."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    context = sample["context"]
    future = sample["future"]
    actions = sample.get("actions")
    if actions is None and "action_sequence" in sample:
        actions = sample["action_sequence"].actions
    np.savez_compressed(
        target,
        context_rgb=_to_numpy(context.rgb),
        context_depth=_to_numpy(context.depth),
        context_pose=_to_numpy(context.pose),
        context_velocity=_to_numpy(context.velocity),
        future_rgb=_to_numpy(future.rgb),
        future_depth=_to_numpy(future.depth),
        future_pose=_to_numpy(future.pose),
        future_velocity=_to_numpy(future.velocity),
        actions=_to_numpy(actions),
    )


def load_npz_sequence(path: str | Path) -> Dict[str, torch.Tensor]:
    """Load one `.npz` sequence shard into torch tensors."""
    with np.load(Path(path)) as data:
        return {key: torch.from_numpy(data[key]).float() for key in data.files}


def _load_metadata(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _pad_last_dim(value: torch.Tensor, dim: int) -> torch.Tensor:
    if value.shape[-1] >= dim:
        return value
    padding = torch.zeros(*value.shape[:-1], dim - value.shape[-1], device=value.device)
    return torch.cat([value, padding], dim=-1)
