"""Lightweight observation encoder for RGB-D UAV context windows."""

from __future__ import annotations

from typing import Any, Dict

import torch
import torch.nn as nn

from src.generated_world_model.types import GWMConfig, ObservationBatch


class ObservationEncoder(nn.Module):
    """Encode RGB, depth, pose, and velocity context into latent states."""

    def __init__(self, config: GWMConfig | Dict[str, Any] | None = None) -> None:
        super().__init__()
        self.config = GWMConfig.from_any(config)
        input_channels = self.config.image_channels + self.config.depth_channels

        self.frame_encoder = nn.Sequential(
            nn.Conv2d(input_channels, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(32, self.config.visual_feature_dim),
            nn.ReLU(),
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(self.config.pose_dim + self.config.velocity_dim, self.config.state_feature_dim),
            nn.ReLU(),
        )
        self.fusion = nn.Sequential(
            nn.Linear(self.config.visual_feature_dim + self.config.state_feature_dim, self.config.latent_dim),
            nn.ReLU(),
        )

    def encode(self, batch: ObservationBatch | Dict[str, torch.Tensor]) -> torch.Tensor:
        """Return latent context tensor shaped ``[B, T, latent_dim]``."""
        obs = _coerce_observation_batch(batch)
        _validate_observation_batch(obs, self.config)

        rgb = obs.rgb.float()
        depth = obs.depth.float()
        pose = obs.pose.float()
        velocity = obs.velocity.float()

        batch_size, sequence_length = rgb.shape[:2]
        visual_input = torch.cat([rgb, depth], dim=2)
        visual_input = visual_input.reshape(
            batch_size * sequence_length,
            visual_input.shape[2],
            visual_input.shape[3],
            visual_input.shape[4],
        )
        visual_features = self.frame_encoder(visual_input)

        state_input = torch.cat([pose, velocity], dim=-1)
        state_features = self.state_encoder(
            state_input.reshape(batch_size * sequence_length, state_input.shape[-1])
        )
        latent = self.fusion(torch.cat([visual_features, state_features], dim=-1))
        return latent.reshape(batch_size, sequence_length, self.config.latent_dim)

    def forward(self, batch: ObservationBatch | Dict[str, torch.Tensor]) -> torch.Tensor:
        """Alias for :meth:`encode`."""
        return self.encode(batch)


def _coerce_observation_batch(batch: ObservationBatch | Dict[str, torch.Tensor]) -> ObservationBatch:
    if isinstance(batch, ObservationBatch):
        return batch
    return ObservationBatch(
        rgb=batch["rgb"],
        depth=batch["depth"],
        pose=batch["pose"],
        velocity=batch["velocity"],
        metadata=dict(batch.get("metadata", {})),
    )


def _validate_observation_batch(batch: ObservationBatch, config: GWMConfig) -> None:
    if batch.rgb.ndim != 5:
        raise ValueError("rgb must have shape [B, T, 3, H, W].")
    if batch.depth.ndim != 5:
        raise ValueError("depth must have shape [B, T, 1, H, W].")
    if batch.pose.ndim != 3:
        raise ValueError("pose must have shape [B, T, pose_dim].")
    if batch.velocity.ndim != 3:
        raise ValueError("velocity must have shape [B, T, 3].")

    expected_batch_time = batch.rgb.shape[:2]
    if batch.depth.shape[:2] != expected_batch_time:
        raise ValueError("depth must share rgb batch/time dimensions.")
    if batch.pose.shape[:2] != expected_batch_time:
        raise ValueError("pose must share rgb batch/time dimensions.")
    if batch.velocity.shape[:2] != expected_batch_time:
        raise ValueError("velocity must share rgb batch/time dimensions.")
    if batch.rgb.shape[2] != config.image_channels:
        raise ValueError(f"rgb channel mismatch: expected {config.image_channels}.")
    if batch.depth.shape[2] != config.depth_channels:
        raise ValueError(f"depth channel mismatch: expected {config.depth_channels}.")
    if batch.pose.shape[-1] != config.pose_dim:
        raise ValueError(f"pose_dim mismatch: expected {config.pose_dim}.")
    if batch.velocity.shape[-1] != config.velocity_dim:
        raise ValueError(f"velocity_dim mismatch: expected {config.velocity_dim}.")
