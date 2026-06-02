"""Shared types for the Generated World Model core."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import torch


@dataclass(frozen=True)
class GWMConfig:
    """Configuration for the lightweight Phase 4-A generated world model."""

    image_channels: int = 3
    depth_channels: int = 1
    image_height: int = 32
    image_width: int = 32
    pose_dim: int = 6
    velocity_dim: int = 3
    action_dim: int = 3
    intention_dim: int = 4
    latent_dim: int = 32
    visual_feature_dim: int = 32
    state_feature_dim: int = 16
    conditioning_dim: int = 32
    hidden_dim: int = 64
    context_length: int = 4
    horizon: int = 3

    @classmethod
    def from_any(cls, value: "GWMConfig | Dict[str, Any] | None") -> "GWMConfig":
        """Create a config from None, a dict, or an existing config."""
        if value is None:
            return cls()
        if isinstance(value, cls):
            return value
        allowed = {field_name for field_name in cls.__dataclass_fields__}
        filtered = {key: val for key, val in value.items() if key in allowed}
        return cls(**filtered)


@dataclass
class ObservationBatch:
    """Batched observation tensors for generated-world-model training.

    Shapes:
        rgb: ``[B, T, 3, H, W]``
        depth: ``[B, T, 1, H, W]``
        pose: ``[B, T, pose_dim]``
        velocity: ``[B, T, 3]``
    """

    rgb: torch.Tensor
    depth: torch.Tensor
    pose: torch.Tensor
    velocity: torch.Tensor
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def batch_size(self) -> int:
        return int(self.rgb.shape[0])

    @property
    def sequence_length(self) -> int:
        return int(self.rgb.shape[1])

    def to(self, device: torch.device | str) -> "ObservationBatch":
        """Move all tensors to a device."""
        return ObservationBatch(
            rgb=self.rgb.to(device),
            depth=self.depth.to(device),
            pose=self.pose.to(device),
            velocity=self.velocity.to(device),
            metadata=dict(self.metadata),
        )


@dataclass
class ActionSequence:
    """Batched action sequence with optional pose and intention conditioning."""

    actions: torch.Tensor
    poses: Optional[torch.Tensor] = None
    intentions: Optional[torch.Tensor] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def horizon(self) -> int:
        return int(self.actions.shape[1])

    def to(self, device: torch.device | str) -> "ActionSequence":
        """Move all tensors to a device."""
        return ActionSequence(
            actions=self.actions.to(device),
            poses=None if self.poses is None else self.poses.to(device),
            intentions=None if self.intentions is None else self.intentions.to(device),
            metadata=dict(self.metadata),
        )


@dataclass
class GeneratedObservation:
    """One predicted future observation step."""

    predicted_rgb: torch.Tensor
    predicted_depth: torch.Tensor
    predicted_latent: torch.Tensor
    uncertainty: torch.Tensor
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class GeneratedRollout:
    """Predicted future observation sequence."""

    predicted_rgb: torch.Tensor
    predicted_depth: torch.Tensor
    predicted_latent: torch.Tensor
    uncertainty: torch.Tensor
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def horizon(self) -> int:
        return int(self.predicted_rgb.shape[1])

    def step(self, index: int) -> GeneratedObservation:
        """Return a single generated step."""
        return GeneratedObservation(
            predicted_rgb=self.predicted_rgb[:, index],
            predicted_depth=self.predicted_depth[:, index],
            predicted_latent=self.predicted_latent[:, index],
            uncertainty=self.uncertainty[:, index],
            metadata=dict(self.metadata),
        )

    def to_dict(self) -> Dict[str, torch.Tensor]:
        """Return tensor outputs in a simple dictionary."""
        return {
            "predicted_rgb": self.predicted_rgb,
            "predicted_depth": self.predicted_depth,
            "predicted_latent": self.predicted_latent,
            "uncertainty": self.uncertainty,
        }


@dataclass
class TrajectoryCandidate:
    """Candidate UAV trajectory used by the scorer and planner."""

    positions: Any
    actions: Optional[Any] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TrajectoryScore:
    """Deterministic score for a trajectory candidate."""

    total_score: float
    components: Dict[str, float]
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-safe score dictionary."""
        return {
            "total_score": float(self.total_score),
            "components": {key: float(value) for key, value in self.components.items()},
            "metadata": dict(self.metadata),
        }
