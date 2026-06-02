"""Action and intention conditioning for generated UAV rollouts."""

from __future__ import annotations

from typing import Any, Dict, Optional

import torch
import torch.nn as nn

from src.generated_world_model.types import ActionSequence, GWMConfig


class ActionConditioner(nn.Module):
    """Encode action sequences, optional poses, and optional intentions."""

    def __init__(self, config: GWMConfig | Dict[str, Any] | None = None) -> None:
        super().__init__()
        self.config = GWMConfig.from_any(config)
        input_dim = self.config.action_dim + self.config.pose_dim + self.config.intention_dim
        self.network = nn.Sequential(
            nn.Linear(input_dim, self.config.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.config.hidden_dim, self.config.conditioning_dim),
            nn.ReLU(),
        )

    def encode(
        self,
        actions: torch.Tensor | ActionSequence,
        poses: Optional[torch.Tensor] = None,
        intentions: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """Return action conditioning shaped ``[B, H, conditioning_dim]``."""
        if isinstance(actions, ActionSequence):
            sequence = actions
            actions_tensor = sequence.actions
            poses = sequence.poses if poses is None else poses
            intentions = sequence.intentions if intentions is None else intentions
        else:
            actions_tensor = actions

        if actions_tensor.ndim != 3:
            raise ValueError("actions must have shape [B, H, action_dim].")
        if actions_tensor.shape[-1] != self.config.action_dim:
            raise ValueError(f"action_dim mismatch: expected {self.config.action_dim}.")

        batch_size, horizon, _ = actions_tensor.shape
        device = actions_tensor.device
        dtype = actions_tensor.dtype
        pose_tensor = _optional_sequence(
            poses,
            batch_size=batch_size,
            horizon=horizon,
            dim=self.config.pose_dim,
            device=device,
            dtype=dtype,
            name="poses",
        )
        intention_tensor = _optional_sequence(
            intentions,
            batch_size=batch_size,
            horizon=horizon,
            dim=self.config.intention_dim,
            device=device,
            dtype=dtype,
            name="intentions",
        )

        conditioning_input = torch.cat(
            [actions_tensor.float(), pose_tensor.float(), intention_tensor.float()],
            dim=-1,
        )
        flat = conditioning_input.reshape(batch_size * horizon, conditioning_input.shape[-1])
        encoded = self.network(flat)
        return encoded.reshape(batch_size, horizon, self.config.conditioning_dim)

    def forward(
        self,
        actions: torch.Tensor | ActionSequence,
        poses: Optional[torch.Tensor] = None,
        intentions: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """Alias for :meth:`encode`."""
        return self.encode(actions, poses=poses, intentions=intentions)


def _optional_sequence(
    value: Optional[torch.Tensor],
    *,
    batch_size: int,
    horizon: int,
    dim: int,
    device: torch.device,
    dtype: torch.dtype,
    name: str,
) -> torch.Tensor:
    if value is None:
        return torch.zeros(batch_size, horizon, dim, device=device, dtype=dtype)
    if value.ndim == 2:
        value = value.unsqueeze(1).expand(batch_size, horizon, dim)
    if value.ndim != 3:
        raise ValueError(f"{name} must have shape [B, H, {dim}] or [B, {dim}].")
    if value.shape != (batch_size, horizon, dim):
        raise ValueError(f"{name} shape mismatch: expected {(batch_size, horizon, dim)}.")
    return value.to(device=device, dtype=dtype)
