"""Small action-conditioned latent video dynamics baseline."""

from __future__ import annotations

from typing import Any, Dict

import torch
import torch.nn as nn

from src.generated_world_model.types import GWMConfig, GeneratedRollout


class VideoDynamicsModel(nn.Module):
    """Predict future RGB, depth, latent state, and uncertainty from context."""

    def __init__(self, config: GWMConfig | Dict[str, Any] | None = None) -> None:
        super().__init__()
        self.config = GWMConfig.from_any(config)
        self.initial_hidden = nn.Linear(self.config.latent_dim, self.config.hidden_dim)
        self.gru = nn.GRU(
            input_size=self.config.conditioning_dim,
            hidden_size=self.config.hidden_dim,
            batch_first=True,
        )
        self.latent_head = nn.Linear(self.config.hidden_dim, self.config.latent_dim)
        image_size = self.config.image_channels * self.config.image_height * self.config.image_width
        depth_size = self.config.depth_channels * self.config.image_height * self.config.image_width
        self.rgb_head = nn.Linear(self.config.hidden_dim, image_size)
        self.depth_head = nn.Linear(self.config.hidden_dim, depth_size)
        self.uncertainty_head = nn.Linear(self.config.hidden_dim, 1)

    def forward(
        self,
        context: torch.Tensor,
        action_conditioning: torch.Tensor,
    ) -> GeneratedRollout:
        """Predict future observations from latent context and action conditioning."""
        if context.ndim != 3:
            raise ValueError("context must have shape [B, T, latent_dim].")
        if context.shape[-1] != self.config.latent_dim:
            raise ValueError(f"context latent_dim mismatch: expected {self.config.latent_dim}.")
        if action_conditioning.ndim != 3:
            raise ValueError("action_conditioning must have shape [B, H, conditioning_dim].")
        if action_conditioning.shape[0] != context.shape[0]:
            raise ValueError("context and action_conditioning must share batch size.")
        if action_conditioning.shape[-1] != self.config.conditioning_dim:
            raise ValueError(
                f"conditioning_dim mismatch: expected {self.config.conditioning_dim}."
            )

        initial = torch.tanh(self.initial_hidden(context[:, -1, :])).unsqueeze(0)
        hidden_seq, _ = self.gru(action_conditioning.float(), initial)
        batch_size, horizon, _ = hidden_seq.shape

        predicted_latent = self.latent_head(hidden_seq)
        predicted_rgb = torch.sigmoid(self.rgb_head(hidden_seq)).reshape(
            batch_size,
            horizon,
            self.config.image_channels,
            self.config.image_height,
            self.config.image_width,
        )
        predicted_depth = torch.nn.functional.softplus(self.depth_head(hidden_seq)).reshape(
            batch_size,
            horizon,
            self.config.depth_channels,
            self.config.image_height,
            self.config.image_width,
        )
        uncertainty = torch.sigmoid(self.uncertainty_head(hidden_seq))
        return GeneratedRollout(
            predicted_rgb=predicted_rgb,
            predicted_depth=predicted_depth,
            predicted_latent=predicted_latent,
            uncertainty=uncertainty,
            metadata={"model": "lightweight_gru_baseline"},
        )
