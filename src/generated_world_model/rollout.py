"""Autoregressive rollout utilities for generated world models."""

from __future__ import annotations

from typing import Any, Optional

import torch

from src.generated_world_model.action_conditioner import ActionConditioner
from src.generated_world_model.observation_encoder import ObservationEncoder
from src.generated_world_model.types import ActionSequence, GeneratedRollout, ObservationBatch
from src.generated_world_model.video_dynamics_model import VideoDynamicsModel


_SUPPORTED_MODES = {"teacher_forcing", "autoregressive", "scheduled_sampling"}


class AutoregressiveRollout:
    """Roll out a generated world model over a sequence of UAV actions."""

    def __init__(
        self,
        model: VideoDynamicsModel,
        encoder: ObservationEncoder,
        conditioner: ActionConditioner,
        mode: str = "autoregressive",
    ) -> None:
        if mode not in _SUPPORTED_MODES:
            raise ValueError(f"Unsupported rollout mode: {mode}")
        self.model = model
        self.encoder = encoder
        self.conditioner = conditioner
        self.mode = mode

    def rollout(
        self,
        initial_context: ObservationBatch | torch.Tensor,
        action_sequence: ActionSequence | torch.Tensor,
        horizon: Optional[int] = None,
    ) -> GeneratedRollout:
        """Generate a future observation rollout.

        Phase 4-A implements the same minimal autoregressive loop for all
        accepted mode names. Teacher-forcing and scheduled-sampling-specific
        behavior is reserved for later training slices.
        """
        context_latent = (
            initial_context
            if isinstance(initial_context, torch.Tensor)
            else self.encoder.encode(initial_context)
        )
        if isinstance(action_sequence, ActionSequence):
            actions = action_sequence.actions
            poses = action_sequence.poses
            intentions = action_sequence.intentions
        else:
            actions = action_sequence
            poses = None
            intentions = None

        if actions.ndim != 3:
            raise ValueError("action_sequence must have shape [B, H, action_dim].")
        target_horizon = int(horizon if horizon is not None else actions.shape[1])
        if target_horizon <= 0:
            raise ValueError("horizon must be positive.")
        if target_horizon > actions.shape[1]:
            raise ValueError("horizon cannot exceed action_sequence length.")

        rgb_steps = []
        depth_steps = []
        latent_steps = []
        uncertainty_steps = []

        for step in range(target_horizon):
            step_actions = actions[:, step:step + 1, :]
            step_poses = None if poses is None else poses[:, step:step + 1, :]
            step_intentions = None if intentions is None else intentions[:, step:step + 1, :]
            conditioning = self.conditioner.encode(
                step_actions,
                poses=step_poses,
                intentions=step_intentions,
            )
            prediction = self.model(context_latent, conditioning)
            rgb_steps.append(prediction.predicted_rgb)
            depth_steps.append(prediction.predicted_depth)
            latent_steps.append(prediction.predicted_latent)
            uncertainty_steps.append(prediction.uncertainty)

            next_latent = prediction.predicted_latent[:, -1:, :]
            if context_latent.shape[1] > 1:
                context_latent = torch.cat([context_latent[:, 1:, :], next_latent], dim=1)
            else:
                context_latent = next_latent

        return GeneratedRollout(
            predicted_rgb=torch.cat(rgb_steps, dim=1),
            predicted_depth=torch.cat(depth_steps, dim=1),
            predicted_latent=torch.cat(latent_steps, dim=1),
            uncertainty=torch.cat(uncertainty_steps, dim=1),
            metadata={"mode": self.mode},
        )
