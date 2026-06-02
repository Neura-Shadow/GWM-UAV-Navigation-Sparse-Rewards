"""Training helpers for the Phase 4-A generated world model baseline."""

from __future__ import annotations

from typing import Any, Dict, Tuple

import torch

from src.generated_world_model.action_conditioner import ActionConditioner
from src.generated_world_model.dataset import create_synthetic_batch
from src.generated_world_model.losses import generated_world_model_loss
from src.generated_world_model.observation_encoder import ObservationEncoder
from src.generated_world_model.types import GWMConfig
from src.generated_world_model.video_dynamics_model import VideoDynamicsModel


def build_baseline_components(
    config: GWMConfig | Dict[str, Any] | None = None,
) -> Tuple[ObservationEncoder, ActionConditioner, VideoDynamicsModel]:
    """Create encoder, conditioner, and dynamics modules for Phase 4-A."""
    active_config = GWMConfig.from_any(config)
    encoder = ObservationEncoder(active_config)
    conditioner = ActionConditioner(active_config)
    model = VideoDynamicsModel(active_config)
    return encoder, conditioner, model


def train_synthetic_step(
    encoder: ObservationEncoder,
    conditioner: ActionConditioner,
    model: VideoDynamicsModel,
    optimizer: torch.optim.Optimizer,
    batch: Dict[str, Any],
    device: torch.device | str = "cpu",
) -> Dict[str, float]:
    """Run one synthetic training step and return loss metrics."""
    encoder.train()
    conditioner.train()
    model.train()

    context = batch["context"].to(device)
    future = batch["future"].to(device)
    action_sequence = batch["action_sequence"].to(device)

    optimizer.zero_grad(set_to_none=True)
    context_latent = encoder.encode(context)
    conditioning = conditioner.encode(action_sequence)
    prediction = model(context_latent, conditioning)
    loss, metrics = generated_world_model_loss(prediction, future)
    loss.backward()
    torch.nn.utils.clip_grad_norm_(
        list(encoder.parameters()) + list(conditioner.parameters()) + list(model.parameters()),
        max_norm=5.0,
    )
    optimizer.step()
    return metrics


def make_synthetic_training_batch(
    config: GWMConfig,
    batch_size: int,
    seed: int | None = None,
) -> Dict[str, Any]:
    """Create a synthetic batch using the active config dimensions."""
    return create_synthetic_batch(
        batch_size=batch_size,
        context_length=config.context_length,
        horizon=config.horizon,
        image_height=config.image_height,
        image_width=config.image_width,
        pose_dim=config.pose_dim,
        action_dim=config.action_dim,
        intention_dim=config.intention_dim,
        seed=seed,
    )
