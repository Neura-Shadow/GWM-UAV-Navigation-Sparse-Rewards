"""Training losses for the lightweight generated world model."""

from __future__ import annotations

from typing import Dict, Mapping, Optional

import torch
import torch.nn.functional as F

from src.generated_world_model.types import GeneratedRollout, ObservationBatch


DEFAULT_LOSS_WEIGHTS: Dict[str, float] = {
    "rgb": 1.0,
    "depth": 0.5,
    "latent_regularization": 0.01,
    "uncertainty_regularization": 0.01,
}


def generated_world_model_loss(
    prediction: GeneratedRollout,
    target: ObservationBatch,
    weights: Optional[Mapping[str, float]] = None,
) -> tuple[torch.Tensor, Dict[str, float]]:
    """Compute a small reconstruction-style loss for synthetic Phase 4-A training."""
    active_weights = dict(DEFAULT_LOSS_WEIGHTS)
    if weights is not None:
        active_weights.update({key: float(value) for key, value in weights.items()})

    if prediction.predicted_rgb.shape != target.rgb.shape:
        raise ValueError("predicted_rgb and target rgb shapes must match.")
    if prediction.predicted_depth.shape != target.depth.shape:
        raise ValueError("predicted_depth and target depth shapes must match.")

    rgb_loss = F.mse_loss(prediction.predicted_rgb, target.rgb.float())
    depth_loss = F.mse_loss(prediction.predicted_depth, target.depth.float())
    latent_reg = prediction.predicted_latent.pow(2).mean()
    uncertainty_reg = prediction.uncertainty.mean()
    total = (
        active_weights["rgb"] * rgb_loss
        + active_weights["depth"] * depth_loss
        + active_weights["latent_regularization"] * latent_reg
        + active_weights["uncertainty_regularization"] * uncertainty_reg
    )
    metrics = {
        "loss": float(total.detach().item()),
        "rgb_loss": float(rgb_loss.detach().item()),
        "depth_loss": float(depth_loss.detach().item()),
        "latent_regularization": float(latent_reg.detach().item()),
        "uncertainty_regularization": float(uncertainty_reg.detach().item()),
    }
    return total, metrics


def future_frame_projection_loss(
    predicted_rgb: torch.Tensor,
    projected_rgb: torch.Tensor,
    valid_mask: torch.Tensor,
    weight: float = 1.0,
) -> torch.Tensor:
    """Masked RGB consistency loss against a projected geometry prior."""
    if predicted_rgb.shape != projected_rgb.shape:
        raise ValueError("predicted_rgb and projected_rgb shapes must match.")
    if valid_mask.ndim != predicted_rgb.ndim:
        while valid_mask.ndim < predicted_rgb.ndim:
            valid_mask = valid_mask.unsqueeze(1)
    if valid_mask.shape[0] != predicted_rgb.shape[0] or valid_mask.shape[-2:] != predicted_rgb.shape[-2:]:
        raise ValueError("valid_mask must share batch and spatial dimensions with predicted_rgb.")

    mask = valid_mask.to(device=predicted_rgb.device, dtype=predicted_rgb.dtype)
    if mask.shape[1] == 1 and predicted_rgb.shape[1] != 1:
        mask = mask.expand(-1, predicted_rgb.shape[1], *([-1] * (predicted_rgb.ndim - 2)))
    valid_count = mask.sum()
    if float(valid_count.detach().item()) <= 0.0:
        return predicted_rgb.sum() * 0.0

    error = (predicted_rgb - projected_rgb.to(device=predicted_rgb.device, dtype=predicted_rgb.dtype)).pow(2)
    return float(weight) * (error * mask).sum() / valid_count.clamp_min(1.0)
