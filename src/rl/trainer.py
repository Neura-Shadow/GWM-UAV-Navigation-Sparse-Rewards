"""World-model trainer – MSE on state deltas with gradient clipping.

Direct port of ``AirSimNeuroPlanner.train_world_model`` into a standalone,
reusable class.  Extended with checkpoint save/load and support for
``LatentWorldModel`` reconstruction loss.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from src.rl.replay_buffer import ReplayBuffer

logger = logging.getLogger(__name__)


@dataclass
class TrainerConfig:
    """Hyper-parameters for world-model training.

    Default values match the original monolithic implementation.
    """

    batch_size: int = 128
    train_epochs: int = 2
    grad_clip: float = 5.0
    min_buffer_size: int = 40  # train_after_steps in original
    reconstruction_weight: float = 0.1  # auxiliary recon loss weight


class WorldModelTrainer:
    """Trains a world model on replay buffer transitions.

    The training objective is MSE on **state deltas**:

    .. math::

        L = \\| \\text{model}(s, a) - (s' - s) \\|^2

    matching the original ``train_world_model`` method.

    If the model exposes a ``forward_with_reconstruction`` method (e.g.
    ``LatentWorldModel``), an auxiliary reconstruction loss is added:

    .. math::

        L_{total} = L_{dynamics} + \\alpha \\cdot L_{recon}

    where ``L_{recon} = MSE(decoder(encoder(s)), s)`` and ``α`` is
    controlled by ``TrainerConfig.reconstruction_weight``.

    Parameters
    ----------
    model:
        The world model ``nn.Module`` mapping ``(s, a) → Δs``.
    optimizer:
        A ``torch.optim.Optimizer`` already wrapping *model* parameters.
    replay_buffer:
        Shared replay buffer from which batches are sampled.
    device:
        Torch device for training.
    config:
        Training hyper-parameters.
    """

    def __init__(
        self,
        model: nn.Module,
        optimizer: optim.Optimizer,
        replay_buffer: ReplayBuffer,
        device: torch.device,
        config: Optional[TrainerConfig] = None,
    ) -> None:
        self.model = model
        self.optimizer = optimizer
        self.replay_buffer = replay_buffer
        self.device = device
        self.config = config or TrainerConfig()

        self._total_updates = 0
        self._has_reconstruction = hasattr(model, "forward_with_reconstruction")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def train_step(self) -> Optional[float]:
        """Run one training step (potentially multiple gradient epochs).

        Returns
        -------
        loss:
            Average loss across all gradient steps in this call, or ``None``
            if the buffer does not contain enough transitions yet.
        """
        min_required = max(self.config.batch_size, self.config.min_buffer_size)
        if len(self.replay_buffer) < min_required:
            return None

        total_loss = 0.0

        for _ in range(self.config.train_epochs):
            states_np, actions_np, next_states_np = self.replay_buffer.sample(
                self.config.batch_size
            )

            states = torch.tensor(states_np, dtype=torch.float32, device=self.device)
            actions = torch.tensor(actions_np, dtype=torch.float32, device=self.device)
            next_states = torch.tensor(
                next_states_np, dtype=torch.float32, device=self.device
            )

            # Target is the state delta (next_state - state)
            targets = next_states - states

            if self._has_reconstruction:
                preds, recon = self.model.forward_with_reconstruction(states, actions)
                dynamics_loss = nn.functional.mse_loss(preds, targets)
                recon_loss = nn.functional.mse_loss(recon, states)
                loss = dynamics_loss + self.config.reconstruction_weight * recon_loss
            else:
                preds = self.model(states, actions)
                loss = nn.functional.mse_loss(preds, targets)

            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.model.parameters(), self.config.grad_clip)
            self.optimizer.step()

            total_loss += loss.item()
            self._total_updates += 1

        avg_loss = total_loss / self.config.train_epochs
        logger.debug(
            "World-model training step: avg_loss=%.6f  total_updates=%d",
            avg_loss,
            self._total_updates,
        )
        return avg_loss

    # ------------------------------------------------------------------
    # Checkpoint management
    # ------------------------------------------------------------------

    def save_checkpoint(
        self,
        path: str,
        model_type: Optional[str] = None,
        config: Optional[dict] = None,
        final_loss: Optional[float] = None,
    ) -> None:
        """Save model weights, optimizer state, and training progress.

        Parameters
        ----------
        path:
            File path for the ``.pt`` checkpoint.
        model_type:
            Optional string describing the type of model (e.g. 'baseline', 'latent').
        config:
            Optional dictionary of configuration parameters/metadata.
        final_loss:
            Optional final validation/training loss value.
        """
        save_dir = Path(path).parent
        save_dir.mkdir(parents=True, exist_ok=True)

        checkpoint = {
            "model_state_dict": self.model.state_dict(),
            "optimizer_state_dict": self.optimizer.state_dict(),
            "total_updates": self._total_updates,
            "model_type": model_type,
            "config": config,
            "final_loss": final_loss,
        }
        torch.save(checkpoint, path)
        logger.info("Checkpoint saved to %s (updates=%d)", path, self._total_updates)

    def load_checkpoint(self, path: str) -> None:
        """Load model weights, optimizer state, and training progress.

        Parameters
        ----------
        path:
            File path of the ``.pt`` checkpoint to load.
        """
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        self.model.load_state_dict(checkpoint["model_state_dict"])
        self.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        self._total_updates = checkpoint.get("total_updates", 0)
        logger.info(
            "Checkpoint loaded from %s (updates=%d)", path, self._total_updates
        )

    @property
    def total_updates(self) -> int:
        """Total number of gradient updates performed so far."""
        return self._total_updates
