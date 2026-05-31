"""Latent world model: encoder + dynamics + decoder.

Composes a learned encoder, latent-space dynamics predictor, and decoder
into a single ``nn.Module`` that is a drop-in replacement for
``BaselineWorldModel`` in the training pipeline.

**Avoiding encoder collapse** — The training target is the raw next-state
(not ``encoder(next_state) - encoder(state)``), so the decoder forces the
latent to remain informative.  An auxiliary reconstruction loss on the
current state further regularises the autoencoder.
"""

from __future__ import annotations

import logging

import torch
import torch.nn as nn

logger = logging.getLogger(__name__)


class LatentWorldModel(nn.Module):
    """Encoder → Dynamics → Decoder world model.

    The model takes raw 8-dim state vectors and 3-dim actions, projects
    the state into a compact latent space, predicts the latent delta via
    a residual MLP, and decodes back to raw state space.

    .. code-block:: text

        state ──► encoder ──► z ──┬──► dynamics(z, a) ──► dz
                                  │                        │
                                  └────────► z + dz ───► decoder ──► pred_next
                                  │
                                  └────────► z ──────► decoder ──► recon (aux)

    The ``forward`` method returns ``pred_next - state`` (the predicted
    *state delta*), matching the ``BaselineWorldModel`` interface.

    Parameters
    ----------
    state_dim:
        Dimensionality of the raw state vector (default 8).
    action_dim:
        Dimensionality of the action vector (default 3).
    latent_dim:
        Size of the latent representation (default 32).
    hidden_dim:
        Width of hidden layers in the dynamics MLP (default 128).
    """

    def __init__(
        self,
        state_dim: int = 8,
        action_dim: int = 3,
        latent_dim: int = 32,
        hidden_dim: int = 128,
    ) -> None:
        super().__init__()
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.latent_dim = latent_dim

        # Encoder: state_dim → latent_dim
        self.encoder = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, latent_dim),
        )

        # Dynamics: (latent_dim + action_dim) → latent_dim  (predicts delta)
        self.dynamics = nn.Sequential(
            nn.Linear(latent_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, latent_dim),
        )

        # Decoder: latent_dim → state_dim
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, 64),
            nn.ReLU(),
            nn.Linear(64, state_dim),
        )

        logger.info(
            "LatentWorldModel created: state=%d, action=%d, latent=%d, hidden=%d",
            state_dim, action_dim, latent_dim, hidden_dim,
        )

    # ------------------------------------------------------------------
    # Forward passes
    # ------------------------------------------------------------------

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        """Predict the state delta ``Δs = s' - s``.

        This matches the ``BaselineWorldModel.forward`` signature so
        that ``WorldModelTrainer`` can use either model interchangeably.

        Parameters
        ----------
        state:  ``(B, state_dim)``
        action: ``(B, action_dim)``

        Returns
        -------
        delta:  ``(B, state_dim)`` — predicted ``s' - s``.
        """
        z = self.encoder(state)
        dz = self.dynamics(torch.cat([z, action], dim=-1))
        pred_next = self.decoder(z + dz)
        return pred_next - state

    def forward_with_reconstruction(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Forward pass that also returns the autoencoder reconstruction.

        Used by ``WorldModelTrainer`` to compute the auxiliary
        reconstruction loss ``MSE(recon, state)`` during training.

        Returns
        -------
        delta: ``(B, state_dim)`` — predicted state delta.
        recon: ``(B, state_dim)`` — decoder(encoder(state)).
        """
        z = self.encoder(state)
        dz = self.dynamics(torch.cat([z, action], dim=-1))
        pred_next = self.decoder(z + dz)
        recon = self.decoder(z)
        return pred_next - state, recon

    def encode(self, state: torch.Tensor) -> torch.Tensor:
        """Encode raw state to latent vector: ``(B, state_dim) → (B, latent_dim)``."""
        return self.encoder(state)
