"""Latent-space dynamics models: predict the next latent state given an action.

Provides an abstract ``LatentDynamicsModel`` interface and two concrete
implementations:

* ``MLPDynamics`` — a three-layer MLP that predicts *state deltas* in latent
  space.  This is a generalised, modular version of the ``WorldModel`` in
  ``main.py``.
* ``LinearDynamics`` — a trivial integrator (``next = current + action * dt``)
  useful for sanity-checking the planning pipeline.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod

import numpy as np
import torch
import torch.nn as nn

from src.utils.data_types import LatentState

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------

class LatentDynamicsModel(ABC):
    """Abstract interface for one-step latent dynamics prediction."""

    @abstractmethod
    def predict(self, latent: LatentState, action: np.ndarray) -> LatentState:
        """Predict the next latent state given the current state and action."""


# ---------------------------------------------------------------------------
# MLP dynamics (learned)
# ---------------------------------------------------------------------------

class MLPDynamics(nn.Module, LatentDynamicsModel):
    """MLP-based dynamics model predicting *state deltas* in latent space.

    Architecture: ``(latent_dim + action_dim) -> hidden -> hidden -> latent_dim``

    The network learns the *residual* ``delta = s_{t+1} - s_t``, so the
    predicted next state is ``s_t + delta``.  This mirrors the training
    objective in the original monolithic ``WorldModel``.

    Parameters
    ----------
    latent_dim:
        Dimensionality of the latent state vector.
    action_dim:
        Dimensionality of the action vector.
    hidden_dim:
        Width of the hidden layers.
    """

    def __init__(
        self,
        latent_dim: int = 32,
        action_dim: int = 3,
        hidden_dim: int = 128,
    ) -> None:
        nn.Module.__init__(self)
        self.latent_dim = latent_dim
        self.action_dim = action_dim

        self.net = nn.Sequential(
            nn.Linear(latent_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, latent_dim),
        )
        logger.info(
            "MLPDynamics created: latent=%d, action=%d, hidden=%d",
            latent_dim,
            action_dim,
            hidden_dim,
        )

    # -- LatentDynamicsModel interface --------------------------------------

    def predict(self, latent: LatentState, action: np.ndarray) -> LatentState:
        """Predict next latent state by computing the learned residual."""
        state_np = np.asarray(latent.vector, dtype=np.float32).flatten()
        action_np = np.asarray(action, dtype=np.float32).flatten()

        state_t = torch.tensor(state_np, dtype=torch.float32).unsqueeze(0)
        action_t = torch.tensor(action_np, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            delta = self.forward(state_t, action_t).squeeze(0).numpy()

        next_vector = state_np + delta
        return LatentState(
            vector=next_vector,
            uncertainty=latent.uncertainty,
            timestamp=latent.timestamp,
            metadata=dict(latent.metadata),
        )

    # -- nn.Module interface ------------------------------------------------

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        """Predict the state *delta* ``(B, latent_dim)``."""
        x = torch.cat([state, action], dim=-1)
        return self.net(x)


# ---------------------------------------------------------------------------
# Linear dynamics (test / baseline)
# ---------------------------------------------------------------------------

class LinearDynamics(LatentDynamicsModel):
    """Trivial linear integrator: ``next = current + action * dt``.

    Useful for verifying the planning loop without a learned model.

    Parameters
    ----------
    dt:
        Integration timestep in seconds.
    """

    def __init__(self, dt: float = 0.4) -> None:
        self.dt = dt
        logger.info("LinearDynamics created with dt=%.3f", dt)

    def predict(self, latent: LatentState, action: np.ndarray) -> LatentState:
        """Euler-integrate the action into the latent vector."""
        current = np.asarray(latent.vector, dtype=np.float32).flatten()
        act = np.asarray(action, dtype=np.float32).flatten()

        # Pad/truncate action to match state dim
        if act.shape[0] < current.shape[0]:
            act = np.pad(act, (0, current.shape[0] - act.shape[0]))
        elif act.shape[0] > current.shape[0]:
            act = act[: current.shape[0]]

        next_vector = current + act * self.dt
        return LatentState(
            vector=next_vector,
            uncertainty=latent.uncertainty,
            timestamp=latent.timestamp + self.dt,
            metadata=dict(latent.metadata),
        )
