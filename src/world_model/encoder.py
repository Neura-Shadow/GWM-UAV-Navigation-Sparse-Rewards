"""Sensor encoder: maps raw observations to compact latent representations.

Provides an abstract ``SensorEncoder`` interface and two concrete
implementations:

* ``MLPEncoder`` — a small two-layer MLP that processes numeric features
  (pose + velocity) and outputs a learned latent vector.
* ``IdentityEncoder`` — a zero-cost passthrough useful for unit tests
  and baselines that operate directly on raw state vectors.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import torch
import torch.nn as nn

from src.utils.data_types import LatentState, SensorObservation

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------

class SensorEncoder(ABC):
    """Abstract interface for encoding sensor observations into latent states."""

    @abstractmethod
    def encode(self, obs: SensorObservation) -> LatentState:
        """Encode a single sensor observation into a latent state."""


# ---------------------------------------------------------------------------
# MLP-based encoder
# ---------------------------------------------------------------------------

class MLPEncoder(nn.Module, SensorEncoder):
    """Simple MLP encoder: extracts numeric features from SensorObservation.

    The encoder concatenates ``pose`` (3-dim) and ``velocity`` (3-dim) and,
    optionally, additional numeric features appended via the observation's
    metadata (key ``"extra_features"``).  The result is projected through a
    two-layer MLP to produce a dense latent vector.

    Parameters
    ----------
    input_dim:
        Dimensionality of the raw numeric input.  Default 8 corresponds to
        pose(3) + velocity(3) + dist_goal(1) + obstacle_dist(1).
    latent_dim:
        Size of the output latent vector.
    """

    def __init__(self, input_dim: int = 8, latent_dim: int = 32) -> None:
        nn.Module.__init__(self)
        self.input_dim = input_dim
        self.latent_dim = latent_dim

        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(),
            nn.Linear(64, latent_dim),
        )
        logger.info(
            "MLPEncoder created: input_dim=%d, latent_dim=%d", input_dim, latent_dim
        )

    # -- SensorEncoder interface --------------------------------------------

    def encode(self, obs: SensorObservation) -> LatentState:
        """Convert *obs* to a tensor, run through the MLP, return a LatentState."""
        raw = self._obs_to_numpy(obs)
        x = torch.tensor(raw, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            z = self.forward(x).squeeze(0).numpy()
        return LatentState(
            vector=z,
            uncertainty=0.0,
            timestamp=obs.timestamp,
        )

    # -- nn.Module interface ------------------------------------------------

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Raw forward pass: ``(B, input_dim) -> (B, latent_dim)``."""
        return self.net(x)

    def encode_batch(self, states: torch.Tensor) -> torch.Tensor:
        """Encode a batch of raw state vectors to latent vectors.

        Convenience method for training loops where observations are
        already converted to tensors.

        Parameters
        ----------
        states: ``(B, input_dim)`` raw state vectors.

        Returns
        -------
        latents: ``(B, latent_dim)``.
        """
        return self.forward(states)

    # -- helpers ------------------------------------------------------------

    def _obs_to_numpy(self, obs: SensorObservation) -> np.ndarray:
        """Extract a flat numeric vector from *obs*.

        Uses ``SensorObservation.to_state_vector()`` as the canonical
        extraction path, which includes pose, velocity, goal_distance,
        and obstacle_distance (8-dim).
        """
        vec = obs.to_state_vector()  # [px,py,pz,vx,vy,vz,goal_dist,obs_dist]

        # Pad or truncate to input_dim
        if vec.shape[0] < self.input_dim:
            vec = np.pad(vec, (0, self.input_dim - vec.shape[0]))
        elif vec.shape[0] > self.input_dim:
            vec = vec[: self.input_dim]
        return vec


# ---------------------------------------------------------------------------
# Identity (passthrough) encoder
# ---------------------------------------------------------------------------

class IdentityEncoder(SensorEncoder):
    """Dummy encoder that passes through raw numeric features as the latent state.

    Useful for testing pipelines where a learned embedding is not needed.
    """

    def encode(self, obs: SensorObservation) -> LatentState:
        """Concatenate pose + velocity into a numpy array and wrap it."""
        vector = np.array(
            list(obs.pose) + list(obs.velocity), dtype=np.float32
        )
        logger.debug("IdentityEncoder produced vector of shape %s", vector.shape)
        return LatentState(
            vector=vector,
            uncertainty=0.0,
            timestamp=obs.timestamp,
        )
