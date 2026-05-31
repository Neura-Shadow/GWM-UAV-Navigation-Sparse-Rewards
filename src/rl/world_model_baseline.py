"""Baseline world model – the exact original 3-layer MLP architecture.

This module preserves the original ``WorldModel(nn.Module)`` class verbatim so
that regression tests and comparisons against the monolithic implementation
remain valid.  More sophisticated world models can subclass or replace this.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class BaselineWorldModel(nn.Module):
    """3-layer MLP world model predicting state *deltas*.

    Given the current state ``s`` and action ``a`` the model predicts
    ``Δs = s' - s``, so the next state is obtained as ``s' = s + model(s, a)``.

    Architecture (matching the original)::

        Linear(state_dim + action_dim, 128) → ReLU
        Linear(128, 128)                    → ReLU
        Linear(128, state_dim)

    Parameters
    ----------
    state_dim:
        Dimensionality of the state vector (default 8).
    action_dim:
        Dimensionality of the action vector (default 3).
    """

    def __init__(self, state_dim: int = 8, action_dim: int = 3) -> None:
        super().__init__()
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, state_dim),
        )

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        """Predict the state delta ``Δs`` given ``(s, a)``.

        Parameters
        ----------
        state:  ``(*, state_dim)``
        action: ``(*, action_dim)``

        Returns
        -------
        delta:  ``(*, state_dim)`` – predicted ``s' - s``.
        """
        return self.net(torch.cat([state, action], dim=-1))
