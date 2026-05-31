"""Policy intent mapper: converts world-model predictions to high-level intents.

``PolicyIntentMapper`` bridges the latent-space trajectory forecasts produced
by the world model with the ``PolicyIntent`` consumed by the control layer.
It computes risk-aware velocity targets and confidence scores.
"""

from __future__ import annotations

import logging
from typing import List, Tuple

import numpy as np

from src.utils.data_types import LatentState, PolicyIntent

logger = logging.getLogger(__name__)


class PolicyIntentMapper:
    """Converts world model predictions into high-level policy intents.

    Parameters
    ----------
    goal:
        Target waypoint ``(x, y, z)`` in world frame.
    max_velocity:
        Maximum desired speed (m/s).  Reduced proportionally to risk.
    """

    def __init__(
        self,
        goal: Tuple[float, float, float],
        max_velocity: float = 4.0,
    ) -> None:
        self.goal = goal
        self.max_velocity = max_velocity
        logger.info(
            "PolicyIntentMapper created: goal=%s, max_vel=%.1f",
            goal,
            max_velocity,
        )

    def map_to_intent(
        self,
        current_latent: LatentState,
        predicted_trajectory: List[LatentState],
        collision_probs: List[float],
    ) -> PolicyIntent:
        """Map predictions to a ``PolicyIntent``.

        * ``target_position`` is set to the goal.
        * ``risk_score`` equals the maximum collision probability across
          the trajectory.
        * ``desired_velocity`` is throttled down linearly with risk.
        * ``confidence`` is ``1 - current_latent.uncertainty``.

        Parameters
        ----------
        current_latent:
            The encoder output for the current observation.
        predicted_trajectory:
            Sequence of predicted future latent states.
        collision_probs:
            Per-step collision probabilities from :class:`FuturePredictor`.

        Returns
        -------
        PolicyIntent
        """
        risk_score = max(collision_probs) if collision_probs else 0.0
        desired_velocity = self.max_velocity * (1.0 - risk_score)
        confidence = 1.0 - min(1.0, max(0.0, current_latent.uncertainty))
        horizon = len(predicted_trajectory)

        intent = PolicyIntent(
            target_position=self.goal,
            desired_velocity=desired_velocity,
            risk_score=risk_score,
            horizon=horizon,
            confidence=confidence,
        )
        logger.debug(
            "PolicyIntent: risk=%.3f vel=%.2f conf=%.3f horizon=%d",
            risk_score,
            desired_velocity,
            confidence,
            horizon,
        )
        return intent
