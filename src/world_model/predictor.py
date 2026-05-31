"""Future-state predictor: rolls out a dynamics model over a planning horizon.

``FuturePredictor`` wraps any ``LatentDynamicsModel`` and provides two
capabilities:

1. **Trajectory rollout** — iteratively predict a sequence of future latent
   states given an action sequence.
2. **Collision probability estimation** — a lightweight heuristic that
   flags high-risk steps based on obstacle proximity encoded in the latent
   vector.
"""

from __future__ import annotations

import logging
from typing import List

import numpy as np

from src.utils.data_types import LatentState
from src.world_model.latent_dynamics import LatentDynamicsModel

logger = logging.getLogger(__name__)


class FuturePredictor:
    """Rolls out a dynamics model for multiple steps to predict a future trajectory.

    Parameters
    ----------
    dynamics:
        A ``LatentDynamicsModel`` used for one-step predictions.
    horizon:
        Default planning horizon (number of rollout steps).
    """

    def __init__(self, dynamics: LatentDynamicsModel, horizon: int = 12) -> None:
        self.dynamics = dynamics
        self.horizon = horizon
        logger.info("FuturePredictor created with horizon=%d", horizon)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict_trajectory(
        self,
        initial_state: LatentState,
        action_sequence: np.ndarray,
    ) -> List[LatentState]:
        """Predict a sequence of future latent states.

        Parameters
        ----------
        initial_state:
            The starting latent state.
        action_sequence:
            Array of shape ``(T, action_dim)`` where ``T`` is the number
            of rollout steps.  If ``T`` exceeds :pyattr:`horizon`, only
            the first :pyattr:`horizon` actions are used.

        Returns
        -------
        List[LatentState]
            One predicted state per action step (length = ``min(T, horizon)``).
        """
        steps = min(len(action_sequence), self.horizon)
        trajectory: List[LatentState] = []
        current = initial_state

        for t in range(steps):
            action = np.asarray(action_sequence[t], dtype=np.float32)
            current = self.dynamics.predict(current, action)
            trajectory.append(current)

        logger.debug("Predicted trajectory of length %d", len(trajectory))
        return trajectory

    def predict_collision_probability(
        self,
        trajectory: List[LatentState],
        obstacle_threshold: float = 4.0,
    ) -> List[float]:
        """Estimate collision probability at each trajectory step.

        This is a simple heuristic: if the latent vector has at least 8
        dimensions, the 8th element (index 7) is treated as an obstacle
        distance (matching the state layout used in the legacy code).
        When the obstacle distance is below *obstacle_threshold* the
        probability ramps linearly from 0 to 1.

        Parameters
        ----------
        trajectory:
            List of predicted latent states.
        obstacle_threshold:
            Distance below which collision risk starts rising.

        Returns
        -------
        List[float]
            Collision probabilities in [0, 1], one per trajectory step.
        """
        probabilities: List[float] = []

        for state in trajectory:
            vec = np.asarray(state.vector, dtype=np.float32).flatten()
            if vec.shape[0] >= 8:
                obstacle_dist = float(vec[7])
                if obstacle_dist <= 0.0:
                    prob = 1.0
                elif obstacle_dist >= obstacle_threshold:
                    prob = 0.0
                else:
                    prob = 1.0 - (obstacle_dist / obstacle_threshold)
            else:
                # No obstacle info encoded — assume safe
                prob = 0.0
            probabilities.append(prob)

        return probabilities
