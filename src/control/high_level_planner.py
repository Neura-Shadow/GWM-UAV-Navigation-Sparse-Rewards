"""High-level planner: world-model-guided deliberative planning.

``HighLevelPlanner`` composes the encoder, future predictor, uncertainty
estimator, and policy intent mapper into a single ``plan()`` call that
transforms a raw sensor observation into a ``PolicyIntent``.

This is the *cortex* — slow, deliberative, risk-aware.
"""

from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from src.utils.data_types import PolicyIntent, SensorObservation
from src.world_model.encoder import SensorEncoder
from src.world_model.policy_intent import PolicyIntentMapper
from src.world_model.predictor import FuturePredictor
from src.world_model.uncertainty import UncertaintyEstimator

logger = logging.getLogger(__name__)


class HighLevelPlanner:
    """World-model-guided high-level planner.

    Slow, deliberative planning using world-model rollouts.  Evaluates
    multiple action candidates, scores their predicted trajectories,
    and outputs the best ``PolicyIntent`` for the low-level controller.

    Parameters
    ----------
    predictor:
        A ``FuturePredictor`` that rolls out a dynamics model.
    intent_mapper:
        Converts trajectory predictions to ``PolicyIntent``.
    uncertainty_estimator:
        Estimates model confidence for the current latent state.
    """

    def __init__(
        self,
        predictor: FuturePredictor,
        intent_mapper: PolicyIntentMapper,
        uncertainty_estimator: UncertaintyEstimator,
    ) -> None:
        self.predictor = predictor
        self.intent_mapper = intent_mapper
        self.uncertainty_estimator = uncertainty_estimator
        logger.info("HighLevelPlanner created")

    def plan(
        self,
        observation: SensorObservation,
        encoder: SensorEncoder,
        action_candidates: np.ndarray,
    ) -> PolicyIntent:
        """Generate a high-level policy intent from the current observation.

        Steps
        -----
        1. Encode the observation into latent space.
        2. For each action candidate sequence, predict the trajectory.
        3. Estimate collision probabilities for each trajectory.
        4. Select the trajectory with the lowest aggregate risk.
        5. Map the best trajectory to a ``PolicyIntent``.

        Parameters
        ----------
        observation:
            Current sensor snapshot.
        encoder:
            A ``SensorEncoder`` to map the observation to latent space.
        action_candidates:
            Array of shape ``(N, horizon, action_dim)`` containing *N*
            candidate action sequences.  If shape is ``(horizon, action_dim)``
            a single candidate is assumed.

        Returns
        -------
        PolicyIntent
        """
        # 1. Encode
        latent = encoder.encode(observation)
        latent.uncertainty = self.uncertainty_estimator.estimate(latent)

        # Ensure action_candidates is 3-D
        candidates = np.asarray(action_candidates, dtype=np.float32)
        if candidates.ndim == 2:
            candidates = candidates[np.newaxis, ...]  # (1, H, A)

        # 2-3. Evaluate each candidate
        best_cost = float("inf")
        best_trajectory = None
        best_collision_probs = None

        for i in range(candidates.shape[0]):
            traj = self.predictor.predict_trajectory(latent, candidates[i])
            c_probs = self.predictor.predict_collision_probability(traj)
            cost = self._trajectory_cost(traj, c_probs)

            if cost < best_cost:
                best_cost = cost
                best_trajectory = traj
                best_collision_probs = c_probs

        # Fallback: empty candidates
        if best_trajectory is None:
            logger.warning("No valid trajectory found; returning default intent")
            best_trajectory = []
            best_collision_probs = []

        # 5. Map to PolicyIntent
        intent = self.intent_mapper.map_to_intent(
            latent, best_trajectory, best_collision_probs
        )
        logger.debug(
            "HighLevelPlanner selected trajectory with cost=%.3f", best_cost
        )
        return intent

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _trajectory_cost(
        trajectory: list,
        collision_probs: list,
    ) -> float:
        """Simple aggregate cost: sum of collision probabilities.

        Can be extended with goal-distance, smoothness, and energy terms.
        """
        if not collision_probs:
            return 0.0
        return float(np.sum(collision_probs))
