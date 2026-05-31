"""Random-shooting MPC planner for world-model-guided navigation.

Exact port of the original planning logic from ``AirSimNeuroPlanner``:
random-shooting with configurable horizon, samples, and cost function.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import torch
import torch.nn as nn

logger = logging.getLogger(__name__)


@dataclass
class CostWeights:
    """Scalar weights for each term of the trajectory cost function.

    Default values match the original monolithic implementation.
    """

    obstacle: float = 15.0
    smooth: float = 0.15
    energy: float = 0.02
    sparse_bonus: float = -120.0


class MPCPlanner:
    """Random-shooting model-predictive controller.

    At each planning step the planner:

    1. Samples ``num_samples`` random action sequences of length ``horizon``.
    2. Rolls each sequence out through the learned world model.
    3. Evaluates a hand-designed cost function on the predicted trajectory.
    4. Returns the first action of the lowest-cost sequence.

    Parameters
    ----------
    state_dim:
        Dimension of the state vector (8).
    action_dim:
        Dimension of the action vector (3: vx, vy, vz).
    horizon:
        Planning horizon (number of future steps to simulate).
    num_samples:
        Number of random action sequences to evaluate.
    target_speed:
        Maximum absolute velocity for vx and vy [m/s].
    cost_weights:
        Weights for each cost term (see :class:`CostWeights`).
    goal:
        3D goal position as a numpy array.
    min_obstacle_dist:
        Safety threshold [m].
    goal_reach_dist:
        Distance at which the sparse goal bonus is awarded.
    """

    def __init__(
        self,
        state_dim: int = 8,
        action_dim: int = 3,
        horizon: int = 12,
        num_samples: int = 120,
        target_speed: float = 4.0,
        cost_weights: Optional[CostWeights] = None,
        goal: Optional[np.ndarray] = None,
        min_obstacle_dist: float = 4.0,
        goal_reach_dist: float = 3.0,
    ) -> None:
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.horizon = horizon
        self.num_samples = num_samples
        self.target_speed = target_speed
        self.weights = cost_weights or CostWeights()
        self.goal = (
            goal.copy()
            if goal is not None
            else np.array([60.0, 20.0, -8.0], dtype=np.float32)
        )
        self.min_obstacle_dist = min_obstacle_dist
        self.goal_reach_dist = goal_reach_dist

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan_action(
        self,
        state: np.ndarray,
        model: nn.Module,
        device: torch.device,
        last_action: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Select the best first action via random shooting.

        Parameters
        ----------
        state:
            Current 8-dim state vector.
        model:
            World model ``(s, a) → Δs``.
        device:
            Torch device for model inference.
        last_action:
            Previous action for smoothness cost (defaults to zeros).

        Returns
        -------
        action:
            Best first action ``[vx, vy, vz]``.
        """
        if last_action is None:
            last_action = np.zeros(self.action_dim, dtype=np.float32)

        best_cost = float("inf")
        best_seq: Optional[np.ndarray] = None

        for _ in range(self.num_samples):
            seq = np.array(
                [self._sample_action() for _ in range(self.horizon)],
                dtype=np.float32,
            )
            cost = self._rollout_cost(state, seq, model, device, last_action)
            if cost < best_cost:
                best_cost = cost
                best_seq = seq

        if best_seq is None:
            logger.warning("No valid sequence found – returning random action.")
            return self._sample_action()

        return best_seq[0].copy()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _sample_action(self) -> np.ndarray:
        """Sample a random action within the allowed velocity bounds."""
        vx = np.random.uniform(-self.target_speed, self.target_speed)
        vy = np.random.uniform(-self.target_speed, self.target_speed)
        vz = np.random.uniform(-1.0, 1.0)
        return np.array([vx, vy, vz], dtype=np.float32)

    def _rollout_cost(
        self,
        state: np.ndarray,
        action_seq: np.ndarray,
        model: nn.Module,
        device: torch.device,
        last_action: np.ndarray,
    ) -> float:
        """Compute the cumulative cost of rolling out *action_seq* from *state*.

        The cost function reproduces the original implementation exactly:

        * **goal_cost**: Euclidean distance to the goal.
        * **obstacle_cost**: ``15.0 * max(0, min_obstacle_dist - obs_dist)``.
        * **smooth_cost**: ``0.15 * ||a_t - a_{t-1}||``.
        * **energy_cost**: ``0.02 * ||a_t||^2``.
        * **sparse_bonus**: ``-120`` when within ``goal_reach_dist`` of the goal.
        """
        total_cost = 0.0
        st = torch.tensor(
            state, dtype=torch.float32, device=device
        ).unsqueeze(0)

        for t in range(self.horizon):
            act = torch.tensor(
                action_seq[t], dtype=torch.float32, device=device
            ).unsqueeze(0)

            with torch.no_grad():
                delta = model(st, act)
            st = st + delta

            pred = st.squeeze(0).cpu().numpy()
            px, py, pz = pred[0], pred[1], pred[2]
            dist_goal = float(
                np.linalg.norm(
                    np.array([px, py, pz], dtype=np.float32) - self.goal
                )
            )
            obstacle_dist = float(np.clip(pred[7], 0.2, 50.0))

            goal_cost = dist_goal
            obstacle_cost = self.weights.obstacle * max(
                0.0, self.min_obstacle_dist - obstacle_dist
            )
            prev = last_action if t == 0 else action_seq[t - 1]
            smooth_cost = self.weights.smooth * float(
                np.linalg.norm(action_seq[t] - prev)
            )
            energy_cost = self.weights.energy * float(
                np.linalg.norm(action_seq[t]) ** 2
            )
            sparse_bonus = (
                self.weights.sparse_bonus
                if dist_goal < self.goal_reach_dist
                else 0.0
            )

            total_cost += (
                goal_cost + obstacle_cost + smooth_cost + energy_cost + sparse_bonus
            )

        return total_cost
