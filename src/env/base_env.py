"""Abstract base class for navigation environments.

Every concrete environment (AirSim, mock, Isaac Sim, …) inherits from
:class:`BaseNavigationEnv` and implements the same Gym-like interface so that
planning and training code is environment-agnostic.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple

import numpy as np

from src.utils.data_types import SensorObservation


class BaseNavigationEnv(ABC):
    """Environment interface for UAV navigation tasks.

    Subclasses must implement all abstract methods and the two dimension
    properties.  The interface deliberately mirrors *OpenAI Gym* conventions
    (``reset`` / ``step`` / ``close``) so that downstream RL code can treat
    any backend identically.
    """

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @abstractmethod
    def reset(self) -> SensorObservation:
        """Reset the environment and return the initial observation."""

    @abstractmethod
    def step(
        self, action: np.ndarray
    ) -> Tuple[SensorObservation, float, bool, Dict[str, Any]]:
        """Execute *action* and return ``(obs, reward, done, info)``.

        Parameters
        ----------
        action:
            Velocity command ``[vx, vy, vz]`` in m/s.

        Returns
        -------
        obs:
            New observation after the action.
        reward:
            Scalar reward for this transition (negative cost convention).
        done:
            ``True`` when the episode has terminated (goal reached or
            collision).
        info:
            Auxiliary diagnostics dictionary.
        """

    @abstractmethod
    def get_observation(self) -> SensorObservation:
        """Return the current observation *without* advancing the simulation."""

    @abstractmethod
    def get_state_vector(self) -> np.ndarray:
        """Return the current 8-dim state vector for the world model.

        Layout: ``[px, py, pz, vx, vy, vz, goal_dist, obstacle_dist]``
        """

    @abstractmethod
    def close(self) -> None:
        """Release environment resources (connections, windows, …)."""

    # ------------------------------------------------------------------
    # Dimension properties
    # ------------------------------------------------------------------

    @property
    @abstractmethod
    def state_dim(self) -> int:
        """Dimensionality of the state vector (default: 8)."""

    @property
    @abstractmethod
    def action_dim(self) -> int:
        """Dimensionality of the action vector (default: 3)."""
