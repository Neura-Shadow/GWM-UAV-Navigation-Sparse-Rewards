"""Lightweight mock navigation environment for testing and CI.

Implements simple point-mass dynamics with configurable spherical obstacles so
that the full planning and training pipeline can be exercised without AirSim,
ROS 2, or any GPU.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np

from src.env.base_env import BaseNavigationEnv
from src.utils.data_types import SensorObservation

logger = logging.getLogger(__name__)


@dataclass
class Obstacle:
    """Spherical obstacle defined by centre and radius."""

    position: np.ndarray
    radius: float


class MockNavigationEnv(BaseNavigationEnv):
    """Point-mass UAV simulator for unit tests and rapid prototyping.

    Parameters
    ----------
    goal:
        3D goal position ``(x, y, z)`` in world frame.
    start_position:
        Initial UAV position.
    obstacles:
        List of ``(centre, radius)`` tuples describing spherical obstacles.
    dt:
        Simulation timestep [s].
    min_obstacle_dist:
        Safety threshold used for the ``done`` check on collision.
    goal_reach_dist:
        Episode ends successfully when the UAV is this close to the goal.
    max_steps:
        Maximum episode length before forced termination.
    """

    def __init__(
        self,
        goal: Tuple[float, float, float] = (60.0, 20.0, -8.0),
        start_position: Tuple[float, float, float] = (0.0, 0.0, -5.0),
        obstacles: List[Tuple[Tuple[float, float, float], float]] | None = None,
        dt: float = 0.4,
        min_obstacle_dist: float = 4.0,
        goal_reach_dist: float = 3.0,
        max_steps: int = 600,
    ) -> None:
        self._goal = np.array(goal, dtype=np.float32)
        self._start_position = np.array(start_position, dtype=np.float32)
        self._dt = dt
        self._min_obstacle_dist = min_obstacle_dist
        self._goal_reach_dist = goal_reach_dist
        self._max_steps = max_steps

        # Build obstacle list
        self._obstacles: List[Obstacle] = []
        if obstacles is not None:
            for centre, radius in obstacles:
                self._obstacles.append(
                    Obstacle(
                        position=np.array(centre, dtype=np.float32),
                        radius=radius,
                    )
                )
        else:
            # Default obstacle layout for a non-trivial test
            self._obstacles = [
                Obstacle(position=np.array([20.0, 5.0, -5.0], dtype=np.float32), radius=3.0),
                Obstacle(position=np.array([40.0, 15.0, -7.0], dtype=np.float32), radius=4.0),
            ]

        # Mutable state – initialised by reset()
        self._position = self._start_position.copy()
        self._velocity = np.zeros(3, dtype=np.float32)
        self._step_count = 0

    # ------------------------------------------------------------------
    # BaseNavigationEnv interface
    # ------------------------------------------------------------------

    def reset(self) -> SensorObservation:
        self._position = self._start_position.copy()
        self._velocity = np.zeros(3, dtype=np.float32)
        self._step_count = 0
        logger.debug("MockNavigationEnv reset. Start=%s", self._position)
        return self.get_observation()

    def step(
        self, action: np.ndarray
    ) -> Tuple[SensorObservation, float, bool, Dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)

        # Simple point-mass dynamics: position += velocity * dt
        self._velocity = action.copy()
        self._position = self._position + self._velocity * self._dt
        self._step_count += 1

        obs = self.get_observation()

        # Terminal conditions
        goal_reached = obs.goal_distance < self._goal_reach_dist
        collision = obs.obstacle_distance < 0.5  # hard collision
        timeout = self._step_count >= self._max_steps
        done = goal_reached or collision or timeout

        # Reward (negative cost)
        reward = -obs.goal_distance
        if goal_reached:
            reward += 120.0
        if collision:
            reward -= 50.0

        info: Dict[str, Any] = {
            "step": self._step_count,
            "goal_distance": obs.goal_distance,
            "obstacle_distance": obs.obstacle_distance,
            "goal_reached": goal_reached,
            "collision": collision,
            "timeout": timeout,
        }
        return obs, reward, done, info

    def get_observation(self) -> SensorObservation:
        goal_dist = float(np.linalg.norm(self._position - self._goal))
        obs_dist = self._compute_obstacle_distance()

        return SensorObservation(
            timestamp=float(self._step_count * self._dt),
            pose=tuple(self._position.tolist()),  # type: ignore[arg-type]
            velocity=tuple(self._velocity.tolist()),  # type: ignore[arg-type]
            goal_distance=goal_dist,
            obstacle_distance=obs_dist,
        )

    def get_state_vector(self) -> np.ndarray:
        return self.get_observation().to_state_vector()

    def close(self) -> None:
        logger.debug("MockNavigationEnv closed.")

    # ------------------------------------------------------------------
    # Curriculum support
    # ------------------------------------------------------------------

    def update_difficulty(
        self,
        goal_distance: float,
        num_obstacles: int,
        max_steps: int,
        seed: int | None = None,
    ) -> SensorObservation:
        """Reconfigure the environment for a new difficulty level.

        Places the goal at ``goal_distance`` from the start position in
        the original goal direction, generates ``num_obstacles`` random
        obstacles between start and goal, and updates the episode length.

        Parameters
        ----------
        goal_distance:
            Euclidean distance from start to goal [m].
        num_obstacles:
            Number of spherical obstacles to place.
        max_steps:
            Maximum episode length.
        seed:
            Optional RNG seed for reproducible obstacle placement.

        Returns
        -------
        obs:
            Initial observation after reset.
        """
        rng = np.random.default_rng(seed)

        # Compute goal direction (unit vector from start toward original goal)
        direction = self._goal - self._start_position
        norm = float(np.linalg.norm(direction))
        if norm > 1e-6:
            direction = direction / norm
        else:
            direction = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        # Set new goal
        self._goal = self._start_position + direction * goal_distance

        # Generate obstacles between start and goal
        self._obstacles = []
        for _ in range(num_obstacles):
            # Random point along the start-goal corridor with lateral offset
            t = rng.uniform(0.2, 0.8)
            centre = self._start_position + direction * (goal_distance * t)
            lateral = rng.normal(0.0, 3.0, size=3).astype(np.float32)
            lateral -= np.dot(lateral, direction) * direction  # remove along-path component
            centre = centre + lateral
            radius = float(rng.uniform(1.5, 4.0))
            self._obstacles.append(Obstacle(position=centre, radius=radius))

        # Update episode length
        self._max_steps = max_steps

        logger.info(
            "Difficulty updated: goal_dist=%.1f, obstacles=%d, max_steps=%d",
            goal_distance,
            num_obstacles,
            max_steps,
        )
        return self.reset()

    @property
    def state_dim(self) -> int:
        return 8

    @property
    def action_dim(self) -> int:
        return 3

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _compute_obstacle_distance(self) -> float:
        """Return the minimum surface distance to any obstacle."""
        if not self._obstacles:
            return 50.0

        min_dist = 50.0
        for obs in self._obstacles:
            centre_dist = float(np.linalg.norm(self._position - obs.position))
            surface_dist = max(centre_dist - obs.radius, 0.2)
            min_dist = min(min_dist, surface_dist)
        return float(np.clip(min_dist, 0.2, 50.0))
