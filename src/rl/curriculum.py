"""Sparse reward curriculum scheduler.

Progressively increases navigation difficulty (goal distance, obstacle count,
episode length) based on rolling success rate.  Designed for the mock
environment but applicable to any ``BaseNavigationEnv`` that supports
``update_difficulty``.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class CurriculumConfig:
    """Configuration for the curriculum scheduler.

    Attributes
    ----------
    initial_goal_distance:
        Starting goal distance for easiest difficulty [m].
    max_goal_distance:
        Maximum goal distance at hardest difficulty [m].
    goal_distance_step:
        How much to increase goal distance per level [m].
    initial_obstacle_count:
        Starting number of obstacles.
    max_obstacle_count:
        Maximum obstacle count at hardest difficulty.
    obstacle_count_step:
        How many obstacles to add per level.
    initial_max_steps:
        Starting episode length.
    max_max_steps:
        Maximum episode length at hardest difficulty.
    steps_increment:
        How many steps to add per level.
    success_rate_threshold:
        Advance to next level when rolling success rate ≥ this value.
    failure_rate_threshold:
        Regress to previous level when rolling success rate ≤ this value.
    window_size:
        Number of episodes to compute rolling success rate over.
    """

    # Goal distance progression
    initial_goal_distance: float = 15.0
    max_goal_distance: float = 65.0
    goal_distance_step: float = 5.0

    # Obstacle count progression
    initial_obstacle_count: int = 0
    max_obstacle_count: int = 6
    obstacle_count_step: int = 1

    # Max steps progression
    initial_max_steps: int = 100
    max_max_steps: int = 600
    steps_increment: int = 50

    # Success/failure thresholds
    success_rate_threshold: float = 0.7
    failure_rate_threshold: float = 0.3
    window_size: int = 10


class CurriculumScheduler:
    """Tracks episode outcomes and adjusts difficulty level.

    Level 0 is easiest.  Each level increase bumps goal distance,
    obstacle count, and max steps by their configured step sizes.
    The scheduler only evaluates difficulty changes once the rolling
    window is full (``window_size`` episodes observed).

    Parameters
    ----------
    config:
        Curriculum hyper-parameters.  Uses defaults if ``None``.
    """

    def __init__(self, config: Optional[CurriculumConfig] = None) -> None:
        self.config = config or CurriculumConfig()
        self._level: int = 0
        self._max_level: int = self._compute_max_level()
        self._history: deque[bool] = deque(maxlen=self.config.window_size)
        logger.info(
            "CurriculumScheduler initialised: max_level=%d, window=%d",
            self._max_level,
            self.config.window_size,
        )

    # ------------------------------------------------------------------
    # Level computation
    # ------------------------------------------------------------------

    def _compute_max_level(self) -> int:
        """Compute the maximum difficulty level from config ranges."""
        cfg = self.config
        goal_levels = int(
            (cfg.max_goal_distance - cfg.initial_goal_distance)
            / max(cfg.goal_distance_step, 1e-6)
        )
        obs_levels = int(
            (cfg.max_obstacle_count - cfg.initial_obstacle_count)
            / max(cfg.obstacle_count_step, 1)
        )
        step_levels = int(
            (cfg.max_max_steps - cfg.initial_max_steps)
            / max(cfg.steps_increment, 1)
        )
        return max(1, min(goal_levels, obs_levels, step_levels))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def report_episode(self, success: bool) -> None:
        """Record episode outcome and potentially adjust difficulty.

        Difficulty only changes once the rolling window is full:
        - Advances if ``success_rate ≥ success_rate_threshold``
        - Regresses if ``success_rate ≤ failure_rate_threshold``
        - Stays the same otherwise
        After a level change, the history window is cleared.
        """
        self._history.append(success)
        if len(self._history) < self.config.window_size:
            return

        rate = self.success_rate

        if rate >= self.config.success_rate_threshold and self._level < self._max_level:
            self._level += 1
            self._history.clear()
            logger.info(
                "Curriculum level UP -> %d (success_rate=%.2f)",
                self._level,
                rate,
            )
        elif rate <= self.config.failure_rate_threshold and self._level > 0:
            self._level -= 1
            self._history.clear()
            logger.info(
                "Curriculum level DOWN -> %d (success_rate=%.2f)",
                self._level,
                rate,
            )

    @property
    def success_rate(self) -> float:
        """Current rolling success rate in [0, 1]."""
        if not self._history:
            return 0.0
        return sum(self._history) / len(self._history)

    @property
    def level(self) -> int:
        """Current difficulty level (0 = easiest)."""
        return self._level

    @property
    def max_level(self) -> int:
        """Maximum achievable difficulty level."""
        return self._max_level

    def get_goal_distance(self) -> float:
        """Goal distance for the current level [m]."""
        return min(
            self.config.initial_goal_distance
            + self._level * self.config.goal_distance_step,
            self.config.max_goal_distance,
        )

    def get_obstacle_count(self) -> int:
        """Obstacle count for the current level."""
        return min(
            self.config.initial_obstacle_count
            + self._level * self.config.obstacle_count_step,
            self.config.max_obstacle_count,
        )

    def get_max_steps(self) -> int:
        """Episode length for the current level."""
        return min(
            self.config.initial_max_steps
            + self._level * self.config.steps_increment,
            self.config.max_max_steps,
        )

    def get_env_overrides(self) -> Dict[str, Any]:
        """Return parameters to pass to ``MockNavigationEnv.update_difficulty()``."""
        return {
            "goal_distance": self.get_goal_distance(),
            "num_obstacles": self.get_obstacle_count(),
            "max_steps": self.get_max_steps(),
        }

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def state_dict(self) -> Dict[str, Any]:
        """Serialize scheduler state for checkpointing."""
        return {
            "level": self._level,
            "history": list(self._history),
        }

    def load_state_dict(self, state: Dict[str, Any]) -> None:
        """Restore scheduler state from a checkpoint."""
        self._level = state.get("level", 0)
        self._history = deque(
            state.get("history", []), maxlen=self.config.window_size
        )
