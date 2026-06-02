"""BaseNavigationEnv wrapper for guarded Isaac Sim runtime access."""

from __future__ import annotations

import logging
from typing import Any, Dict, Tuple

import numpy as np

from src.digital_twin import IsaacSimRuntime
from src.env.base_env import BaseNavigationEnv
from src.utils.data_types import SensorObservation

logger = logging.getLogger(__name__)


class IsaacSimNavigationEnv(BaseNavigationEnv):
    """Navigation environment backed by ``IsaacSimRuntime``.

    This wrapper is mock-first: normal tests inject a fake runtime/backend and
    no Isaac Sim import is required. Real Isaac launch remains opt-in through
    the runtime object or config.
    """

    def __init__(
        self,
        descriptor: Dict[str, Any] | str | None = None,
        runtime: IsaacSimRuntime | None = None,
        config: Dict[str, Any] | None = None,
    ) -> None:
        self.config = dict(config or {})
        self.runtime = runtime or IsaacSimRuntime(self.config.get("runtime"))
        self.descriptor = descriptor or self.config.get("descriptor")
        self._goal = np.asarray(self.config.get("goal", (0.0, 0.0, 0.0)), dtype=np.float32)
        self._control_dt = float(self.config.get("control_dt", self.config.get("timestep", 0.05)))
        self._goal_reach_dist = float(self.config.get("goal_reach_dist", 3.0))
        self._collision_dist = float(self.config.get("collision_dist", 0.5))
        self._max_steps = int(self.config.get("max_steps", 600))
        self._launch_on_reset = bool(self.config.get("launch_on_reset", False))
        self._step_count = 0
        self._current_observation: SensorObservation | None = None
        self._descriptor_loaded = False

    def reset(self) -> SensorObservation:
        """Reset bookkeeping, connect the runtime, and return an observation."""
        if not self.runtime.is_connected:
            if self._launch_on_reset:
                self.runtime.launch(headless=self.config.get("headless", True))
            else:
                self.runtime.connect()

        if self.descriptor is not None and not self._descriptor_loaded:
            loaded = self.runtime.load_descriptor(self.descriptor)
            self._descriptor_loaded = True
            self._set_goal_from_descriptor(loaded)

        self._step_count = 0
        self._current_observation = self.get_observation()
        return self._current_observation

    def step(
        self, action: np.ndarray
    ) -> Tuple[SensorObservation, float, bool, Dict[str, Any]]:
        """Execute a velocity action and return ``(obs, reward, done, info)``."""
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size != self.action_dim:
            raise ValueError(f"IsaacSimNavigationEnv action must have shape ({self.action_dim},)")

        diagnostics = self.runtime.step(action=action.tolist(), dt=self._control_dt)
        self._step_count += 1
        obs = self.get_observation()
        self._current_observation = obs

        goal_reached = obs.goal_distance < self._goal_reach_dist
        collision = obs.obstacle_distance < self._collision_dist
        timeout = self._step_count >= self._max_steps
        done = bool(goal_reached or collision or timeout)
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
            "runtime": diagnostics,
        }
        return obs, float(reward), done, info

    def get_observation(self) -> SensorObservation:
        """Return the latest runtime observation without advancing simulation."""
        snapshot = self.runtime.read_sensors()
        obs = self.runtime.to_sensor_observation(snapshot)
        if obs.goal_distance == 0.0 and np.linalg.norm(self._goal) > 0.0:
            pose = np.asarray(obs.pose, dtype=np.float32)
            obs.goal_distance = float(np.linalg.norm(pose - self._goal))
        return obs

    def get_state_vector(self) -> np.ndarray:
        """Return the current 8D state vector."""
        if self._current_observation is None:
            self._current_observation = self.get_observation()
        return self._current_observation.to_state_vector()

    def close(self) -> None:
        """Release runtime resources."""
        self.runtime.close()

    @property
    def state_dim(self) -> int:
        return 8

    @property
    def action_dim(self) -> int:
        return 3

    def _set_goal_from_descriptor(self, descriptor: Dict[str, Any]) -> None:
        goal = descriptor.get("goal", {}) if isinstance(descriptor, dict) else {}
        position = goal.get("position")
        if position is not None:
            self._goal = np.asarray(position, dtype=np.float32).reshape(-1)[:3]
