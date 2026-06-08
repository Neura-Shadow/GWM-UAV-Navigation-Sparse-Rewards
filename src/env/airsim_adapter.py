"""Guarded Cosys-AirSim primary / legacy AirSim fallback navigation environment."""

from __future__ import annotations

from typing import Any, Dict, Tuple

import numpy as np

from src.digital_twin.airsim_runtime import AirSimRuntime
from src.env.base_env import BaseNavigationEnv
from src.utils.data_types import SensorObservation


class AirSimNavigationEnv(BaseNavigationEnv):
    """Navigation environment backed by ``AirSimRuntime``.

    The backend registry key remains ``airsim``. Cosys-AirSim / ``cosysairsim``
    is preferred, while legacy AirSim / ``airsim`` is retained as an optional
    fallback. The class is import-safe without either package installed. Tests
    can inject a fake runtime or fake client through ``AirSimRuntime``.
    """

    def __init__(
        self,
        goal: Tuple[float, float, float] = (60.0, 20.0, -8.0),
        target_altitude: float = -8.0,
        control_dt: float = 0.4,
        min_obstacle_dist: float = 4.0,
        goal_reach_dist: float = 3.0,
        lidar_name: str = "LidarSensor1",
        host: str | None = None,
        port: int = 41451,
        vehicle_name: str = "",
        runtime: AirSimRuntime | None = None,
        config: Dict[str, Any] | None = None,
    ) -> None:
        runtime_config = {
            "goal": goal,
            "target_altitude": target_altitude,
            "control_dt": control_dt,
            "lidar_name": lidar_name,
            "host": "127.0.0.1" if host is None else host,
            "port": port,
            "vehicle_name": vehicle_name,
        }
        runtime_config.update(config or {})
        self.runtime = runtime or AirSimRuntime(runtime_config)
        self.config = runtime_config
        self._goal = np.asarray(runtime_config.get("goal", goal), dtype=np.float32)
        self._control_dt = float(runtime_config.get("control_dt", control_dt))
        self._min_obstacle_dist = float(runtime_config.get("min_obstacle_dist", min_obstacle_dist))
        self._goal_reach_dist = float(runtime_config.get("goal_reach_dist", goal_reach_dist))
        self._max_steps = int(runtime_config.get("max_steps", 600))
        self._step_count = 0
        self._current_observation: SensorObservation | None = None

    def reset(self) -> SensorObservation:
        """Connect the runtime if needed and return the initial observation."""
        if not self.runtime.is_connected:
            self.runtime.connect()
        self._step_count = 0
        self._current_observation = self.runtime.reset()
        return self._current_observation

    def step(
        self, action: np.ndarray
    ) -> Tuple[SensorObservation, float, bool, Dict[str, Any]]:
        """Execute a velocity command and return ``(obs, reward, done, info)``."""
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size != self.action_dim:
            raise ValueError(f"AirSimNavigationEnv action must have shape ({self.action_dim},)")

        diagnostics = self.runtime.step(action[:3], dt=self._control_dt)
        self._step_count += 1
        obs = self.get_observation()
        self._current_observation = obs

        goal_reached = obs.goal_distance < self._goal_reach_dist
        collision = obs.obstacle_distance < self._min_obstacle_dist
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
        obs = self.runtime.to_sensor_observation(self.runtime.read_sensors())
        if obs.goal_distance == 0.0 and np.linalg.norm(self._goal) > 0.0:
            obs.goal_distance = float(np.linalg.norm(np.asarray(obs.pose, dtype=np.float32) - self._goal))
        return obs

    def get_state_vector(self) -> np.ndarray:
        """Return the current 8D state vector."""
        if self._current_observation is None:
            self._current_observation = self.get_observation()
        return self._current_observation.to_state_vector()

    def close(self) -> None:
        self.runtime.close()

    @property
    def state_dim(self) -> int:
        return 8

    @property
    def action_dim(self) -> int:
        return 3
