"""Domain randomisation for simulation scenarios.

Generates randomised variations of a base ``ScenarioSpec`` by perturbing
obstacle positions, weather conditions, sensor noise, physics parameters,
obstacle counts, and lighting conditions.  Deterministic when seeded.

Extended in Phase 2-C with:
- Configurable lighting / time-of-day randomisation
- ``to_dict`` / ``to_yaml`` serialisation helpers
- ``from_config`` factory method for YAML-driven construction
"""

from __future__ import annotations

import copy
import io
import logging
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

from src.utils.data_types import ScenarioSpec

logger = logging.getLogger(__name__)


class DomainRandomizer:
    """Generates randomised variations of simulation scenarios.

    Randomises: obstacle positions, weather conditions, sensor noise,
    surface friction, wind speed/direction, lighting, and time-of-day.

    Parameters
    ----------
    seed:
        RNG seed for reproducibility.
    obstacle_position_noise:
        Std-dev of Gaussian noise added to obstacle positions [m].
    obstacle_count_range:
        ``(min, max)`` number of extra obstacles to add.
    wind_speed_range:
        ``(min, max)`` wind speed [m/s].
    sensor_noise_range:
        ``(min, max)`` per-sensor noise scale.
    friction_range:
        ``(min, max)`` surface friction coefficient.
    lighting_time_range:
        ``(min, max)`` time of day [hours, 0–24].
    lighting_intensity_range:
        ``(min, max)`` light intensity multiplier.
    lighting_color_temp_range:
        ``(min, max)`` colour temperature [K].
    """

    def __init__(
        self,
        seed: Optional[int] = None,
        obstacle_position_noise: float = 2.0,
        obstacle_count_range: Tuple[int, int] = (0, 3),
        wind_speed_range: Tuple[float, float] = (0.0, 5.0),
        sensor_noise_range: Tuple[float, float] = (0.0, 0.1),
        friction_range: Tuple[float, float] = (0.5, 1.5),
        lighting_time_range: Tuple[float, float] = (6.0, 18.0),
        lighting_intensity_range: Tuple[float, float] = (0.5, 1.5),
        lighting_color_temp_range: Tuple[int, int] = (3000, 7000),
    ) -> None:
        self.rng = np.random.default_rng(seed)

        self.obstacle_position_noise = obstacle_position_noise
        self.obstacle_count_range = obstacle_count_range
        self.wind_speed_range = wind_speed_range
        self.sensor_noise_range = sensor_noise_range
        self.friction_range = friction_range
        self.lighting_time_range = lighting_time_range
        self.lighting_intensity_range = lighting_intensity_range
        self.lighting_color_temp_range = lighting_color_temp_range

        logger.info(
            "DomainRandomizer initialised (seed=%s, obs_noise=%.1f, "
            "wind=[%.1f,%.1f], lighting_time=[%.1f,%.1f]).",
            seed,
            obstacle_position_noise,
            *wind_speed_range,
            *lighting_time_range,
        )

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "DomainRandomizer":
        """Create a DomainRandomizer from a configuration dict.

        Expected structure (all keys optional)::

            domain_randomization:
              seed: 42
              obstacle_position_noise: 2.0
              obstacle_count_range: [0, 3]
              wind_speed_range: [0.0, 8.0]
              sensor_noise_range: [0.0, 0.15]
              friction_range: [0.5, 1.5]
              lighting:
                time_of_day_range: [6.0, 18.0]
                intensity_range: [0.5, 1.5]
                color_temp_range: [3000, 7000]
        """
        dr = config.get("domain_randomization", config)
        lighting = dr.get("lighting", {})
        return cls(
            seed=dr.get("seed"),
            obstacle_position_noise=dr.get("obstacle_position_noise", 2.0),
            obstacle_count_range=tuple(dr.get("obstacle_count_range", [0, 3])),
            wind_speed_range=tuple(dr.get("wind_speed_range", [0.0, 5.0])),
            sensor_noise_range=tuple(dr.get("sensor_noise_range", [0.0, 0.1])),
            friction_range=tuple(dr.get("friction_range", [0.5, 1.5])),
            lighting_time_range=tuple(
                lighting.get("time_of_day_range", [6.0, 18.0])
            ),
            lighting_intensity_range=tuple(
                lighting.get("intensity_range", [0.5, 1.5])
            ),
            lighting_color_temp_range=tuple(
                lighting.get("color_temp_range", [3000, 7000])
            ),
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def randomize(
        self, base_spec: ScenarioSpec, num_variations: int = 5
    ) -> List[ScenarioSpec]:
        """Generate *num_variations* randomised copies of *base_spec*.

        Each variation receives:
        - Perturbed obstacle positions
        - Possibly added / removed obstacles
        - Randomised weather (wind, fog, rain)
        - Randomised sensor noise
        - Randomised physics (friction, drag)
        - Randomised lighting (time-of-day, intensity, colour temp)
        - A unique ``scenario_id``
        """
        variations: List[ScenarioSpec] = []
        for i in range(num_variations):
            spec = copy.deepcopy(base_spec)
            spec.scenario_id = f"{base_spec.scenario_id}_var{i}_{uuid.uuid4().hex[:6]}"
            spec.description = f"Randomised variation {i} of {base_spec.scenario_id}"

            spec.obstacles = self._randomize_obstacles(spec.obstacles)
            spec.weather = self._randomize_weather()
            spec.physics = self._randomize_physics()
            spec.sensor_noise = self._randomize_sensor_noise()
            spec.metadata["lighting"] = self._randomize_lighting()

            variations.append(spec)

        logger.info(
            "Generated %d variations from base scenario '%s'.",
            len(variations),
            base_spec.scenario_id,
        )
        return variations

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    @staticmethod
    def to_dict(spec: ScenarioSpec) -> Dict[str, Any]:
        """Convert a ScenarioSpec to a plain dict for JSON/YAML output."""
        return {
            "scenario_id": spec.scenario_id,
            "description": spec.description,
            "start_position": list(spec.start_position),
            "goal_position": list(spec.goal_position),
            "obstacles": spec.obstacles,
            "weather": spec.weather,
            "sensor_noise": spec.sensor_noise,
            "physics": spec.physics,
            "metadata": spec.metadata,
        }

    @classmethod
    def to_yaml(cls, specs: List[ScenarioSpec], path: str) -> None:
        """Write a list of ScenarioSpecs to a YAML file.

        Raises ImportError if pyyaml is not installed.
        """
        if yaml is None:
            raise ImportError("pyyaml is required for YAML output.")
        data = {"scenarios": [cls.to_dict(s) for s in specs]}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        logger.info("Wrote %d scenarios to %s", len(specs), path)

    @classmethod
    def to_yaml_string(cls, specs: List[ScenarioSpec]) -> str:
        """Serialise a list of ScenarioSpecs to a YAML string."""
        if yaml is None:
            raise ImportError("pyyaml is required for YAML output.")
        data = {"scenarios": [cls.to_dict(s) for s in specs]}
        return yaml.dump(data, default_flow_style=False, sort_keys=False)

    # ------------------------------------------------------------------
    # Randomisation helpers
    # ------------------------------------------------------------------

    def _randomize_obstacles(
        self, obstacles: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Perturb existing obstacle positions and optionally add extras."""
        result: List[Dict[str, Any]] = []

        for obs in obstacles:
            new_obs = dict(obs)
            if "position" in new_obs:
                pos = np.asarray(new_obs["position"], dtype=np.float64)
                pos += self.rng.normal(0.0, self.obstacle_position_noise, size=pos.shape)
                new_obs["position"] = pos.tolist()
            result.append(new_obs)

        # Randomly add extra obstacles
        num_extra = int(self.rng.integers(
            self.obstacle_count_range[0], self.obstacle_count_range[1] + 1
        ))
        for _ in range(num_extra):
            result.append(self._generate_random_obstacle())

        return result

    def _generate_random_obstacle(self) -> Dict[str, Any]:
        """Create a random obstacle within plausible bounds."""
        return {
            "position": self.rng.uniform(-50.0, 50.0, size=3).tolist(),
            "size": float(self.rng.uniform(0.5, 3.0)),
            "type": "sphere",
        }

    def _randomize_weather(self) -> Dict[str, Any]:
        """Sample random weather conditions."""
        return {
            "wind_speed": float(self.rng.uniform(*self.wind_speed_range)),
            "wind_direction": float(self.rng.uniform(0.0, 360.0)),
            "fog_density": float(self.rng.uniform(0.0, 0.3)),
            "rain_intensity": float(self.rng.uniform(0.0, 1.0)),
        }

    def _randomize_physics(self) -> Dict[str, Any]:
        """Sample random physics parameters."""
        return {
            "friction": float(self.rng.uniform(*self.friction_range)),
            "drag_coefficient": float(self.rng.uniform(0.1, 0.5)),
        }

    def _randomize_sensor_noise(self) -> Dict[str, float]:
        """Sample per-sensor noise scales."""
        return {
            "lidar": float(self.rng.uniform(*self.sensor_noise_range)),
            "camera": float(self.rng.uniform(*self.sensor_noise_range)),
            "depth": float(self.rng.uniform(*self.sensor_noise_range)),
            "imu": float(self.rng.uniform(*self.sensor_noise_range)),
        }

    def _randomize_lighting(self) -> Dict[str, Any]:
        """Sample random lighting / time-of-day parameters."""
        return {
            "time_of_day": float(self.rng.uniform(*self.lighting_time_range)),
            "intensity": float(self.rng.uniform(*self.lighting_intensity_range)),
            "color_temperature": int(
                self.rng.integers(*self.lighting_color_temp_range)
            ),
        }
