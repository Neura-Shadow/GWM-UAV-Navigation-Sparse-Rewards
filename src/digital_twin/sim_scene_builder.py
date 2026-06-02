"""Simulation scene builder for the digital-twin pipeline.

Converts ``ScenarioSpec`` instances into simulator-ready scene descriptors
(JSON / YAML dicts).  The output format is intentionally simulator-agnostic;
future integration with NVIDIA Isaac Sim / OpenUSD will read these
descriptors and instantiate the actual 3D scene.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

from src.utils.data_types import ScenarioSpec

logger = logging.getLogger(__name__)

# Default sensor configuration baked into every scene
_DEFAULT_SENSOR_CONFIG: Dict[str, Any] = {
    "lidar_channels": 16,
    "lidar_range": 50.0,
    "camera_fov": 90,
    "depth_enabled": True,
}

# Default world bounds: (x_min, y_min, z_min, x_max, y_max, z_max)
_DEFAULT_WORLD_BOUNDS = (-100.0, -100.0, -50.0, 100.0, 100.0, 0.0)


class SimSceneBuilder:
    """Builds simulation scene specifications from ScenarioSpecs.

    Future integration: Isaac Sim / OpenUSD scene generation.
    Current implementation: outputs JSON/YAML scene descriptors.
    """

    def __init__(
        self,
        world_bounds: tuple = _DEFAULT_WORLD_BOUNDS,
        sensor_config: Dict[str, Any] | None = None,
        backend: str = "mock",
    ) -> None:
        self.world_bounds = world_bounds
        self.sensor_config = sensor_config or dict(_DEFAULT_SENSOR_CONFIG)
        self.backend = backend
        logger.info("SimSceneBuilder initialised with bounds=%s", self.world_bounds)

    @classmethod
    def create(
        cls,
        backend: str = "mock",
        world_bounds: tuple = _DEFAULT_WORLD_BOUNDS,
        sensor_config: Dict[str, Any] | None = None,
    ):
        """Create a scene builder for the requested backend."""
        if backend == "mock":
            return cls(world_bounds=world_bounds, sensor_config=sensor_config, backend=backend)
        if backend == "isaac_sim":
            from src.digital_twin.isaac_sim_builder import IsaacSimSceneBuilder

            return IsaacSimSceneBuilder(world_bounds=world_bounds, sensor_config=sensor_config)
        raise ValueError(f"Unsupported scene builder backend: {backend}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self, spec: ScenarioSpec) -> Dict[str, Any]:
        """Convert a ``ScenarioSpec`` into a simulation-ready scene descriptor.

        Returns a nested dict with the following top-level keys:

        - ``scenario_id``
        - ``description``
        - ``world_bounds`` — 6-element list
        - ``spawn_point`` — xyz dict
        - ``goal`` — xyz dict
        - ``obstacles`` — list of obstacle dicts
        - ``environment`` — weather + physics
        - ``sensors`` — sensor configuration
        """
        scene: Dict[str, Any] = {
            "scenario_id": spec.scenario_id,
            "description": spec.description,
            "world_bounds": list(self.world_bounds),
            "spawn_point": {
                "x": spec.start_position[0],
                "y": spec.start_position[1],
                "z": spec.start_position[2],
            },
            "goal": {
                "x": spec.goal_position[0],
                "y": spec.goal_position[1],
                "z": spec.goal_position[2],
            },
            "obstacles": list(spec.obstacles),
            "environment": {
                "weather": dict(spec.weather),
                "physics": dict(spec.physics),
                "sensor_noise": dict(spec.sensor_noise),
            },
            "sensors": dict(self.sensor_config),
        }
        logger.debug("Built scene descriptor for scenario '%s'.", spec.scenario_id)
        return scene

    def to_yaml(self, scene: Dict[str, Any], output_path: str) -> None:
        """Save scene descriptor to a YAML file."""
        if yaml is None:
            raise ImportError("PyYAML is required to write YAML files.")
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            yaml.dump(scene, fh, default_flow_style=False, sort_keys=False)
        logger.info("Scene descriptor written to %s (YAML).", path)

    def to_json(self, scene: Dict[str, Any], output_path: str) -> None:
        """Save scene descriptor to a JSON file."""
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(scene, fh, indent=2)
        logger.info("Scene descriptor written to %s (JSON).", path)
