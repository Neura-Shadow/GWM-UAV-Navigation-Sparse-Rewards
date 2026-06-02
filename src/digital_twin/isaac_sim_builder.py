"""Mock-first Isaac Sim / OpenUSD scene descriptor builder."""

from __future__ import annotations

import json
import logging
import copy
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

from src.digital_twin.mock_isaac_sim import MockUSDStage
from src.utils.data_types import ScenarioSpec

logger = logging.getLogger(__name__)

try:  # pragma: no cover - exercised only in Omniverse environments
    import omni.isaac.core  # noqa: F401

    _HAS_ISAAC_SIM = True
except ImportError:  # pragma: no cover - default in CI/dev environments
    _HAS_ISAAC_SIM = False

try:  # pragma: no cover - exercised only when OpenUSD Python bindings exist
    from pxr import Usd, UsdGeom

    _HAS_OPENUSD = True
except ImportError:  # pragma: no cover - default in CI/dev environments
    Usd = None  # type: ignore[assignment]
    UsdGeom = None  # type: ignore[assignment]
    _HAS_OPENUSD = False


_DEFAULT_SENSOR_CONFIG: Dict[str, Any] = {
    "lidar": {
        "type": "lidar",
        "range": 50.0,
        "channels": 16,
    },
    "depth_camera": {
        "type": "depth_camera",
        "fov": 90.0,
        "enabled": True,
    },
    "imu": {
        "type": "imu",
        "enabled": True,
    },
}

_DEFAULT_WORLD_BOUNDS = (-100.0, -100.0, -50.0, 100.0, 100.0, 0.0)


class IsaacSimSceneBuilder:
    """Build OpenUSD-style scene descriptors from ``ScenarioSpec`` objects.

    ``build()`` is pure Python and always works without Isaac Sim. Real USD
    stage export is guarded behind ``build_usd_stage()``.
    """

    def __init__(
        self,
        world_bounds: Sequence[float] = _DEFAULT_WORLD_BOUNDS,
        sensor_config: Dict[str, Any] | None = None,
        stage_units: float = 1.0,
    ) -> None:
        self.world_bounds = tuple(float(v) for v in world_bounds)
        self.sensor_config = self._normalize_sensor_config(sensor_config)
        self.stage_units = float(stage_units)
        logger.info("IsaacSimSceneBuilder initialised (stage_units=%.3f)", self.stage_units)

    def build(self, spec: ScenarioSpec) -> Dict[str, Any]:
        """Return a JSON-serializable OpenUSD-style descriptor."""
        obstacles = self._build_obstacles(spec.obstacles)
        sensors = self._build_sensors(self.sensor_config)
        prims = [
            self._prim("/World", "Xform", purpose="root"),
            self._prim("/World/GroundPlane", "Plane", purpose="ground"),
            self._prim("/World/Obstacles", "Xform", purpose="obstacle_group"),
            self._prim("/World/Vehicle", "Xform", purpose="vehicle_group"),
            self._prim("/World/Vehicle/UAV", "Xform", purpose="vehicle"),
            self._prim("/World/Vehicle/UAV/Sensors", "Xform", purpose="sensor_group"),
            self._prim("/World/Goal", "Xform", purpose="goal"),
        ]
        prims.extend(obstacles)
        prims.extend(sensors)

        descriptor: Dict[str, Any] = {
            "schema_version": "gwm_openusd_descriptor_v1",
            "backend": "isaac_sim_descriptor",
            "scenario_id": spec.scenario_id,
            "description": spec.description,
            "metadata": {
                "source_coordinate_frame": "project_default",
                "target_coordinate_frame": "isaac_z_up_pending",
                "coordinate_conversion_applied": False,
                "stage_units": self.stage_units,
                "generator": "IsaacSimSceneBuilder",
            },
            "stage": {
                "root_path": "/World",
                "default_prim": "/World",
                "units": self.stage_units,
            },
            "world": {
                "path": "/World",
                "bounds": list(self.world_bounds),
            },
            "ground": {
                "path": "/World/GroundPlane",
                "prim_type": "Plane",
                "position": [0.0, 0.0, 0.0],
                "size": self._ground_size(),
            },
            "obstacles": obstacles,
            "vehicle": {
                "path": "/World/Vehicle/UAV",
                "prim_type": "Xform",
                "vehicle_type": "uav",
                "position": self._vector(spec.start_position),
                "sensors_path": "/World/Vehicle/UAV/Sensors",
                "sensors": sensors,
            },
            "goal": {
                "path": "/World/Goal",
                "prim_type": "Xform",
                "position": self._vector(spec.goal_position),
            },
            "environment": {
                "weather": dict(spec.weather),
                "physics": dict(spec.physics),
                "sensor_noise": dict(spec.sensor_noise),
            },
            "prims": prims,
        }
        logger.debug("Built Isaac descriptor for scenario '%s'", spec.scenario_id)
        return descriptor

    def build_usd_stage(self, spec: ScenarioSpec, output_path: str) -> Any:
        """Create a real USD stage when Isaac Sim/OpenUSD APIs are available."""
        if not (_HAS_ISAAC_SIM and _HAS_OPENUSD):
            raise RuntimeError(
                "build_usd_stage requires Isaac Sim / OpenUSD Python APIs. "
                "Use build() or export_json() for descriptor JSON without Isaac Sim."
            )

        descriptor = self.build(spec)
        stage = Usd.Stage.CreateNew(output_path)
        UsdGeom.SetStageMetersPerUnit(stage, self.stage_units)
        for prim in descriptor["prims"]:
            stage.DefinePrim(prim["path"], prim["prim_type"])
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        stage.GetRootLayer().Save()
        return stage

    def export_json(self, descriptor: Dict[str, Any], output_path: str) -> None:
        """Write a descriptor produced by ``build()`` to JSON."""
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(descriptor, fh, indent=2)

    def populate_mock_stage(self, stage: MockUSDStage, descriptor: Dict[str, Any]) -> MockUSDStage:
        """Populate a mock stage from a descriptor for tests and offline tooling."""
        for prim_def in descriptor["prims"]:
            prim = stage.DefinePrim(prim_def["path"], prim_def["prim_type"])
            for key, value in prim_def.get("attributes", {}).items():
                prim.set_attribute(key, value)
        return stage

    def add_obstacles(self, stage: MockUSDStage, obstacles: Iterable[Dict[str, Any]]) -> None:
        """Add obstacle prims to a mock stage."""
        for obstacle in obstacles:
            prim = stage.DefinePrim(obstacle["path"], obstacle["prim_type"])
            prim.set_attribute("position", obstacle["attributes"]["position"])
            prim.set_attribute("size", obstacle["attributes"]["size"])

    def add_vehicle(self, stage: MockUSDStage, spawn_point: Sequence[float]) -> None:
        """Add the UAV prim to a mock stage."""
        prim = stage.DefinePrim("/World/Vehicle/UAV", "Xform")
        prim.set_attribute("position", self._vector(spawn_point))

    def add_sensors(self, stage: MockUSDStage, sensor_config: Dict[str, Any]) -> None:
        """Add configured sensor prims to a mock stage."""
        for sensor in self._build_sensors(sensor_config):
            prim = stage.DefinePrim(sensor["path"], sensor["prim_type"])
            prim.set_attribute("sensor", sensor["attributes"])

    def configure_physics(self, stage: MockUSDStage, physics_config: Dict[str, Any]) -> None:
        """Record physics configuration on the root prim in a mock stage."""
        prim = stage.GetPrimAtPath("/World") or stage.DefinePrim("/World", "Xform")
        prim.set_attribute("physics", dict(physics_config))

    def _build_obstacles(self, obstacles: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
        result: List[Dict[str, Any]] = []
        for idx, obstacle in enumerate(obstacles):
            obstacle_type = str(obstacle.get("type", "sphere")).lower()
            prim_type = {
                "sphere": "Sphere",
                "box": "Cube",
                "cube": "Cube",
                "cylinder": "Cylinder",
            }.get(obstacle_type, "Xform")
            size = obstacle.get("radius", obstacle.get("size", 1.0))
            result.append(
                self._prim(
                    f"/World/Obstacles/Obstacle_{idx}",
                    prim_type,
                    purpose="obstacle",
                    obstacle_type=obstacle_type,
                    position=self._vector(obstacle.get("position", (0.0, 0.0, 0.0))),
                    size=self._json_value(size),
                    material=obstacle.get("material", "default"),
                    source_index=idx,
                )
            )
        return result

    def _build_sensors(self, sensor_config: Dict[str, Any]) -> List[Dict[str, Any]]:
        sensors: List[Dict[str, Any]] = []
        for name, config in sensor_config.items():
            if isinstance(config, dict) and config.get("enabled", True) is False:
                continue
            sensor_type = config.get("type", name) if isinstance(config, dict) else name
            sensors.append(
                self._prim(
                    f"/World/Vehicle/UAV/Sensors/{self._sensor_prim_name(name)}",
                    "Sensor",
                    purpose="sensor",
                    sensor_type=sensor_type,
                    config=dict(config) if isinstance(config, dict) else {"value": config},
                )
            )
        return sensors

    def _ground_size(self) -> List[float]:
        x_min, y_min, _z_min, x_max, y_max, _z_max = self.world_bounds
        return [abs(x_max - x_min), abs(y_max - y_min)]

    @staticmethod
    def _prim(path: str, prim_type: str, **attributes: Any) -> Dict[str, Any]:
        return {
            "path": path,
            "prim_type": prim_type,
            "attributes": attributes,
        }

    @staticmethod
    def _sensor_prim_name(name: str) -> str:
        return "".join(part.capitalize() for part in name.split("_"))

    @staticmethod
    def _normalize_sensor_config(sensor_config: Dict[str, Any] | None) -> Dict[str, Any]:
        if sensor_config is None:
            return copy.deepcopy(_DEFAULT_SENSOR_CONFIG)

        legacy_keys = {"lidar_channels", "lidar_range", "camera_fov", "depth_enabled"}
        if legacy_keys.intersection(sensor_config):
            return {
                "lidar": {
                    "type": "lidar",
                    "channels": int(sensor_config.get("lidar_channels", 16)),
                    "range": float(sensor_config.get("lidar_range", 50.0)),
                },
                "depth_camera": {
                    "type": "depth_camera",
                    "fov": float(sensor_config.get("camera_fov", 90.0)),
                    "enabled": bool(sensor_config.get("depth_enabled", True)),
                },
                "imu": {
                    "type": "imu",
                    "enabled": bool(sensor_config.get("imu_enabled", True)),
                },
            }

        return copy.deepcopy(sensor_config)

    @classmethod
    def _vector(cls, value: Sequence[float]) -> List[float]:
        return [float(v) for v in value]

    @classmethod
    def _json_value(cls, value: Any) -> Any:
        if isinstance(value, (list, tuple)):
            return [cls._json_value(v) for v in value]
        if isinstance(value, (int, float, str, bool)) or value is None:
            return value
        return str(value)
